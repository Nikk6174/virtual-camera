/*
 * capture_one.c - Step 1: capture ONE frame from the webcam using raw V4L2.
 *
 * WHAT IT DOES
 *   Opens the webcam (/dev/video0), asks it for YUYV 640x480 frames, maps the
 *   kernel's frame buffers into this process with mmap(), starts streaming,
 *   throws away the first few (dark) frames, and saves one frame to a file
 *   (frame.yuv). The file is raw pixels, with no header: 640*480*2 = 614,400 bytes.
 *
 * HOW IT CONNECTS TO THE REST OF THE PROJECT
 *   - Built by the top-level CMakeLists.txt as the "capture_one" program.
 *   - It is a standalone program for now. In Step 2 the same sequence of calls
 *     gets moved into a reusable capture module (capture/v4l2_capture.c + .h)
 *     that the SDL preview uses, and in Step 4 the pipeline's capture thread uses it.
 *   - Its output (frame.yuv) is what Step 2's YUYV->RGB converter learns to read.
 *
 * THE SEQUENCE (every arrow is a system call into the uvcvideo driver)
 *   open -> QUERYCAP -> S_FMT -> REQBUFS -> QUERYBUF + mmap (each buffer)
 *        -> QBUF (each buffer) -> STREAMON
 *        -> repeat: poll -> DQBUF -> use pixels -> QBUF
 *        -> STREAMOFF -> munmap -> REQBUFS(0) -> close
 *
 * Usage:  ./capture_one [device] [output_file]     (default: /dev/video0 frame.yuv)
 * View:   ffplay -f rawvideo -pixel_format yuyv422 -video_size 640x480 frame.yuv
 */

#include <errno.h>          /* errno, EINTR, EAGAIN */
#include <fcntl.h>          /* open(), O_RDWR, O_NONBLOCK */
#include <poll.h>           /* poll(), struct pollfd, POLLIN */
#include <stdint.h>         /* uint32_t */
#include <stdio.h>          /* printf, fopen, fwrite */
#include <stdlib.h>         /* EXIT_SUCCESS / EXIT_FAILURE */
#include <string.h>         /* memset, strerror */
#include <sys/ioctl.h>      /* ioctl() */
#include <sys/mman.h>       /* mmap(), munmap(), MAP_FAILED */
#include <unistd.h>         /* close() */

#include <linux/videodev2.h> /* every V4L2 struct, ioctl code and constant */

#define WIDTH        640
#define HEIGHT       480
#define NUM_BUFFERS  4     /* how many buffers we ASK for; the driver may give a different number */
#define WARMUP       10    /* frames to throw away while auto-exposure adjusts (first frames are dark) */
#define POLL_TIMEOUT 2000  /* ms to wait for a frame before giving up */

/*
 * One frame buffer. The memory belongs to the kernel driver; mmap() gives us
 * an address in OUR virtual address space that points at the same physical pages.
 */
struct buffer {
    void   *start;   /* where the buffer appears in our process (returned by mmap) */
    size_t  length;  /* size of the buffer in bytes (from QUERYBUF) */
};

/*
 * VIDEO_MAX_FRAME (32) is the most buffers V4L2 will ever hand out per queue.
 * Sizing the array to it means we never overflow even if the driver gives us
 * MORE buffers than we asked for (drivers are allowed to do that when they
 * need a minimum number to run the hardware).
 */
static struct buffer buffers[VIDEO_MAX_FRAME];
static unsigned int  n_buffers;      /* how many buffers the driver actually allocated */
static uint32_t      frame_bytes;    /* bytes per frame, as reported by the driver (sizeimage) */
static int           streaming;      /* 1 after STREAMON succeeded, so cleanup knows to STREAMOFF */

/*
 * ioctl() wrapper. If a signal arrives while we're inside the system call,
 * the kernel aborts it and returns -1 with errno == EINTR. Nothing was done,
 * so the correct reaction is simply to call it again.
 */
static int xioctl(int fd, unsigned long request, void *arg)
{
    int r;
    do {
        r = ioctl(fd, request, arg);          /* enter the kernel; driver handles 'request' */
    } while (r == -1 && errno == EINTR);      /* retry only on "interrupted by a signal" */
    return r;
}

/*
 * A V4L2 pixel format is a "fourcc": 4 ASCII characters packed into a 32-bit
 * number, lowest byte first. 'Y','U','Y','V' -> 0x56595559. This prints it back as text.
 */
static void print_fourcc(const char *label, uint32_t f)
{
    printf("%s%c%c%c%c\n", label,
           f & 0xff,            /* byte 0: first character */
           (f >> 8) & 0xff,     /* byte 1 */
           (f >> 16) & 0xff,    /* byte 2 */
           (f >> 24) & 0xff);   /* byte 3: last character */
}

/* ------------------------------------------------------------------------ */
/* 1. Ask the driver what this device is and what it can do.                */
/* ------------------------------------------------------------------------ */
static int query_caps(int fd)
{
    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));   /* always zero V4L2 structs: drivers reject garbage in reserved fields */

    /* VIDIOC_QUERYCAP: the driver fills 'cap' with its name, card name and capability bits. */
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
        perror("VIDIOC_QUERYCAP");  /* ENOTTY here means "this file isn't a V4L2 device" */
        return -1;
    }

    /* These are fixed-size char arrays filled by the driver (uvcvideo for USB webcams). */
    printf("driver:   %s\n", cap.driver);     /* kernel driver name, e.g. "uvcvideo" */
    printf("card:     %s\n", cap.card);       /* device name, e.g. "USB2.0 HD UVC WebCam" */
    printf("bus:      %s\n", cap.bus_info);   /* where it's plugged in, e.g. "usb-0000:05:00.3-4" */

    /*
     * Two capability fields exist because one physical device can create
     * several /dev/video nodes. cap.capabilities describes the WHOLE device
     * (all nodes together). cap.device_caps describes THIS node only.
     * Our webcam has /dev/video0 (video) and /dev/video1 (metadata); for
     * video1, 'capabilities' would still say VIDEO_CAPTURE (from video0), but
     * 'device_caps' correctly says only META_CAPTURE.
     * V4L2_CAP_DEVICE_CAPS tells us whether device_caps is filled in.
     */
    uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS)
                        ? cap.device_caps
                        : cap.capabilities;

    /* We need a node that produces video frames... */
    if (!(caps & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf(stderr, "device has no VIDEO_CAPTURE (is this the metadata node?)\n");
        return -1;
    }
    /* ...and supports streaming I/O (the QBUF/DQBUF + mmap method). */
    if (!(caps & V4L2_CAP_STREAMING)) {
        fprintf(stderr, "device does not support streaming I/O\n");
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------------ */
/* 2. Request YUYV 640x480. The driver answers with what it really set.     */
/* ------------------------------------------------------------------------ */
static int set_format(int fd)
{
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));

    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE; /* which part of 'fmt' is used: .pix (single-plane capture) */
    fmt.fmt.pix.width       = WIDTH;
    fmt.fmt.pix.height      = HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;           /* 2 bytes/pixel: Y0 U Y1 V covers 2 pixels */
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;             /* progressive frames, not interlaced halves */

    /*
     * VIDIOC_S_FMT is a negotiation, not a command: if the driver can't do
     * exactly what we asked, it picks the closest thing it CAN do and writes
     * that back into 'fmt'. It only fails (EINVAL/EBUSY) in bad cases, e.g.
     * another program is already streaming from the camera.
     */
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        perror("VIDIOC_S_FMT");     /* EBUSY = another app (Snapshot, browser) has the camera */
        return -1;
    }

    /* So we must read back the result instead of assuming we got our request. */
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
        print_fourcc("driver refused YUYV, gave: ", fmt.fmt.pix.pixelformat);
        return -1;
    }

    print_fourcc("format:   ", fmt.fmt.pix.pixelformat);
    printf("size:     %ux%u\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
    /* bytesperline = bytes in one row INCLUDING any padding the hardware adds at the end.
       For YUYV 640 wide that's 640*2 = 1280 with no padding. Later code must step
       rows by bytesperline, not by width*2, to be correct on cameras that pad. */
    printf("stride:   %u bytes/line\n", fmt.fmt.pix.bytesperline);
    /* sizeimage = total bytes the driver needs for one frame (bytesperline * height here). */
    printf("frame:    %u bytes\n", fmt.fmt.pix.sizeimage);

    frame_bytes = fmt.fmt.pix.sizeimage;
    return 0;
}

/* ------------------------------------------------------------------------ */
/* 3. Have the kernel allocate frame buffers, then map them into our memory.*/
/* ------------------------------------------------------------------------ */
static int init_mmap(int fd)
{
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));

    req.count  = NUM_BUFFERS;                  /* how many we'd like */
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;             /* driver allocates the memory; we mmap it.
                                                  (Alternatives: USERPTR = we give our own memory,
                                                   DMABUF = share buffers with another device like a GPU) */

    /* VIDIOC_REQBUFS: the driver allocates buffers inside the kernel and writes
       back in req.count how many it actually created. */
    if (xioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        perror("VIDIOC_REQBUFS");
        return -1;
    }

    /* We need at least 2: the camera fills one while we read the other.
       With 1, the camera would have nowhere to write while we're busy. */
    if (req.count < 2) {
        fprintf(stderr, "only got %u buffer(s), need at least 2\n", req.count);
        return -1;
    }
    n_buffers = req.count;
    printf("buffers:  %u (asked for %d)\n", n_buffers, NUM_BUFFERS);

    for (unsigned int i = 0; i < n_buffers; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;                         /* "tell me about buffer number i" */

        /* VIDIOC_QUERYBUF: the driver fills in buf.length (size) and
           buf.m.offset (a cookie identifying this buffer for mmap). */
        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            perror("VIDIOC_QUERYBUF");
            return -1;
        }

        buffers[i].length = buf.length;

        /*
         * mmap() creates a range of virtual addresses in our process that
         * point at the driver's buffer pages. No pixel data is copied, now
         * or later: when the camera writes a frame there, we can read it
         * directly at buffers[i].start.
         *
         * buf.m.offset is NOT a position inside a file. The device "file"
         * has no contents to seek through. The offset is a unique cookie
         * the driver invented so that, inside its mmap handler, it knows
         * WHICH of its buffers we want mapped.
         *
         * MAP_SHARED: required, because we want to see the driver's writes
         *   (MAP_PRIVATE would give us a private copy-on-write view).
         * PROT_READ | PROT_WRITE: the V4L2 docs recommend both, even though
         *   we only read.
         */
        buffers[i].start = mmap(NULL,                     /* let the kernel pick the address */
                                buf.length,               /* how many bytes to map */
                                PROT_READ | PROT_WRITE,
                                MAP_SHARED,
                                fd,                       /* the video device */
                                buf.m.offset);            /* which buffer (driver's cookie) */

        /* mmap signals failure with MAP_FAILED, which is (void *)-1, NOT NULL. */
        if (buffers[i].start == MAP_FAILED) {
            perror("mmap");
            buffers[i].start = NULL;  /* so cleanup() doesn't try to munmap it */
            return -1;
        }
        printf("  buffer %u: %zu bytes mapped at %p\n", i, buffers[i].length, buffers[i].start);
    }
    return 0;
}

/* ------------------------------------------------------------------------ */
/* 4. Give every empty buffer to the driver, then turn the camera on.       */
/* ------------------------------------------------------------------------ */
static int start_streaming(int fd)
{
    /*
     * The driver keeps two queues:
     *   incoming: empty buffers it may fill   <- we add with VIDIOC_QBUF
     *   outgoing: filled buffers waiting for us -> we take with VIDIOC_DQBUF
     * Right after REQBUFS, every buffer belongs to us, so the camera has
     * nowhere to write. We queue all of them first.
     */
    for (unsigned int i = 0; i < n_buffers; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;                         /* buffers are identified by index, not by address */

        if (xioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("VIDIOC_QBUF");
            return -1;
        }
    }

    /* STREAMON takes a pointer to an int holding the buffer type, not a struct. */
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        perror("VIDIOC_STREAMON");
        return -1;
    }
    streaming = 1;   /* remember, so cleanup() stops the stream */
    printf("streaming started\n");
    return 0;
}

/* ------------------------------------------------------------------------ */
/* 5. Wait for a filled buffer, optionally save it, then hand it back.      */
/* ------------------------------------------------------------------------ */
static int capture_frame(int fd, const char *out_path)
{
    struct v4l2_buffer buf;

    for (;;) {
        /*
         * poll() puts this thread to sleep (no CPU used) until the driver
         * says a filled buffer is ready (POLLIN), or the timeout passes.
         * This is why we opened the device with O_NONBLOCK: poll does the
         * waiting, and DQBUF never blocks.
         */
        struct pollfd pfd = { .fd = fd, .events = POLLIN };
        int r = poll(&pfd, 1, POLL_TIMEOUT);
        if (r == -1) {
            if (errno == EINTR)     /* a signal woke us; just wait again */
                continue;
            perror("poll");
            return -1;
        }
        if (r == 0) {               /* nothing arrived in 2 s: camera stalled or unplugged */
            fprintf(stderr, "timeout: no frame in %d ms\n", POLL_TIMEOUT);
            return -1;
        }

        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        /* VIDIOC_DQBUF: take the oldest filled buffer off the outgoing queue.
           The driver tells us WHICH one in buf.index. We don't pick it. */
        if (xioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            if (errno == EAGAIN)    /* poll woke us but no buffer is ready yet: wait again */
                continue;
            perror("VIDIOC_DQBUF");
            return -1;
        }
        break;                      /* we own buffer 'buf.index' now */
    }

    /* The driver sets this flag if the frame was damaged (e.g. USB packets
       lost). The data is still there, so we report it but carry on. */
    if (buf.flags & V4L2_BUF_FLAG_ERROR)
        fprintf(stderr, "warning: frame %u marked as corrupted\n", buf.sequence);

    if (out_path) {
        FILE *f = fopen(out_path, "wb");
        if (!f) {
            perror(out_path);
        } else {
            /* bytesused = how much of the buffer holds this frame's data.
               For YUYV it equals sizeimage; for compressed formats (MJPG) it
               varies per frame, which is why we never write buf.length. */
            size_t written = fwrite(buffers[buf.index].start, 1, buf.bytesused, f);
            fclose(f);
            if (written != buf.bytesused)
                fprintf(stderr, "short write: %zu of %u bytes\n", written, buf.bytesused);
        }
        /* sequence = frame counter kept by the driver; gaps mean frames were dropped. */
        printf("frame:    seq %u, buffer %u, %u bytes\n", buf.sequence, buf.index, buf.bytesused);
    }

    /*
     * Give the buffer back to the driver's incoming queue. If we forget this,
     * each frame permanently uses up one buffer. After n_buffers frames the
     * driver has no empty buffer to fill, no frame ever becomes ready, and
     * poll() times out.
     */
    if (xioctl(fd, VIDIOC_QBUF, &buf) == -1) {
        perror("VIDIOC_QBUF");
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------------ */
/* 6. Undo everything, in reverse order of setup. Safe to call after a      */
/*    failure at any earlier step.                                          */
/* ------------------------------------------------------------------------ */
static void cleanup(int fd)
{
    if (streaming) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        /* STREAMOFF stops the camera and removes every buffer from both queues. */
        if (xioctl(fd, VIDIOC_STREAMOFF, &type) == -1)
            perror("VIDIOC_STREAMOFF");
        streaming = 0;
    }

    /* Remove our mappings. Until we do, the kernel cannot free those pages,
       because our page tables still point at them. */
    for (unsigned int i = 0; i < n_buffers; i++) {
        if (buffers[i].start) {
            munmap(buffers[i].start, buffers[i].length);
            buffers[i].start = NULL;
        }
    }

    /* REQBUFS with count = 0 tells the driver to free its buffers now.
       (close() would also free them; doing it explicitly shows the full lifecycle.) */
    if (n_buffers) {
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count  = 0;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        xioctl(fd, VIDIOC_REQBUFS, &req);
        n_buffers = 0;
    }
}

int main(int argc, char **argv)
{
    const char *dev      = argc > 1 ? argv[1] : "/dev/video0";
    const char *out_path = argc > 2 ? argv[2] : "frame.yuv";
    int ret = EXIT_FAILURE;

    /* O_RDWR: V4L2 needs read+write access for ioctls and mmap(PROT_WRITE).
       O_NONBLOCK: DQBUF returns EAGAIN instead of sleeping; we sleep in poll(). */
    int fd = open(dev, O_RDWR | O_NONBLOCK);
    if (fd == -1) {
        fprintf(stderr, "open %s: %s\n", dev, strerror(errno));
        return EXIT_FAILURE;
    }
    printf("opened:   %s (fd %d)\n", dev, fd);

    /* Setup steps 1-4. The || stops at the first one that fails. */
    if (query_caps(fd) == -1 || set_format(fd) == -1 ||
        init_mmap(fd) == -1  || start_streaming(fd) == -1)
        goto out;

    /* Throw away the first frames: the image is dark until auto-exposure adjusts. */
    for (int i = 0; i < WARMUP; i++)
        if (capture_frame(fd, NULL) == -1)
            goto out;

    /* Keep the next frame. */
    if (capture_frame(fd, out_path) == -1)
        goto out;

    printf("saved:    %s (expected %u bytes)\n", out_path, frame_bytes);
    ret = EXIT_SUCCESS;

out:
    /* One exit path: always clean up, whether we succeeded or failed. */
    cleanup(fd);
    close(fd);
    return ret;
}
