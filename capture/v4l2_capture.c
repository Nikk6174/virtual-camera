/*
 * v4l2_capture.c - reusable V4L2 webcam capture (implementation).
 *
 * WHAT IT DOES
 *   Implements the functions declared in capture/v4l2_capture.h: open and
 *   configure a camera, map its buffers, start streaming, hand out frames one
 *   at a time (dequeue), take them back (requeue), and clean up.
 *
 * HOW IT CONNECTS TO THE REST OF THE PROJECT
 *   - Compiled into the "v4l2_capture" static library by CMakeLists.txt.
 *   - Linked into pipeline/preview.cpp (Step 2) and the Step 4 pipeline.
 *   - Same ioctl sequence as capture/capture_one.c (Step 1); the comments
 *     here focus on what's NEW: frame-rate setting (S_PARM), timestamps, a
 *     state struct instead of globals, and splitting DQBUF / QBUF so the
 *     caller decides how long to hold a buffer.
 */

#include "v4l2_capture.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

/* Retry an ioctl that was interrupted by a signal (EINTR = nothing was done). */
static int xioctl(int fd, unsigned long request, void *arg)
{
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

/* Check the node can capture video frames and supports streaming I/O. */
static int check_caps(struct capture *c)
{
    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));
    if (xioctl(c->fd, VIDIOC_QUERYCAP, &cap) == -1) {
        perror("VIDIOC_QUERYCAP");
        return -1;
    }

    /* device_caps = this node only (see Step 1: /dev/video1 is metadata-only). */
    uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps
                                                               : cap.capabilities;
    if (!(caps & V4L2_CAP_VIDEO_CAPTURE) || !(caps & V4L2_CAP_STREAMING)) {
        fprintf(stderr, "%s: not a streaming video capture node\n", (char *)cap.card);
        return -1;
    }
    printf("capture: %s (%s)\n", (char *)cap.card, (char *)cap.driver);
    return 0;
}

/* Negotiate YUYV width x height; store what the driver actually chose. */
static int set_format(struct capture *c, uint32_t width, uint32_t height)
{
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = width;
    fmt.fmt.pix.height      = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

    if (xioctl(c->fd, VIDIOC_S_FMT, &fmt) == -1) {
        perror("VIDIOC_S_FMT");             /* EBUSY: another app is using the camera */
        return -1;
    }
    /* The driver may have adjusted our request; only YUYV is acceptable to the
       rest of the pipeline (the converter only understands YUYV). */
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
        fprintf(stderr, "driver did not accept YUYV\n");
        return -1;
    }
    c->width       = fmt.fmt.pix.width;      /* may differ from the request: */
    c->height      = fmt.fmt.pix.height;     /* callers must use these, not their own constants */
    c->stride      = fmt.fmt.pix.bytesperline;
    c->frame_bytes = fmt.fmt.pix.sizeimage;
    return 0;
}

/*
 * NEW vs Step 1: ask for a frame rate with VIDIOC_S_PARM.
 * V4L2 describes speed as a time PER frame (timeperframe), not frames per
 * second: 30 fps = 1/30 s per frame = numerator 1, denominator 30.
 * Like S_FMT, it's a request: the driver picks the closest interval the
 * camera supports for the current format, and writes that back.
 * Some drivers don't support S_PARM at all; that's not fatal, we just
 * keep whatever rate the camera defaults to.
 */
static void set_fps(struct capture *c, uint32_t fps)
{
    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator   = 1;
    parm.parm.capture.timeperframe.denominator = fps;

    if (xioctl(c->fd, VIDIOC_S_PARM, &parm) == -1) {
        perror("VIDIOC_S_PARM (ignored, using camera default)");
        c->fps_num = 0;                     /* 0 = unknown */
        c->fps_den = 0;
        return;
    }
    c->fps_num = parm.parm.capture.timeperframe.numerator;
    c->fps_den = parm.parm.capture.timeperframe.denominator;
}

/*
 * NEW vs Step 1: camera controls (VIDIOC_S_CTRL).
 * Brightness, exposure, white balance etc. are "controls": numbered settings
 * the driver forwards to the camera. Here we turn OFF
 * exposure_dynamic_framerate. When it's on, the camera may lengthen each
 * exposure in dim light, which silently drops 30 fps to 15 fps. (We measured
 * exactly that: S_PARM said 30 fps, but frames arrived at 15 fps with no
 * sequence gaps.) Off = steady 30 fps, slightly darker image in dim rooms.
 * The setting lives in the camera until it's unplugged, so we set it every
 * time. Cameras without this control return EINVAL; that's fine, so we
 * ignore errors.
 */
static void disable_dynamic_framerate(struct capture *c)
{
    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id    = V4L2_CID_EXPOSURE_AUTO_PRIORITY;  /* kernel name for "exposure_dynamic_framerate" */
    ctrl.value = 0;                                /* 0 = keep the frame rate fixed */
    xioctl(c->fd, VIDIOC_S_CTRL, &ctrl);
}

/* REQBUFS + QUERYBUF + mmap for each buffer (same as Step 1). */
static int map_buffers(struct capture *c, unsigned count)
{
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count  = count;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(c->fd, VIDIOC_REQBUFS, &req) == -1) {
        perror("VIDIOC_REQBUFS");
        return -1;
    }
    if (req.count < 2) {                    /* need one being filled while we read another */
        fprintf(stderr, "only %u buffer(s) available\n", req.count);
        return -1;
    }
    /* Set n_buffers BEFORE mapping, so if mapping fails halfway, capture_close()
       knows how many entries to look at (unmapped ones have start == NULL). */
    c->n_buffers = req.count;

    for (unsigned i = 0; i < c->n_buffers; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (xioctl(c->fd, VIDIOC_QUERYBUF, &buf) == -1) {
            perror("VIDIOC_QUERYBUF");
            return -1;
        }

        /* Map the driver's buffer i into our address space (offset = driver's cookie). */
        void *p = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                       c->fd, buf.m.offset);
        if (p == MAP_FAILED) {              /* MAP_FAILED, not NULL */
            perror("mmap");
            return -1;
        }
        c->buffers[i].start  = p;
        c->buffers[i].length = buf.length;
    }
    return 0;
}

int capture_open(struct capture *c, const char *dev,
                 uint32_t width, uint32_t height, uint32_t fps, unsigned n_buffers)
{
    /* Start from a known state: all zeros, fd = -1 ("nothing open").
       This is what makes capture_close() safe to call after a partial failure. */
    memset(c, 0, sizeof(*c));
    c->fd = -1;

    /* VIDEO_MAX_FRAME (32) is the size of our buffers[] array. */
    if (n_buffers < 2 || n_buffers > VIDEO_MAX_FRAME) {
        fprintf(stderr, "n_buffers must be 2..%d\n", VIDEO_MAX_FRAME);
        return -1;
    }

    /* O_NONBLOCK: DQBUF never sleeps; capture_dequeue() waits in poll() instead. */
    c->fd = open(dev, O_RDWR | O_NONBLOCK);
    if (c->fd == -1) {
        fprintf(stderr, "open %s: %s\n", dev, strerror(errno));
        return -1;
    }

    /* Order matters: the frame rate options depend on the format (e.g. this
       camera can do YUYV 640x480 at 30 fps but YUYV 1280x720 only at 10 fps),
       and buffer sizes depend on the format, so: format -> fps -> buffers. */
    if (check_caps(c) == -1 || set_format(c, width, height) == -1) {
        capture_close(c);
        return -1;
    }
    set_fps(c, fps);
    disable_dynamic_framerate(c);
    if (map_buffers(c, n_buffers) == -1) {
        capture_close(c);
        return -1;
    }

    printf("capture: YUYV %ux%u, stride %u, %u bytes/frame, %u buffers",
           c->width, c->height, c->stride, c->frame_bytes, c->n_buffers);
    if (c->fps_num)
        printf(", %.1f fps", (double)c->fps_den / c->fps_num);  /* fps = den/num */
    printf("\n");
    return 0;
}

int capture_start(struct capture *c)
{
    /* Put every buffer on the driver's "empty, please fill" queue... */
    for (unsigned i = 0; i < c->n_buffers; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (xioctl(c->fd, VIDIOC_QBUF, &buf) == -1) {
            perror("VIDIOC_QBUF");
            return -1;
        }
    }
    /* ...then start the camera. STREAMON takes a pointer to the buffer type. */
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(c->fd, VIDIOC_STREAMON, &type) == -1) {
        perror("VIDIOC_STREAMON");
        return -1;
    }
    c->streaming = 1;
    return 0;
}

int capture_dequeue(struct capture *c, struct frame *f, int timeout_ms)
{
    struct v4l2_buffer buf;

    for (;;) {
        /* Sleep (no CPU used) until the driver has a filled buffer, or timeout. */
        struct pollfd pfd = { .fd = c->fd, .events = POLLIN };
        int r = poll(&pfd, 1, timeout_ms);
        if (r == -1) {
            if (errno == EINTR)             /* woken by a signal (e.g. Ctrl+C handler): */
                return 1;                   /* report as timeout so the caller can check its quit flag */
            perror("poll");
            return -1;
        }
        if (r == 0)
            return 1;                       /* timeout: no frame in timeout_ms */

        /* POLLERR = the device has a problem (e.g. camera unplugged). */
        if (pfd.revents & POLLERR) {
            fprintf(stderr, "capture: device error (unplugged?)\n");
            return -1;
        }

        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (xioctl(c->fd, VIDIOC_DQBUF, &buf) == -1) {
            if (errno == EAGAIN)            /* poll woke us, but no buffer yet: wait again */
                continue;
            perror("VIDIOC_DQBUF");
            return -1;
        }
        break;                              /* we now own buffer buf.index */
    }

    /* Describe the buffer to the caller. f->data points INTO the mmap'ed kernel
       buffer: zero copies. It stays valid until capture_requeue(f). */
    f->data      = (const uint8_t *)c->buffers[buf.index].start;
    f->bytesused = buf.bytesused;
    f->index     = buf.index;
    f->sequence  = buf.sequence;

    /*
     * NEW vs Step 1: the driver's timestamp. uvcvideo stamps each frame with
     * CLOCK_MONOTONIC (a clock that never jumps backwards, unlike wall-clock
     * time), at the moment the first bytes of the frame arrived over USB. In
     * Step 7 we compare it with clock_gettime(CLOCK_MONOTONIC) to measure
     * latency from camera to screen.
     */
    f->timestamp_us = (uint64_t)buf.timestamp.tv_sec * 1000000u + buf.timestamp.tv_usec;
    return 0;
}

int capture_requeue(struct capture *c, const struct frame *f)
{
    /* We only need the index to identify the buffer; type and memory say which queue. */
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index  = f->index;
    if (xioctl(c->fd, VIDIOC_QBUF, &buf) == -1) {
        perror("VIDIOC_QBUF");
        return -1;
    }
    return 0;
}

void capture_close(struct capture *c)
{
    if (c->fd == -1)                        /* never opened, or already closed */
        return;

    if (c->streaming) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(c->fd, VIDIOC_STREAMOFF, &type);   /* stop camera, empty both queues */
        c->streaming = 0;
    }

    /* Remove our mappings so the kernel can free the pages. */
    for (unsigned i = 0; i < c->n_buffers; i++) {
        if (c->buffers[i].start) {
            munmap(c->buffers[i].start, c->buffers[i].length);
            c->buffers[i].start = NULL;
        }
    }

    /* REQBUFS(0): free the driver's buffers. */
    if (c->n_buffers) {
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        xioctl(c->fd, VIDIOC_REQBUFS, &req);
        c->n_buffers = 0;
    }

    close(c->fd);
    c->fd = -1;                             /* mark closed: a second call returns immediately */
}
