/*
 * v4l2_capture.h - public interface of the reusable V4L2 capture module.
 *
 * WHAT IT IS
 *   The same open -> QUERYCAP -> S_FMT -> REQBUFS -> mmap -> STREAMON ->
 *   DQBUF/QBUF -> cleanup sequence as capture/capture_one.c (Step 1), but
 *   packaged as functions that other code can call in a loop, instead of a
 *   program that saves one frame and exits.
 *
 * HOW IT CONNECTS TO THE REST OF THE PROJECT
 *   - Implemented in capture/v4l2_capture.c (plain C, like the kernel APIs it uses).
 *   - Used by the Step 2 preview program (pipeline/preview.cpp), and later by
 *     the capture thread of the Step 4 pipeline.
 *   - It hands out frames as raw YUYV bytes; pipeline/convert.* turns them into RGB.
 *   - capture/capture_one.c stays as the standalone Step 1 teaching program.
 *
 * TYPICAL USE
 *   struct capture cap;
 *   capture_open(&cap, "/dev/video0", 640, 480, 30, 4);
 *   capture_start(&cap);
 *   while (running) {
 *       struct frame f;
 *       capture_dequeue(&cap, &f, 2000);   // wait for a filled buffer (we own it now)
 *       ... read f.data ...                // pixels are read in place, no copy
 *       capture_requeue(&cap, &f);         // give the buffer back to the driver
 *   }
 *   capture_close(&cap);
 */

#ifndef V4L2_CAPTURE_H
#define V4L2_CAPTURE_H

#include <stddef.h>             /* size_t */
#include <stdint.h>             /* uint8_t, uint32_t, uint64_t */
#include <linux/videodev2.h>    /* VIDEO_MAX_FRAME */

/*
 * This header is included from C++ (preview.cpp). C++ compilers "mangle"
 * function names (encode argument types into the symbol name), while the C
 * compiler doesn't. extern "C" tells the C++ compiler to look for the plain
 * C names, so the linker can match them with v4l2_capture.c.
 */
#ifdef __cplusplus
extern "C" {
#endif

/* One kernel buffer mapped into our address space with mmap(). */
struct cap_buffer {
    void   *start;      /* virtual address in our process */
    size_t  length;     /* mapping size in bytes */
};

/*
 * Everything about one open camera. Keeping state in a struct (instead of
 * globals like capture_one.c) means we could open two cameras at once.
 */
struct capture {
    int      fd;                /* file descriptor of /dev/videoN, -1 if not open */
    uint32_t width, height;     /* frame size the driver actually chose */
    uint32_t stride;            /* bytesperline: bytes from one row to the next */
    uint32_t frame_bytes;       /* sizeimage: bytes in one frame */
    uint32_t fps_num, fps_den;  /* frame interval the driver chose, as a fraction (1/30 = 30 fps) */
    struct cap_buffer buffers[VIDEO_MAX_FRAME];
    unsigned n_buffers;         /* how many buffers the driver gave us */
    int      streaming;         /* 1 between STREAMON and STREAMOFF */
};

/*
 * One captured frame, borrowed from the driver.
 * 'data' points straight into a kernel buffer (mmap), so it is valid only
 * until capture_requeue() is called with this frame.
 */
struct frame {
    const uint8_t *data;        /* YUYV pixels, 'stride' bytes per row */
    uint32_t bytesused;         /* bytes of real data in the buffer */
    uint32_t index;             /* which buffer this is; needed to give it back */
    uint32_t sequence;          /* driver's frame counter; gaps = dropped frames */
    uint64_t timestamp_us;      /* when the driver got the frame (CLOCK_MONOTONIC, microseconds) */
};

/*
 * Open the device, check it can capture + stream, set YUYV width x height,
 * ask for 'fps' frames per second (and stop the camera from lowering it in
 * dim light), allocate and mmap 'n_buffers' buffers.
 * Prints what the driver actually chose. Returns 0 on success, -1 on error
 * (everything already opened is cleaned up).
 */
int  capture_open(struct capture *c, const char *dev,
                  uint32_t width, uint32_t height, uint32_t fps, unsigned n_buffers);

/* Queue all buffers and turn the stream on. 0 = ok, -1 = error. */
int  capture_start(struct capture *c);

/*
 * Wait up to timeout_ms for a filled buffer and fill 'f' with it.
 * Returns 0 = got a frame, 1 = timeout, -1 = error.
 * After this, WE own the buffer and the driver can't write into it.
 */
int  capture_dequeue(struct capture *c, struct frame *f, int timeout_ms);

/* Give the frame's buffer back to the driver. 0 = ok, -1 = error. */
int  capture_requeue(struct capture *c, const struct frame *f);

/* STREAMOFF, munmap, free buffers, close. Safe to call more than once. */
void capture_close(struct capture *c);

#ifdef __cplusplus
}
#endif

#endif /* V4L2_CAPTURE_H */
