/*
 * convert.h - colour conversion: camera YUYV -> RGB24 for display and filters.
 *
 * WHAT IT IS
 *   The webcam delivers YUYV (brightness + colour-difference values, with
 *   colour shared between pairs of pixels). Screens and our filters want RGB
 *   (red, green, blue per pixel). This module converts one to the other, by
 *   hand, with no library.
 *
 * HOW IT CONNECTS TO THE REST OF THE PROJECT
 *   - Input:  struct frame from capture/v4l2_capture.h (f.data, cap.stride).
 *   - Output: an RGB24 buffer that pipeline/preview.cpp uploads to the SDL
 *             window, and that the Step 3 filters will modify.
 *   - Step 7 optimises this function (integer maths -> lookup tables ->
 *     threads -> AVX2) and measures each change against this first version.
 *
 * FORMATS
 *   YUYV:  4 bytes = 2 pixels:  [Y0 U Y1 V]  ->  pixel0 = (Y0,U,V), pixel1 = (Y1,U,V)
 *   RGB24: 3 bytes = 1 pixel:   [R G B]
 */

#pragma once

#include <cstdint>

namespace vc {   // "virtual camera": keeps our names from clashing with SDL/libc names

/*
 * Convert a YUYV image to RGB24 (BT.601 limited range, the encoding this
 * camera reports in `v4l2-ctl --get-fmt-video`).
 *
 *   src        first byte of the YUYV image (e.g. frame.data)
 *   src_stride bytes from one YUYV row to the next (cap.stride; >= width*2)
 *   dst        first byte of the RGB24 output (at least dst_stride*height bytes)
 *   dst_stride bytes from one RGB row to the next (normally width*3)
 *   width      in pixels; must be even (YUYV stores pixels in pairs)
 *   height     in pixels
 *
 * No memory is allocated: the caller owns both buffers. That keeps the
 * per-frame cost to pure computation, which matters at 30 fps.
 */
void yuyv_to_rgb24(const uint8_t* src, int src_stride,
                   uint8_t* dst, int dst_stride,
                   int width, int height);

}  // namespace vc
