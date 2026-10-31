/*
 * convert.cpp - YUYV -> RGB24 conversion (baseline floating-point version).
 *
 * WHAT IT DOES
 *   Implements vc::yuyv_to_rgb24() from pipeline/convert.h. For every pair of
 *   pixels it reads 4 YUYV bytes, applies the BT.601 limited-range formulas,
 *   and writes 6 RGB bytes.
 *
 * HOW IT CONNECTS TO THE REST OF THE PROJECT
 *   - Compiled into the "pipeline" static library (CMakeLists.txt).
 *   - Called once per frame by pipeline/preview.cpp, between
 *     capture_dequeue() and drawing to the SDL window.
 *   - This is the deliberately simple "before" version for Step 7. It uses
 *     floats, one pixel pair at a time, single-threaded, so each later
 *     optimisation (integer maths, lookup tables, threads, AVX2) has a
 *     measured baseline to beat.
 *
 * THE MATHS (BT.601, limited range)
 *   The camera stores:
 *     Y (luma, brightness)        in 16..235   (16 = black, 235 = white)
 *     U (Cb, blue minus luma)     in 16..240   (128 = no colour)
 *     V (Cr, red minus luma)      in 16..240   (128 = no colour)
 *   "Limited range" leaves headroom below 16 and above 235/240; this comes
 *   from analogue TV and is still the default for webcams.
 *   To get RGB in 0..255:
 *     R = 1.164 (Y-16)                 + 1.596 (V-128)
 *     G = 1.164 (Y-16) - 0.392 (U-128) - 0.813 (V-128)
 *     B = 1.164 (Y-16) + 2.017 (U-128)
 *   1.164 = 255/219 stretches Y's 219 steps (16..235) to 256 (0..255).
 *   The U/V factors come from how BT.601 weights red/green/blue in Y
 *   (Y = 0.299 R + 0.587 G + 0.114 B): to rebuild G you must subtract the
 *   parts of R and B that are already inside Y, which is why G uses both U and V.
 */

#include "convert.h"

namespace vc {

namespace {   // anonymous namespace: these helpers are private to this file

/*
 * Turn a float result into a valid byte. The formulas can overshoot:
 * e.g. Y = 235 with strong V gives R > 255, and Y = 16 with negative V gives R < 0.
 * Without clamping, storing 260 into a uint8_t wraps to 4 (bright red
 * becomes nearly black), which shows up as coloured speckles.
 * +0.5f rounds to the nearest integer instead of always rounding down.
 */
inline uint8_t clamp_to_byte(float v)
{
    if (v <= 0.0f)   return 0;
    if (v >= 255.0f) return 255;
    return static_cast<uint8_t>(v + 0.5f);
}

/* Convert one pixel's (Y, U, V) and write R, G, B to out[0..2]. */
inline void yuv_to_rgb(int y, int u, int v, uint8_t* out)
{
    // Move the values so that "black" and "no colour" are 0.
    // Without this, a grey pixel (U = V = 128) would get a strong colour cast.
    const float c = 1.164f * static_cast<float>(y - 16);   // scaled luma
    const float d = static_cast<float>(u - 128);           // blue difference, -112..+112
    const float e = static_cast<float>(v - 128);           // red difference,  -112..+112

    out[0] = clamp_to_byte(c + 1.596f * e);                // R: luma + red difference
    out[1] = clamp_to_byte(c - 0.392f * d - 0.813f * e);   // G: luma minus the R and B parts
    out[2] = clamp_to_byte(c + 2.017f * d);                // B: luma + blue difference
}

}  // namespace

void yuyv_to_rgb24(const uint8_t* src, int src_stride,
                   uint8_t* dst, int dst_stride,
                   int width, int height)
{
    for (int row = 0; row < height; ++row) {
        // Find the start of this row in both images using the STRIDES, not
        // width*2 / width*3. If the camera pads its rows, the stride includes
        // the padding, and using width would read the padding as pixels.
        const uint8_t* s = src + static_cast<long>(row) * src_stride;
        uint8_t*       d = dst + static_cast<long>(row) * dst_stride;

        // Walk the row two pixels at a time: one YUYV group per step.
        for (int x = 0; x < width; x += 2) {
            const int y0 = s[0];   // luma of the left pixel
            const int u  = s[1];   // Cb, shared by both pixels
            const int y1 = s[2];   // luma of the right pixel
            const int v  = s[3];   // Cr, shared by both pixels

            // Both pixels use the same U and V (that's what "4:2:2" means:
            // full-resolution brightness, half-resolution colour horizontally).
            // Human eyes notice brightness detail much more than colour detail,
            // so this halves the colour data with little visible loss.
            yuv_to_rgb(y0, u, v, d);       // writes d[0], d[1], d[2]
            yuv_to_rgb(y1, u, v, d + 3);   // writes d[3], d[4], d[5]

            s += 4;   // consumed 4 YUYV bytes
            d += 6;   // produced 2 RGB pixels = 6 bytes
        }
    }
}

}  // namespace vc
