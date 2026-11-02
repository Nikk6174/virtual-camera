/*
 * filters.cpp - implementation of the hand-written RGB24 filters.
 *
 * WHAT IT DOES
 *   Implements FilterRunner::apply() from pipeline/filters.h. Two kinds of filter:
 *     - per-pixel:     output pixel depends only on the same input pixel
 *                      (brightness/contrast, grayscale). Cheap: a few ops per byte.
 *     - neighbourhood: output pixel is a weighted sum of the pixels around it,
 *                      a "convolution" with a small grid of weights called a
 *                      kernel (blur, sharpen, Sobel edges). Cost grows with
 *                      kernel size: a 5x5 kernel reads 25 pixels per output pixel.
 *
 * HOW IT CONNECTS TO THE REST OF THE PROJECT
 *   - Compiled into the "pipeline" library together with convert.cpp.
 *   - Called once per frame by pipeline/preview.cpp (and later by the Step 4
 *     process thread), after YUYV->RGB conversion.
 *   - Written plainly on purpose (straightforward loops, edge checks on every
 *     read). This is the Step 7 baseline; we'll measure and then optimise.
 */

#include "filters.h"

#include <cmath>     // std::sqrt
#include <cstring>   // std::memcpy

namespace vc {

namespace {   // helpers private to this file

/* Limit v to [lo, hi]. Used to keep neighbour coordinates inside the image. */
inline int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Integer -> byte with saturation: below 0 becomes 0, above 255 becomes 255
   (instead of wrapping around, which would turn bright pixels dark). */
inline uint8_t sat(int v)
{
    return static_cast<uint8_t>(clampi(v, 0, 255));
}

/* Float -> byte with rounding and saturation. */
inline uint8_t satf(float v)
{
    if (v <= 0.0f)   return 0;
    if (v >= 255.0f) return 255;
    return static_cast<uint8_t>(v + 0.5f);
}

/* Pointer to the first byte of row y. Strides, not width*3, so row padding works. */
inline const uint8_t* row_ptr(const Image& im, int y) { return im.data + static_cast<long>(y) * im.stride; }
inline uint8_t*       row_ptr_w(const Image& im, int y) { return im.data + static_cast<long>(y) * im.stride; }

/* ---------------------------------------------------------------------- */
/* None: copy row by row (rows may be padded differently in src and dst).  */
/* ---------------------------------------------------------------------- */
void copy_image(const Image& src, const Image& dst)
{
    for (int y = 0; y < src.height; ++y)
        std::memcpy(row_ptr_w(dst, y), row_ptr(src, y), static_cast<size_t>(src.width) * 3);
}

/* ---------------------------------------------------------------------- */
/* Brightness / contrast (per-pixel)                                       */
/*   out = (in - 128) * contrast + 128 + brightness                        */
/* Contrast scales each value's distance from mid-grey (128): > 1 pushes    */
/* darks darker and lights lighter, < 1 pulls everything toward grey.       */
/* Brightness then shifts everything up or down. Same formula for R, G, B,  */
/* so colours keep their hue.                                               */
/* ---------------------------------------------------------------------- */
void brightness_contrast(const Image& src, const Image& dst, const FilterParams& p)
{
    for (int y = 0; y < src.height; ++y) {
        const uint8_t* s = row_ptr(src, y);
        uint8_t*       d = row_ptr_w(dst, y);
        // Each row is width*3 bytes, and every byte gets the same formula,
        // so we can treat R, G and B alike and walk the bytes in one loop.
        for (int i = 0; i < src.width * 3; ++i)
            d[i] = satf((static_cast<float>(s[i]) - 128.0f) * p.contrast + 128.0f
                        + static_cast<float>(p.brightness));
    }
}

/* Luma (perceived brightness) of one RGB pixel, BT.601 weights, the same
   standard the camera uses. Green counts most because human eyes are most
   sensitive to it; blue counts least. */
inline float luma(const uint8_t* px)
{
    return 0.299f * px[0] + 0.587f * px[1] + 0.114f * px[2];
}

/* ---------------------------------------------------------------------- */
/* Grayscale (per-pixel): write the luma into R, G and B.                   */
/* (A plain average (R+G+B)/3 would make pure green look as dark as pure    */
/* blue, which isn't how we see them.)                                     */
/* ---------------------------------------------------------------------- */
void grayscale(const Image& src, const Image& dst)
{
    for (int y = 0; y < src.height; ++y) {
        const uint8_t* s = row_ptr(src, y);
        uint8_t*       d = row_ptr_w(dst, y);
        for (int x = 0; x < src.width; ++x) {
            const uint8_t g = satf(luma(s + x * 3));
            d[x * 3 + 0] = g;   // equal R, G, B = a shade of grey
            d[x * 3 + 1] = g;
            d[x * 3 + 2] = g;
        }
    }
}

/* ---------------------------------------------------------------------- */
/* Generic RGB convolution with a square kernel of odd size k (3, 5, ...).  */
/*                                                                          */
/* For every output pixel (x, y) and channel c:                             */
/*   sum = Σ kernel[j][i] * src(x + i - r, y + j - r, c)   over the k x k    */
/*   out = sum / divisor                                                    */
/* where r = k / 2 is the kernel "radius" (2 for 5x5).                       */
/*                                                                          */
/* Borders: near the edge, part of the kernel falls outside the image. We   */
/* "clamp" the coordinate to the nearest edge pixel (repeat the border).    */
/* Treating outside pixels as black instead would make a dark frame appear  */
/* around blurred images.                                                   */
/*                                                                          */
/* Integer weights + a divisor keep the maths exact: the blur kernel's       */
/* weights add up to 256, so dividing by 256 keeps overall brightness the    */
/* same (a flat grey area stays the same grey).                             */
/* ---------------------------------------------------------------------- */
void convolve(const Image& src, const Image& dst, const int* kernel, int k, int divisor)
{
    const int r = k / 2;
    for (int y = 0; y < src.height; ++y) {
        uint8_t* d = row_ptr_w(dst, y);
        for (int x = 0; x < src.width; ++x) {
            int sum[3] = {0, 0, 0};                         // R, G, B accumulators

            for (int j = 0; j < k; ++j) {                   // kernel row
                // Source row for this kernel row, clamped into the image.
                const uint8_t* s = row_ptr(src, clampi(y + j - r, 0, src.height - 1));
                for (int i = 0; i < k; ++i) {               // kernel column
                    const int sx = clampi(x + i - r, 0, src.width - 1);
                    const int w  = kernel[j * k + i];        // weight for this position
                    sum[0] += w * s[sx * 3 + 0];
                    sum[1] += w * s[sx * 3 + 1];
                    sum[2] += w * s[sx * 3 + 2];
                }
            }

            // Divide back down to 0..255. Adding divisor/2 first rounds to the
            // nearest value instead of always rounding down (positive sums only;
            // for the sharpen kernel the divisor is 1, so no rounding is needed).
            for (int c = 0; c < 3; ++c)
                d[x * 3 + c] = sat((sum[c] + divisor / 2) / divisor);
        }
    }
}

/*
 * 5x5 Gaussian blur kernel: the outer product of the binomial row [1 4 6 4 1]
 * with itself. The centre pixel weighs most (36) and weights fall off with
 * distance, like a bell curve, so the blur looks smooth. A "box" blur, with
 * all weights equal, produces visible blocky artefacts instead.
 * Sum of weights = 16 * 16 = 256.
 */
constexpr int kGauss5[25] = {
    1,  4,  6,  4, 1,
    4, 16, 24, 16, 4,
    6, 24, 36, 24, 6,
    4, 16, 24, 16, 4,
    1,  4,  6,  4, 1,
};

/*
 * 3x3 sharpen kernel. Equivalent to: out = in + (in - average of 4 neighbours).
 * "in - neighbours" is large only where the pixel differs from its
 * surroundings (an edge), so edges get boosted and flat areas stay unchanged.
 * Sum of weights = 1 (5 - 4), so flat areas keep their brightness.
 */
constexpr int kSharpen3[9] = {
     0, -1,  0,
    -1,  5, -1,
     0, -1,  0,
};

}  // namespace

/* ---------------------------------------------------------------------- */
/* Sobel edge detection.                                                    */
/*   1. Convert to grayscale (edges are about brightness changes).          */
/*   2. For each pixel, estimate how fast brightness changes horizontally   */
/*      (gx) and vertically (gy) using two 3x3 kernels:                     */
/*        gx: [-1 0 1]      gy: [-1 -2 -1]                                   */
/*            [-2 0 2]          [ 0  0  0]                                   */
/*            [-1 0 1]          [ 1  2  1]                                   */
/*      gx = (right column) - (left column), with the middle row counted     */
/*      double, which smooths out noise a little.                           */
/*   3. Edge strength = sqrt(gx² + gy²): big where brightness changes fast.  */
/*      Output it as grey: edges bright, flat areas black.                 */
/* ---------------------------------------------------------------------- */
static void sobel_edges(const Image& src, const Image& dst, std::vector<uint8_t>& gray)
{
    const int w = src.width, h = src.height;

    // Step 1: grayscale into the scratch buffer (resize only allocates the
    // first time, or if the frame size changes).
    gray.resize(static_cast<size_t>(w) * h);
    for (int y = 0; y < h; ++y) {
        const uint8_t* s = row_ptr(src, y);
        for (int x = 0; x < w; ++x)
            gray[static_cast<size_t>(y) * w + x] = satf(luma(s + x * 3));
    }

    // Read gray(x, y) with border clamping.
    auto g = [&](int x, int y) -> int {
        return gray[static_cast<size_t>(clampi(y, 0, h - 1)) * w + clampi(x, 0, w - 1)];
    };

    // Steps 2-3: gradients and strength for every pixel.
    for (int y = 0; y < h; ++y) {
        uint8_t* d = row_ptr_w(dst, y);
        for (int x = 0; x < w; ++x) {
            const int gx = -g(x - 1, y - 1) + g(x + 1, y - 1)
                         - 2 * g(x - 1, y) + 2 * g(x + 1, y)
                         -  g(x - 1, y + 1) + g(x + 1, y + 1);
            const int gy = -g(x - 1, y - 1) - 2 * g(x, y - 1) - g(x + 1, y - 1)
                         +  g(x - 1, y + 1) + 2 * g(x, y + 1) + g(x + 1, y + 1);

            // Max possible strength is about 1442 (sqrt(1020² + 1020²)); we
            // saturate at 255, so medium and strong edges both show as white.
            const uint8_t e = satf(std::sqrt(static_cast<float>(gx * gx + gy * gy)));
            d[x * 3 + 0] = e;
            d[x * 3 + 1] = e;
            d[x * 3 + 2] = e;
        }
    }
}

const char* filter_name(Filter f)
{
    switch (f) {
        case Filter::None:               return "none";
        case Filter::BrightnessContrast: return "brightness/contrast";
        case Filter::Grayscale:          return "grayscale";
        case Filter::Blur:               return "blur 5x5";
        case Filter::Sharpen:            return "sharpen";
        case Filter::Edges:              return "edges (Sobel)";
        case Filter::Count:              break;
    }
    return "?";
}

void FilterRunner::apply(Filter f, const FilterParams& p, const Image& src, const Image& dst)
{
    switch (f) {
        case Filter::None:               copy_image(src, dst);                  break;
        case Filter::BrightnessContrast: brightness_contrast(src, dst, p);      break;
        case Filter::Grayscale:          grayscale(src, dst);                   break;
        case Filter::Blur:               convolve(src, dst, kGauss5, 5, 256);   break;
        case Filter::Sharpen:            convolve(src, dst, kSharpen3, 3, 1);   break;
        case Filter::Edges:              sobel_edges(src, dst, gray_);          break;
        case Filter::Count:              copy_image(src, dst);                  break;
    }
}

}  // namespace vc
