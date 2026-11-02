/*
 * filters.h - hand-written image filters for RGB24 frames.
 *
 * WHAT IT IS
 *   The list of filters (none, brightness/contrast, grayscale, blur, sharpen,
 *   edge detection), their adjustable settings, and one entry point
 *   (FilterRunner::apply) that runs the selected filter on a frame.
 *   No library does the image work: every pixel operation is in filters.cpp.
 *
 * HOW IT CONNECTS TO THE REST OF THE PROJECT
 *   - Input:  RGB24 frames produced by vc::yuyv_to_rgb24 (pipeline/convert.h).
 *   - Output: RGB24 frames that pipeline/preview.cpp draws in the SDL window
 *             (number keys pick the filter), and later the Step 5 virtual
 *             camera output sends to Meet.
 *   - Step 4 runs FilterRunner::apply on its own "process" thread.
 *   - Step 7 profiles these filters and optimises the slowest ones.
 */

#pragma once

#include <cstdint>
#include <vector>

namespace vc {

/*
 * A view of an RGB24 image: it POINTS at pixels owned by someone else (an
 * SDL texture, a std::vector, ...). It never allocates or frees.
 * Pixel (x, y) channel c is at data[y * stride + x * 3 + c], c: 0=R 1=G 2=B.
 */
struct Image {
    uint8_t* data;
    int      width;
    int      height;
    int      stride;   // bytes from one row to the next (>= width * 3)
};

/* The filters, in number-key order: key 1 = None, key 2 = BrightnessContrast, ... */
enum class Filter {
    None,                 // copy the image unchanged
    BrightnessContrast,   // per-pixel: stretch around mid-grey, then shift
    Grayscale,            // per-pixel: weighted sum of R, G, B
    Blur,                 // 5x5 Gaussian convolution (neighbourhood filter)
    Sharpen,              // 3x3 sharpen convolution (neighbourhood filter)
    Edges,                // Sobel edge detection on the grayscale image
    Count                 // number of filters (not a filter itself)
};

/* Human-readable name for the window title, e.g. "blur 5x5". */
const char* filter_name(Filter f);

/* Settings the user can change live (arrow keys in the preview). */
struct FilterParams {
    int   brightness = 0;      // added to every channel, -100..+100
    float contrast   = 1.0f;   // multiplies distance from mid-grey, 0.2..3.0
};

/*
 * Runs filters. It's a class only because edge detection needs a scratch
 * buffer (the grayscale image) that we want to allocate once and reuse every
 * frame, instead of allocating ~300 KB 30 times per second.
 */
class FilterRunner {
public:
    /*
     * Apply filter f to src, writing the result into dst.
     * src and dst MUST be different buffers of the same size: neighbourhood
     * filters (blur, sharpen, edges) read pixels around (x, y) after (x, y)'s
     * neighbours have already been processed, so writing in place would make
     * them read already-filtered values.
     */
    void apply(Filter f, const FilterParams& p, const Image& src, const Image& dst);

private:
    std::vector<uint8_t> gray_;   // width*height luma values, reused for Edges
};

}  // namespace vc
