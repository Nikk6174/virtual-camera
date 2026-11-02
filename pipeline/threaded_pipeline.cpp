/*
 * threaded_pipeline.cpp - Step 4: the live video path, split across three threads.
 *
 * WHAT IT DOES
 *   Does the same job as preview.cpp, but each stage runs on its own thread, so
 *   a slow stage can't hold up the others:
 *
 *     capture thread --[raw_queue]--> process thread --[out_queue]--> output thread
 *     copy YUYV out of                convert to RGB,                 draw in the
 *     the driver buffer               run the filter                  SDL window
 *
 *   If processing falls behind, the queues throw away the OLDEST frames. The
 *   video shows fewer frames per second, but what it shows stays recent.
 *
 * KEYS
 *   1-6        pick a filter (same as preview.cpp)
 *   Up/Down    brightness +/- 5
 *   Right/Left contrast +/- 0.1
 *   R          reset brightness and contrast
 *   H          add or remove a 50 ms test load, which makes processing slower
 *              than the camera's 33 ms between frames
 *   Esc or Q   quit (the close button and Ctrl+C also work)
 *
 * HOW IT CONNECTS TO THE REST OF THE PROJECT
 *   - capture/v4l2_capture.h : gets frames from the camera (C library)
 *   - pipeline/frame_queue.h : carries frames between the threads
 *   - pipeline/convert.h     : vc::yuyv_to_rgb24
 *   - pipeline/filters.h     : vc::FilterRunner
 *   - SDL2                   : shows the pixels; used only on the main thread
 *   - Step 5 adds the virtual camera as a second destination for finished frames.
 *   - preview.cpp stays as the single-threaded version to compare against.
 *
 * WHO OWNS WHAT (this is what makes the threading safe)
 *   A Frame belongs to exactly one thread at a time. Handing it to a queue gives
 *   it away, and the sender must not touch it afterwards, so the pixels themselves
 *   never need a lock. The only state threads share is: the queues and the pools
 *   (each has its own mutex), Settings (its own mutex), and the atomic flags and
 *   counters.
 *
 * Usage: ./threaded_pipeline [device] [--frames N] [--filter K] [--load-ms MS]
 *   --frames N    quit after showing N frames and print a summary
 *   --filter K    start with filter K (1-6, same numbers as the keys)
 *   --load-ms MS  start with a test load of MS milliseconds per frame
 */

#include <SDL.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "convert.h"
#include "filters.h"
#include "frame_queue.h"
#include "v4l2_capture.h"

namespace {

constexpr int kWidth   = 640;
constexpr int kHeight  = 480;
constexpr int kFps     = 30;
constexpr int kBuffers = 4;

// Small on purpose: every queued frame adds up to one frame of delay. Two slots
// are enough to absorb a brief hiccup without building up a visible lag.
constexpr size_t kQueueCapacity = 2;

// The longest a thread sleeps before it checks whether it should stop (capture)
// or handle keyboard events (output).
constexpr int kWaitMs = 100;

constexpr int kToggleLoadMs = 50;   // what the H key switches on

using Clock = std::chrono::steady_clock;

double ms_since(Clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

/*
 * The current time in microseconds, in the same clock as frame.timestamp_us.
 * The driver stamps frames with CLOCK_MONOTONIC, and on Linux steady_clock reads
 * that same clock, so the two values can be subtracted to get the frame's age.
 */
uint64_t now_us()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
               Clock::now().time_since_epoch()).count();
}

/* One frame moving through the pipeline, with its pixels and its measurements. */
struct Frame {
    std::vector<uint8_t> pixels;      // YUYV in raw_queue, RGB24 in out_queue
    uint64_t timestamp_us = 0;        // when the driver received the frame
    double   convert_ms   = 0.0;      // time spent in yuyv_to_rgb24
    double   filter_ms    = 0.0;      // time spent in the filter
};

/*
 * A stock of reusable pixel buffers. Allocating about 1 MB for every frame would
 * waste time, so finished buffers come back here. New memory is allocated only
 * when the pool is empty, which stops happening once the pipeline has warmed up:
 * the number of frames that exist at once is limited by the queues.
 */
class BufferPool {
public:
    explicit BufferPool(size_t buffer_bytes) : buffer_bytes_(buffer_bytes) {}

    Frame acquire()
    {
        // The lock is released before any allocation, so a slow allocation can't
        // hold up another thread that's returning a buffer.
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!free_.empty()) {
                Frame frame = std::move(free_.back());
                free_.pop_back();
                return frame;
            }
            ++allocated_;
        }
        Frame frame;
        frame.pixels.resize(buffer_bytes_);
        return frame;
    }

    void release(Frame frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        free_.push_back(std::move(frame));
    }

    size_t allocated() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return allocated_;
    }

private:
    mutable std::mutex mutex_;
    std::vector<Frame> free_;
    size_t buffer_bytes_;
    size_t allocated_ = 0;
};

/*
 * What the user can change while the video runs. The output thread writes it and
 * the process thread reads it. It needs a mutex rather than atomics because the
 * fields must be read together: the filter and its parameters should always come
 * from the same key press.
 */
struct Settings {
    vc::Filter       filter = vc::Filter::None;
    vc::FilterParams params;
    int              load_ms = 0;
};

/* Everything the threads share. It's created in main() and outlives the threads. */
struct Shared {
    struct capture* cap;
    int width;
    int height;

    vc::FrameQueue<Frame> raw_queue{kQueueCapacity};
    vc::FrameQueue<Frame> out_queue{kQueueCapacity};
    BufferPool raw_pool;
    BufferPool rgb_pool;

    std::mutex settings_mutex;
    Settings   settings;

    // Atomics: each one is a single value, so no lock is needed to read or
    // update it safely from several threads.
    std::atomic<bool> stop{false};        // main asks the capture thread to finish
    std::atomic<bool> failed{false};      // the camera returned a real error
    std::atomic<long> captured{0};        // frames copied out of the driver
    std::atomic<long> driver_dropped{0};  // frames the driver lost (sequence gaps)
    std::atomic<long> processed{0};       // frames that finished the process stage

    Shared(struct capture* c, int w, int h)
        : cap(c), width(w), height(h),
          raw_pool(c->frame_bytes),
          rgb_pool(static_cast<size_t>(w) * h * 3) {}
};

/*
 * Burn CPU for 'ms' milliseconds. This stands in for a slow filter. It spins
 * instead of sleeping because a real filter keeps a CPU core busy, and sleeping
 * wouldn't.
 */
void busy_wait(int ms)
{
    const auto t0 = Clock::now();
    while (ms_since(t0) < ms) {
    }
}

/*
 * CAPTURE THREAD: gets frames from the camera and never waits for later stages.
 *
 * It copies each frame out of the driver's buffer and gives that buffer back right
 * away. The driver has only 4 buffers, so if slow stages kept hold of them, it
 * would run out and lose frames. The copy costs far less than the filters.
 */
void capture_thread(Shared& s)
{
    uint32_t last_seq = 0;
    bool     have_seq = false;

    while (!s.stop) {
        // Sleeps in poll() for up to kWaitMs, so 'stop' is noticed quickly even
        // if the camera has stalled.
        struct frame f;
        const int r = capture_dequeue(s.cap, &f, kWaitMs);
        if (r == 1)
            continue;                  // timeout: no frame yet
        if (r == -1) {
            s.failed = true;           // e.g. the camera was unplugged
            break;
        }

        // A jump bigger than 1 means the driver had no free buffer for a frame.
        if (have_seq && f.sequence > last_seq + 1)
            s.driver_dropped += f.sequence - last_seq - 1;
        last_seq = f.sequence;
        have_seq = true;

        // Copy the pixels into our own buffer. bytesused comes from the driver,
        // and the pool's buffers hold cap->frame_bytes, so limit it to that.
        Frame frame = s.raw_pool.acquire();
        std::memcpy(frame.pixels.data(), f.data,
                    std::min<size_t>(f.bytesused, frame.pixels.size()));
        frame.timestamp_us = f.timestamp_us;

        // The copy is done, so the driver can have its buffer back.
        if (capture_requeue(s.cap, &f) != 0) {
            s.failed = true;
            s.raw_pool.release(std::move(frame));
            break;
        }

        ++s.captured;

        // Never blocks. If the queue was full we get the oldest frame back, and
        // its buffer returns to the pool.
        if (auto rejected = s.raw_queue.push(std::move(frame)))
            s.raw_pool.release(std::move(*rejected));
    }

    // Closing is how shutdown moves down the pipeline: the process thread
    // finishes what's queued, sees the end, and closes the next queue in turn.
    s.raw_queue.close();
}

/*
 * PROCESS THREAD: does the slow work. It's the only thread touching 'runner' and
 * 'rgb', so those need no lock.
 */
void process_thread(Shared& s)
{
    vc::FilterRunner runner;
    std::vector<uint8_t> rgb(static_cast<size_t>(s.width) * s.height * 3);
    const int rgb_stride = s.width * 3;
    const vc::Image rgb_image{rgb.data(), s.width, s.height, rgb_stride};
    const int raw_stride = static_cast<int>(s.cap->stride);  // set before threads start

    // pop() returns nothing only when the queue is closed and empty, so this
    // loop ends only after capture has finished and every frame has been handled.
    while (auto raw = s.raw_queue.pop()) {
        // Copy the settings and unlock at once: the mutex must not be held during
        // the filter, or the output thread would stall on the next key press.
        Settings settings;
        {
            std::lock_guard<std::mutex> lock(s.settings_mutex);
            settings = s.settings;
        }

        Frame out = s.rgb_pool.acquire();
        out.timestamp_us = raw->timestamp_us;

        auto t0 = Clock::now();
        vc::yuyv_to_rgb24(raw->pixels.data(), raw_stride,
                          rgb.data(), rgb_stride, s.width, s.height);
        out.convert_ms = ms_since(t0);

        // The YUYV buffer isn't needed any more, so the capture thread can reuse it.
        s.raw_pool.release(std::move(*raw));

        // Filters need separate input and output buffers: 'rgb' in, out.pixels out.
        t0 = Clock::now();
        const vc::Image out_image{out.pixels.data(), s.width, s.height, rgb_stride};
        runner.apply(settings.filter, settings.params, rgb_image, out_image);
        if (settings.load_ms > 0)
            busy_wait(settings.load_ms);
        out.filter_ms = ms_since(t0);

        ++s.processed;

        if (auto rejected = s.out_queue.push(std::move(out)))
            s.rgb_pool.release(std::move(*rejected));
    }

    s.out_queue.close();
}

}  // namespace

/*
 * main() is also the OUTPUT THREAD. It has to be: SDL requires window events to be
 * handled on the thread that initialised video, so the window stays here while the
 * other two stages get their own threads.
 */
int main(int argc, char** argv)
{
    // ---- 1. Parse arguments --------------------------------------------
    const char* device     = "/dev/video0";
    long        max_frames = 0;       // 0 = run until the user quits
    Settings    settings;             // main's own copy of the current settings
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
            max_frames = std::strtol(argv[++i], nullptr, 10);
        } else if (std::strcmp(argv[i], "--load-ms") == 0 && i + 1 < argc) {
            settings.load_ms = std::max(0, std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--filter") == 0 && i + 1 < argc) {
            const long k = std::strtol(argv[++i], nullptr, 10);
            if (k < 1 || k > static_cast<long>(vc::Filter::Count)) {
                std::fprintf(stderr, "--filter must be 1-%d\n", static_cast<int>(vc::Filter::Count));
                return EXIT_FAILURE;
            }
            settings.filter = static_cast<vc::Filter>(k - 1);
        } else {
            device = argv[i];
        }
    }

    // ---- 2. Open the camera and the window -------------------------------
    struct capture cap;
    if (capture_open(&cap, device, kWidth, kHeight, kFps, kBuffers) != 0)
        return EXIT_FAILURE;

    // SDL_Init runs before any thread starts. It also turns Ctrl+C into an SDL_QUIT
    // event, so Ctrl+C shuts down through the same clean path as the Q key.
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        capture_close(&cap);
        return EXIT_FAILURE;
    }

    const int width  = static_cast<int>(cap.width);
    const int height = static_cast<int>(cap.height);

    SDL_Window* window = SDL_CreateWindow(
        "Virtual Camera - threaded pipeline",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        width, height, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = window
        ? SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED)
        : nullptr;
    SDL_Texture* texture = renderer
        ? SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING,
                            width, height)
        : nullptr;

    if (!texture || capture_start(&cap) != 0) {
        if (!texture)
            std::fprintf(stderr, "SDL window/renderer/texture: %s\n", SDL_GetError());
        if (texture)  SDL_DestroyTexture(texture);
        if (renderer) SDL_DestroyRenderer(renderer);
        if (window)   SDL_DestroyWindow(window);
        SDL_Quit();
        capture_close(&cap);
        return EXIT_FAILURE;
    }
    SDL_RenderSetLogicalSize(renderer, width, height);

    // ---- 3. Start the other two threads ----------------------------------
    Shared shared(&cap, width, height);
    shared.settings = settings;

    // std::ref passes the object itself. Without it, std::thread would try to
    // copy 'shared', and a struct holding mutexes can't be copied.
    std::thread capturer(capture_thread, std::ref(shared));
    std::thread processor(process_thread, std::ref(shared));

    // ---- 4. The output loop ------------------------------------------------
    long   shown_total    = 0;
    long   shown_window   = 0;       // frames shown in the current 1-second window
    double convert_ms_sum = 0.0;
    double filter_ms_sum  = 0.0;
    double latency_ms_sum = 0.0;
    double latency_ms_all = 0.0;
    double latency_ms_max = 0.0;
    auto   window_start   = Clock::now();
    auto   run_start      = Clock::now();
    bool   running        = true;

    while (running) {
        // 4a. Handle all pending keyboard and window events without waiting.
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) {
                running = false;
            } else if (ev.type == SDL_KEYDOWN) {
                const SDL_Keycode key = ev.key.keysym.sym;
                const Settings previous = settings;

                if (key == SDLK_ESCAPE || key == SDLK_q) {
                    running = false;
                } else if (key >= SDLK_1 &&
                           key < SDLK_1 + static_cast<int>(vc::Filter::Count)) {
                    settings.filter = static_cast<vc::Filter>(key - SDLK_1);
                } else if (key == SDLK_UP || key == SDLK_DOWN) {
                    settings.params.brightness = std::clamp(
                        settings.params.brightness + (key == SDLK_UP ? 5 : -5), -100, 100);
                    settings.filter = vc::Filter::BrightnessContrast;
                } else if (key == SDLK_RIGHT || key == SDLK_LEFT) {
                    settings.params.contrast = std::clamp(
                        settings.params.contrast + (key == SDLK_RIGHT ? 0.1f : -0.1f), 0.2f, 3.0f);
                    settings.filter = vc::Filter::BrightnessContrast;
                } else if (key == SDLK_r) {
                    settings.params = vc::FilterParams{};
                } else if (key == SDLK_h) {
                    settings.load_ms = settings.load_ms > 0 ? 0 : kToggleLoadMs;
                }

                // Publish the new settings for the process thread.
                {
                    std::lock_guard<std::mutex> lock(shared.settings_mutex);
                    shared.settings = settings;
                }

                // Times from different setups must not be averaged together.
                if (settings.filter != previous.filter || settings.load_ms != previous.load_ms) {
                    shown_window   = 0;
                    convert_ms_sum = 0.0;
                    filter_ms_sum  = 0.0;
                    latency_ms_sum = 0.0;
                    window_start   = Clock::now();
                }
            }
        }
        if (!running)
            break;

        // 4b. Wait for a finished frame, but not for long, so that keys still
        // work if the camera stalls.
        auto out = shared.out_queue.pop_for(std::chrono::milliseconds(kWaitMs));
        if (!out) {
            // finished() is true only if the capture thread hit an error, because
            // on a normal quit this loop has already ended.
            if (shared.out_queue.finished())
                break;
            continue;
        }

        // 4c. Copy the pixels to the texture and draw them.
        SDL_UpdateTexture(texture, nullptr, out->pixels.data(), width * 3);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);

        // 4d. Latency: from the driver receiving this frame until it was drawn.
        // This is the number that shows whether the video has fallen behind.
        const double latency_ms = (now_us() - out->timestamp_us) / 1000.0;
        latency_ms_sum += latency_ms;
        latency_ms_all += latency_ms;
        latency_ms_max  = std::max(latency_ms_max, latency_ms);
        convert_ms_sum += out->convert_ms;
        filter_ms_sum  += out->filter_ms;
        ++shown_total;
        ++shown_window;

        shared.rgb_pool.release(std::move(*out));

        // 4e. Once per second: update the title.
        const double elapsed = ms_since(window_start);
        if (elapsed >= 1000.0) {
            char title[224];
            std::snprintf(title, sizeof(title),
                          "%s%s | %.1f fps | latency %.0f ms | convert %.2f ms | filter %.2f ms"
                          " | dropped: driver %ld, queues %zu",
                          vc::filter_name(settings.filter),
                          settings.load_ms > 0 ? " + test load" : "",
                          shown_window * 1000.0 / elapsed,
                          latency_ms_sum / shown_window,
                          convert_ms_sum / shown_window,
                          filter_ms_sum / shown_window,
                          shared.driver_dropped.load(),
                          shared.raw_queue.dropped() + shared.out_queue.dropped());
            SDL_SetWindowTitle(window, title);
            shown_window   = 0;
            convert_ms_sum = 0.0;
            filter_ms_sum  = 0.0;
            latency_ms_sum = 0.0;
            window_start   = Clock::now();
        }

        if (max_frames > 0 && shown_total >= max_frames)
            running = false;
    }

    // ---- 5. Clean shutdown -----------------------------------------------------
    // The order matters:
    //   1. stop = true: the capture thread leaves its loop and closes raw_queue.
    //   2. The process thread drains raw_queue, sees the end, and closes out_queue.
    //   3. join() waits for both threads to really finish.
    //   4. Only then is the camera closed, because the capture thread uses it and
    //      the process thread reads its stride.
    // Nothing can hang: push never blocks, and close wakes any sleeping pop.
    const double run_s = ms_since(run_start) / 1000.0;
    shared.stop = true;
    capturer.join();
    processor.join();
    capture_close(&cap);

    // Frames the output thread never took. Counting them lets the summary add up.
    long left_in_queue = 0;
    while (auto leftover = shared.out_queue.pop())
        ++left_in_queue;

    std::printf("pipeline: %ld shown in %.1f s (%.1f fps), latency avg %.0f ms, max %.0f ms\n",
                shown_total, run_s, run_s > 0 ? shown_total / run_s : 0.0,
                shown_total > 0 ? latency_ms_all / shown_total : 0.0, latency_ms_max);
    std::printf("  frames: %ld captured, %ld processed, %zu dropped before processing,\n"
                "          %zu dropped after processing, %ld left in queue at exit\n",
                shared.captured.load(), shared.processed.load(), shared.raw_queue.dropped(),
                shared.out_queue.dropped(), left_in_queue);
    std::printf("  driver lost %ld frames; buffers allocated: %zu YUYV, %zu RGB\n",
                shared.driver_dropped.load(), shared.raw_pool.allocated(),
                shared.rgb_pool.allocated());

    const bool failed = shared.failed;
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
