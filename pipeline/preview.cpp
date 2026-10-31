/*
 * preview.cpp - Step 2: live webcam preview in an SDL2 window, with FPS.
 *
 * WHAT IT DOES
 *   Opens the camera, then loops: wait for a frame -> convert YUYV to RGB
 *   straight into an SDL texture -> give the camera buffer back -> draw the
 *   texture in the window. Shows FPS, conversion time and dropped frames in
 *   the window title. Quit with Esc, Q, the window's close button, or Ctrl+C.
 *
 * HOW IT CONNECTS TO THE REST OF THE PROJECT
 *   - capture/v4l2_capture.h : capture_open/start/dequeue/requeue/close (C library)
 *   - pipeline/convert.h     : vc::yuyv_to_rgb24 (our hand-written converter)
 *   - SDL2                   : ONLY used to show pixels on screen (window,
 *                              texture upload, key events). No SDL drawing or
 *                              image processing, per the project rules.
 *   - Step 3 adds filters between "convert" and "draw".
 *   - Step 4 splits this single loop into capture / process / output threads.
 *
 * Usage: ./preview [device] [--frames N]
 *   --frames N  quit after N frames and print a summary (used for testing
 *               and, later, for benchmarks)
 */

#include <SDL.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "convert.h"
#include "v4l2_capture.h"   // C header, but it has extern "C" so C++ can link to it

namespace {

constexpr int kWidth   = 640;
constexpr int kHeight  = 480;
constexpr int kFps     = 30;
constexpr int kBuffers = 4;

/*
 * How long capture_dequeue() may wait for a frame. Short on purpose: while
 * we wait for the camera, we can't react to the keyboard or the close
 * button. 100 ms keeps the window responsive even if the camera stalls.
 * In normal operation a frame arrives every 33 ms, so we rarely hit it.
 */
constexpr int kDequeueTimeoutMs = 100;

/* Monotonic clock in milliseconds (never jumps, unlike wall-clock time). */
using Clock = std::chrono::steady_clock;
double ms_since(Clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

}  // namespace

int main(int argc, char** argv)
{
    // ---- 1. Parse arguments --------------------------------------------
    const char* device     = "/dev/video0";
    long        max_frames = 0;                 // 0 = run until the user quits
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc)
            max_frames = std::strtol(argv[++i], nullptr, 10);
        else
            device = argv[i];
    }

    // ---- 2. Open the camera (our C module) -------------------------------
    struct capture cap;
    if (capture_open(&cap, device, kWidth, kHeight, kFps, kBuffers) != 0)
        return EXIT_FAILURE;

    // ---- 3. Create the window ---------------------------------------------
    // SDL_Init(VIDEO) connects to the display server (Wayland or X11).
    // SDL also installs a SIGINT handler, so Ctrl+C becomes an SDL_QUIT event
    // and we shut down cleanly instead of being killed mid-stream.
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        capture_close(&cap);
        return EXIT_FAILURE;
    }

    // Window sized to the frame the driver ACTUALLY gave us (cap.width/height),
    // not our constants: S_FMT may have adjusted them.
    SDL_Window* window = SDL_CreateWindow(
        "Virtual Camera - preview",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        static_cast<int>(cap.width), static_cast<int>(cap.height),
        SDL_WINDOW_RESIZABLE);

    // The renderer is SDL's link to the GPU. ACCELERATED = draw with the GPU
    // (OpenGL/Vulkan underneath). No VSYNC: the camera already paces us at
    // 30 fps, and waiting for the 60 Hz display refresh would only add latency.
    SDL_Renderer* renderer = window
        ? SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED)
        : nullptr;

    // A texture is an image stored in GPU memory. STREAMING = we'll replace its
    // contents every frame (SDL keeps a CPU-side copy we can write into).
    // RGB24 = 3 bytes per pixel in R,G,B order: exactly what our converter writes.
    SDL_Texture* texture = renderer
        ? SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING,
                            static_cast<int>(cap.width), static_cast<int>(cap.height))
        : nullptr;

    if (!texture) {
        std::fprintf(stderr, "SDL window/renderer/texture: %s\n", SDL_GetError());
        if (renderer) SDL_DestroyRenderer(renderer);
        if (window)   SDL_DestroyWindow(window);
        SDL_Quit();
        capture_close(&cap);
        return EXIT_FAILURE;
    }

    // If the user resizes the window, scale the 640x480 image to fit while
    // keeping its 4:3 shape (black bars instead of stretching).
    SDL_RenderSetLogicalSize(renderer, static_cast<int>(cap.width), static_cast<int>(cap.height));

    // ---- 4. Start streaming -------------------------------------------------
    if (capture_start(&cap) != 0) {
        SDL_DestroyTexture(texture);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        capture_close(&cap);
        return EXIT_FAILURE;
    }

    // ---- 5. Main loop ---------------------------------------------------------
    // Statistics for the title bar, reset every second.
    long     frames_total   = 0;
    long     frames_window  = 0;      // frames shown in the current 1-second window
    double   convert_ms_sum = 0.0;    // total conversion time in this window
    long     dropped_total  = 0;      // frames the driver produced that we never saw
    uint32_t last_seq       = 0;
    bool     have_seq       = false;
    auto     window_start   = Clock::now();
    auto     run_start      = Clock::now();
    bool     running        = true;
    int      exit_code      = EXIT_SUCCESS;

    while (running) {
        // 5a. Handle all pending window/keyboard events WITHOUT waiting.
        // SDL_PollEvent returns 0 immediately when the queue is empty.
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT)                                   // close button or Ctrl+C
                running = false;
            else if (ev.type == SDL_KEYDOWN &&
                     (ev.key.keysym.sym == SDLK_ESCAPE || ev.key.keysym.sym == SDLK_q))
                running = false;
        }
        if (!running)
            break;

        // 5b. Wait (up to 100 ms) for the next camera frame. We now own the buffer.
        struct frame f;
        int r = capture_dequeue(&cap, &f, kDequeueTimeoutMs);
        if (r == 1)            // timeout: no frame yet, go back and check events
            continue;
        if (r == -1) {         // real error (e.g. camera unplugged)
            exit_code = EXIT_FAILURE;
            break;
        }

        // Sequence numbers should go up by exactly 1. A bigger jump means the
        // driver had no free buffer when a frame arrived (we were too slow),
        // so it dropped frames. This becomes important in Step 3/4.
        if (have_seq && f.sequence > last_seq + 1)
            dropped_total += f.sequence - last_seq - 1;
        last_seq = f.sequence;
        have_seq = true;

        // 5c. Convert YUYV -> RGB directly INTO the texture's memory.
        // SDL_LockTexture gives us a pointer to the texture's pixel buffer and
        // its pitch (= stride: bytes per row; SDL may pad rows too). Writing
        // there avoids converting into our own buffer and then copying it.
        void* tex_pixels = nullptr;
        int   tex_pitch  = 0;
        if (SDL_LockTexture(texture, nullptr, &tex_pixels, &tex_pitch) == 0) {
            auto t0 = Clock::now();
            vc::yuyv_to_rgb24(f.data, static_cast<int>(cap.stride),
                              static_cast<uint8_t*>(tex_pixels), tex_pitch,
                              static_cast<int>(cap.width), static_cast<int>(cap.height));
            convert_ms_sum += ms_since(t0);
            SDL_UnlockTexture(texture);   // SDL uploads the pixels to the GPU
        }

        // 5d. Give the camera buffer back AS SOON AS we've read it, before
        // drawing. The fewer buffers we hold, the less chance the driver runs
        // out of empty ones and drops a frame.
        if (capture_requeue(&cap, &f) != 0) {
            exit_code = EXIT_FAILURE;
            break;
        }

        // 5e. Draw: clear, copy the texture to the window (scaled to fit), show.
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);     // swap the finished image onto the screen

        ++frames_total;
        ++frames_window;

        // 5f. Once per second: update the title with FPS and timings.
        double elapsed = ms_since(window_start);
        if (elapsed >= 1000.0) {
            double fps = frames_window * 1000.0 / elapsed;
            char title[128];
            std::snprintf(title, sizeof(title),
                          "Virtual Camera - %.1f fps | convert %.2f ms | dropped %ld",
                          fps, convert_ms_sum / frames_window, dropped_total);
            SDL_SetWindowTitle(window, title);
            frames_window  = 0;
            convert_ms_sum = 0.0;
            window_start   = Clock::now();
        }

        if (max_frames > 0 && frames_total >= max_frames)
            running = false;
    }

    // ---- 6. Shut down in reverse order of creation ------------------------------
    double run_s = ms_since(run_start) / 1000.0;
    std::printf("preview: %ld frames in %.1f s (%.1f fps avg), %ld dropped\n",
                frames_total, run_s, run_s > 0 ? frames_total / run_s : 0.0, dropped_total);

    capture_close(&cap);            // STREAMOFF, munmap, close the camera
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();                     // disconnect from the display server
    return exit_code;
}
