/*
 * frame_queue.h - Step 4: thread-safe bounded queue that drops the OLDEST item.
 *
 * WHAT IT DOES
 *   Passes items from one thread to another, in order. It holds a fixed number
 *   of items. If the producer pushes while it's full, the oldest item is thrown
 *   out to make room, so the consumer always gets recent frames. The producer
 *   never waits; the consumer sleeps while the queue is empty.
 *
 * HOW IT CONNECTS TO THE REST OF THE PROJECT
 *   - pipeline/threaded_pipeline.cpp uses one queue between the capture and
 *     process threads, and another between the process and output threads.
 *   - It knows nothing about video: T is whatever the pipeline passes along.
 *   - Header only, because a template must be visible wherever it's used. So
 *     it needs no entry in CMakeLists.txt.
 *
 * WHY DROP THE OLDEST FRAME?
 *   This is live video. If processing falls behind, a blocked capture thread
 *   would make the driver lose frames anyway, and an unbounded queue would use
 *   more and more memory while the video lags further behind. Dropping the
 *   oldest frame keeps both memory use and delay limited.
 *
 * THE THREE RULES THIS CODE FOLLOWS
 *   1. Every read or write of shared state happens with mutex_ locked.
 *   2. A consumer waits inside a loop that re-checks its condition.
 *   3. Heavy work (filters, drawing) never happens while mutex_ is locked.
 */

#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

namespace vc {

template <typename T>
class FrameQueue {
public:
    // All slots are created up front, so normal operation doesn't allocate
    // memory for the slots. T must be default-constructible and movable.
    // capacity must be at least 1, because positions are computed with % capacity.
    explicit FrameQueue(size_t capacity) : slots_(capacity) {}

    // A mutex can't be copied, and two queues sharing one identity makes no
    // sense, so copying is forbidden at compile time.
    FrameQueue(const FrameQueue&) = delete;
    FrameQueue& operator=(const FrameQueue&) = delete;

    /*
     * Add an item, never blocking. The return value hands back anything that
     * did NOT end up in the queue, so the caller can reuse its memory:
     *   - queue had room:   returns nothing
     *   - queue was full:   returns the oldest item, which was removed
     *   - queue was closed: returns the item that was passed in
     */
    std::optional<T> push(T item)
    {
        std::optional<T> rejected;
        {
            // lock_guard locks now and unlocks at the closing brace, including
            // if an exception is thrown. The mutex can't be left locked by mistake.
            std::lock_guard<std::mutex> lock(mutex_);

            if (closed_)
                return item;

            if (count_ == slots_.size()) {
                // Full: take the oldest item out, and move head_ past it.
                rejected = std::move(slots_[head_]);
                head_ = (head_ + 1) % slots_.size();
                --count_;
                ++dropped_;
            }

            // Ring buffer: the next free slot is count_ places after head_, and
            // % wraps the position back to 0 at the end of the array.
            slots_[(head_ + count_) % slots_.size()] = std::move(item);
            ++count_;
        }

        // Wake the consumer if it's asleep. This is done after unlocking.
        // Otherwise the consumer could wake up and immediately have to wait for
        // a mutex we still hold.
        not_empty_.notify_one();
        return rejected;
    }

    /*
     * Remove the oldest item, sleeping while the queue is empty.
     * Returns nothing only when the queue is closed AND empty, which tells the
     * consumer thread to finish. Items still queued at close are delivered first.
     */
    std::optional<T> pop()
    {
        // unique_lock rather than lock_guard, because wait() must be able to
        // unlock and re-lock it.
        std::unique_lock<std::mutex> lock(mutex_);

        // wait() atomically unlocks the mutex and sleeps. "Atomically" prevents
        // a lost wakeup: a push can't slip in between our check and our sleep.
        // When woken, wait() re-locks the mutex and checks the condition again.
        // The recheck is required because condition variables are allowed to
        // wake up with nothing to do (a "spurious wakeup").
        not_empty_.wait(lock, [this] { return count_ > 0 || closed_; });

        if (count_ == 0)
            return std::nullopt;       // closed and fully drained

        return take_oldest();
    }

    /*
     * Same as pop(), but sleeps for at most 'timeout'. The output thread uses
     * this because it also has to handle keyboard and window events, so it can't
     * sleep forever if the camera stalls.
     * Returning nothing means "timed out" or "finished": finished() tells which.
     */
    std::optional<T> pop_for(std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        // Same rules as wait() above. If the time runs out, wait_for() still
        // re-locks the mutex, so reading count_ below is safe either way.
        not_empty_.wait_for(lock, timeout, [this] { return count_ > 0 || closed_; });

        if (count_ == 0)
            return std::nullopt;       // timed out, or closed and fully drained

        return take_oldest();
    }

    /* True once the queue is closed and empty: no item will ever come out again. */
    bool finished() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return closed_ && count_ == 0;
    }

    /*
     * Start shutdown: later pushes are rejected, and every sleeping consumer
     * wakes up. Without this, a thread asleep in pop() could wait forever and
     * joining it would hang the program.
     */
    void close()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        // notify_all, not notify_one: every waiter must see the close.
        not_empty_.notify_all();
    }

    /* How many items have been thrown away so far. */
    size_t dropped() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_;
    }

private:
    // Move the oldest item out of the ring. The caller must hold mutex_ and must
    // have checked that count_ > 0.
    std::optional<T> take_oldest()
    {
        std::optional<T> item(std::move(slots_[head_]));
        head_ = (head_ + 1) % slots_.size();
        --count_;
        return item;
    }

    // mutable so that dropped() and finished(), which are const, can still lock it.
    mutable std::mutex      mutex_;
    std::condition_variable not_empty_;   // signalled when an item arrives or on close

    // Everything below is protected by mutex_.
    std::vector<T> slots_;                // fixed-size ring buffer
    size_t head_    = 0;                  // position of the oldest item
    size_t count_   = 0;                  // how many items are stored
    size_t dropped_ = 0;                  // items removed because the queue was full
    bool   closed_  = false;
};

}  // namespace vc
