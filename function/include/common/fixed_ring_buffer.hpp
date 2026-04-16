#pragma once

#include <array>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

namespace bishe::common
{

struct FixedRingBufferStats
{
    std::size_t cached = 0;
    std::size_t capacity = 0;
    std::uint64_t total_pushed = 0;
    std::uint64_t overwritten = 0;
};

template<typename T, std::size_t N>
struct FixedRingBufferWindow
{
    std::array<T, N> items {};
    std::size_t count = 0;

    bool empty() const
    {
        return count == 0;
    }

    const T * latest() const
    {
        return count == 0 ? nullptr : &items[count - 1];
    }
};

template<typename T, std::size_t N>
class FixedRingBuffer
{
public:
    static_assert(N > 0, "FixedRingBuffer capacity must be greater than zero");

    void push(const T & value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool overwriting = count_ == N;
        storage_[next_index_] = value;
        next_index_ = (next_index_ + 1) % N;
        count_ = std::min<std::size_t>(count_ + 1, N);
        ++push_count_;
        if (overwriting) {
            ++overwrite_count_;
        }
    }

    std::optional<T> latest() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (count_ == 0) {
            return std::nullopt;
        }

        const std::size_t latest_index = (next_index_ + N - 1) % N;
        return storage_[latest_index];
    }

    FixedRingBufferWindow<T, N> recent(std::size_t max_count = N) const
    {
        std::lock_guard<std::mutex> lock(mutex_);

        FixedRingBufferWindow<T, N> window;
        window.count = std::min<std::size_t>(count_, std::min<std::size_t>(max_count, N));
        if (window.count == 0) {
            return window;
        }

        const std::size_t start_index = (next_index_ + N - window.count) % N;
        for (std::size_t i = 0; i < window.count; ++i) {
            window.items[i] = storage_[(start_index + i) % N];
        }
        return window;
    }

    void clear(bool reset_stats = true)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        next_index_ = 0;
        count_ = 0;
        if (reset_stats) {
            push_count_ = 0;
            overwrite_count_ = 0;
        }
    }

    std::size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return count_;
    }

    FixedRingBufferStats stats() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        FixedRingBufferStats stats;
        stats.cached = count_;
        stats.capacity = N;
        stats.total_pushed = push_count_;
        stats.overwritten = overwrite_count_;
        return stats;
    }

private:
    mutable std::mutex mutex_;
    std::array<T, N> storage_ {};
    std::size_t next_index_ = 0;
    std::size_t count_ = 0;
    std::uint64_t push_count_ = 0;
    std::uint64_t overwrite_count_ = 0;
};

}  // namespace bishe::common
