#pragma once

#include <chrono>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

struct UartTraceContext {
    std::int64_t control_cycle_id = -1;
    std::string source = "unknown";
    std::string thread_tag = "unknown";
    std::string auto_avoid_state = "unknown";
    bool has_speed_value = false;
    int speed_value = 0;
    bool has_steering_value = false;
    int steering_value = 0;
};

class ScopedUartTraceContext {
public:
    explicit ScopedUartTraceContext(const UartTraceContext& context);
    ~ScopedUartTraceContext();

    ScopedUartTraceContext(const ScopedUartTraceContext&) = delete;
    ScopedUartTraceContext& operator=(const ScopedUartTraceContext&) = delete;

private:
    UartTraceContext previous_;
};

class UART {
public:
    UART(const std::string& device = "/dev/ttyAMA2", int baudrate = 115200);
    ~UART();

    bool init();
    bool close_uart();

    bool send_string(const std::string& str, bool quiet = false);
    bool send_string_wait_ok(const std::string& str, bool quiet = false, int timeout_ms = 300);
    std::string receive_string();

    void start_receive_thread();
    void stop_receive_thread();

private:
    struct WaitResult {
        bool matched = false;
        bool received = false;
        bool ok = false;
        std::chrono::steady_clock::time_point reply_steady_{};
        std::string reply_text;
    };

    void receive_loop();
    void pushHistoryLocked(const std::string& entry);
    void updateStatusSnapshotLocked() const;
    std::string nowIsoSeconds() const;
    WaitResult wait_for_reply_locked(int previous_recv_count, int timeout_ms);
    WaitResult wait_for_ok_reply_locked(int previous_recv_count, int timeout_ms);

private:
    int uart_fd_;
    std::string uart_dev_;
    int baudrate_;
    std::atomic<bool> running_;
    std::thread recv_thread_;
    int recv_counter_;
    mutable std::mutex status_mutex_;
    std::mutex recv_mutex_;
    std::condition_variable recv_cv_;
    std::string last_sent_;
    std::string last_received_;
    std::string reply_buffer_;
    std::string last_update_time_;
    std::chrono::steady_clock::time_point last_receive_steady_{};
    std::deque<std::string> recent_history_;
};
