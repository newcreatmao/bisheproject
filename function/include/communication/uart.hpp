#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

class UART {
public:
    UART(const std::string& device = "/dev/ttyAMA2", int baudrate = 115200);
    ~UART();

    bool init();
    bool close_uart();

    bool send_string(const std::string& str, bool quiet = false);
    std::string receive_string();

    void start_receive_thread();
    void stop_receive_thread();

private:
    void receive_loop();
    void pushHistoryLocked(const std::string& entry);
    void updateStatusSnapshotLocked() const;
    std::string nowIsoSeconds() const;
    bool wait_for_reply_locked(int previous_recv_count, int timeout_ms);

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
    std::string last_update_time_;
    std::deque<std::string> recent_history_;
};
