#include "communication/uart.hpp"
#include "common/logger.hpp"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace {

constexpr int kCommandReplyTimeoutMs = 80;
constexpr const char* kStm32StatusSnapshotPath = "/tmp/project_stm32_status.txt";

speed_t getBaudRate(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default: return B9600;
    }
}

std::string trimTrailingWhitespace(const std::string& value) {
    const auto end = value.find_last_not_of(" \t\r\n");
    if (end == std::string::npos) {
        return "";
    }
    return value.substr(0, end + 1);
}

}  // namespace

UART::UART(const std::string& device, int baudrate)
    : uart_fd_(-1),
      uart_dev_(device),
      baudrate_(baudrate),
      running_(false),
      recv_thread_(),
      recv_counter_(0) {}

UART::~UART() {
    stop_receive_thread();
    close_uart();
}

bool UART::init() {
    uart_fd_ = open(uart_dev_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart_fd_ < 0) {
        log_error("UART", "open fail: " + uart_dev_ + ", errno=" + std::to_string(errno));
        return false;
    }

    struct termios options {};
    if (tcgetattr(uart_fd_, &options) != 0) {
        log_error("UART", "tcgetattr fail, errno=" + std::to_string(errno));
        close(uart_fd_);
        uart_fd_ = -1;
        return false;
    }

    const speed_t speed = getBaudRate(baudrate_);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~OPOST;

    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;

    tcflush(uart_fd_, TCIFLUSH);
    if (tcsetattr(uart_fd_, TCSANOW, &options) != 0) {
        log_error("UART", "tcsetattr fail, errno=" + std::to_string(errno));
        close(uart_fd_);
        uart_fd_ = -1;
        return false;
    }

    log_info("UART", "init success: " + uart_dev_ + ", baud=" + std::to_string(baudrate_));
    return true;
}

bool UART::close_uart() {
    if (uart_fd_ >= 0) {
        if (close(uart_fd_) == 0) {
            uart_fd_ = -1;
            std::remove(kStm32StatusSnapshotPath);
            log_info("UART", "close success: " + uart_dev_);
            return true;
        }
        log_error("UART", "close fail: " + uart_dev_ + ", errno=" + std::to_string(errno));
        return false;
    }
    return true;
}

bool UART::send_string(const std::string& str, bool quiet) {
    if (uart_fd_ < 0) {
        if (!quiet) {
            log_warn("UART", "send fail: uart not open");
        }
        return false;
    }

    if (str.empty()) {
        if (!quiet) {
            log_warn("UART", "send fail: empty string");
        }
        return false;
    }

    int previous_recv_count = 0;
    {
        std::lock_guard<std::mutex> recv_lock(recv_mutex_);
        previous_recv_count = recv_counter_;
    }

    const ssize_t sent = write(uart_fd_, str.c_str(), str.size());
    if (sent < 0) {
        if (!quiet) {
            log_error("UART", "send fail, errno=" + std::to_string(errno));
        }
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        last_sent_ = str;
        last_update_time_ = nowIsoSeconds();
        pushHistoryLocked(last_update_time_ + " TX " + trimTrailingWhitespace(str));
        updateStatusSnapshotLocked();
    }

    if (!quiet) {
        log_info("UART", "send: " + str);
    }
    if (!wait_for_reply_locked(previous_recv_count, kCommandReplyTimeoutMs)) {
        if (!quiet) {
            log_warn("UART", "send timeout waiting reply: " + str);
        }
        return false;
    }
    return true;
}

std::string UART::receive_string() {
    if (uart_fd_ < 0) {
        return "";
    }

    char buffer[256];
    const int len = read(uart_fd_, buffer, sizeof(buffer) - 1);
    if (len > 0) {
        buffer[len] = '\0';
        return std::string(buffer);
    }

    return "";
}

void UART::receive_loop() {
    while (running_.load()) {
        const std::string data = receive_string();
        if (!data.empty()) {
            {
                std::lock_guard<std::mutex> recv_lock(recv_mutex_);
                recv_counter_++;
            }
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                last_received_ = data;
                last_update_time_ = nowIsoSeconds();
                pushHistoryLocked(last_update_time_ + " RX " + trimTrailingWhitespace(data));
                updateStatusSnapshotLocked();
            }
            recv_cv_.notify_all();
            log_info("UART", "recv[" + std::to_string(recv_counter_) + "]: " + data);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void UART::start_receive_thread() {
    if (!running_.load()) {
        running_.store(true);
        recv_thread_ = std::thread(&UART::receive_loop, this);
        log_info("UART", "receive thread started");
    }
}

void UART::stop_receive_thread() {
    if (running_.load()) {
        running_.store(false);
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
        log_info("UART", "receive thread stopped");
    }
}

std::string UART::nowIsoSeconds() const {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm {};
    localtime_r(&t, &tm);
    std::ostringstream out;
    out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S");
    return out.str();
}

void UART::pushHistoryLocked(const std::string& entry) {
    if (entry.empty()) {
        return;
    }
    recent_history_.push_back(entry);
    while (recent_history_.size() > 5) {
        recent_history_.pop_front();
    }
}

void UART::updateStatusSnapshotLocked() const {
    std::ofstream output(kStm32StatusSnapshotPath, std::ios::trunc);
    if (!output.is_open()) {
        return;
    }
    output << "device=" << uart_dev_ << "\n";
    output << "baudrate=" << baudrate_ << "\n";
    output << "last_update_time=" << last_update_time_ << "\n";
    output << "last_sent=" << last_sent_ << "\n";
    output << "last_received=" << last_received_ << "\n";
    output << "recent_history=\n";
    for (const auto& entry : recent_history_) {
        output << entry << "\n";
    }
}

bool UART::wait_for_reply_locked(int previous_recv_count, int timeout_ms) {
    std::unique_lock<std::mutex> lock(recv_mutex_);
    return recv_cv_.wait_for(
        lock,
        std::chrono::milliseconds(timeout_ms),
        [this, previous_recv_count]() {
            return recv_counter_ > previous_recv_count;
        });
}
