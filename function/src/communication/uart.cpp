#include "communication/uart.hpp"
#include "common/logger.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
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
constexpr const char* kUartAckTracePath = "/home/mao/use/project/Log/uart_ack_trace.csv";

struct ParsedCommand {
    std::string cmd_type = "unknown";
    bool has_value = false;
    int raw_value = 0;
    bool has_speed_value = false;
    int speed_value = 0;
    bool has_steering_value = false;
    int steering_value = 0;
};

std::string trimTrailingWhitespace(const std::string& value);

thread_local UartTraceContext g_uart_trace_context;

std::mutex& uartAckTraceMutex() {
    static std::mutex mutex;
    return mutex;
}

std::atomic<std::uint64_t>& uartAckTraceSequence() {
    static std::atomic<std::uint64_t> seq{1};
    return seq;
}

std::int64_t steadyClockNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::int64_t steadyClockNs(const std::chrono::steady_clock::time_point& value) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        value.time_since_epoch()).count();
}

std::string csvEscape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 8);
    escaped.push_back('"');
    for (const char ch : value) {
        if (ch == '"') {
            escaped += "\"\"";
        } else {
            escaped.push_back(ch);
        }
    }
    escaped.push_back('"');
    return escaped;
}

std::string csvNumberOrEmpty(long long value, bool valid = true) {
    if (!valid) {
        return "";
    }
    return std::to_string(value);
}

std::string csvDoubleOrEmpty(double value, bool valid = true, int precision = 3) {
    if (!valid || !std::isfinite(value)) {
        return "";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

std::string compactReplyText(const std::string& value, std::size_t max_chars = 120) {
    std::string compact = trimTrailingWhitespace(value);
    for (char& ch : compact) {
        if (ch == '\n' || ch == '\r') {
            ch = ' ';
        }
    }
    if (compact.size() > max_chars) {
        compact.resize(max_chars);
    }
    return compact;
}

std::string traceFileTimestampSuffix() {
    const std::time_t now = std::time(nullptr);
    std::tm local_tm {};
    localtime_r(&now, &local_tm);
    std::ostringstream out;
    out << std::put_time(&local_tm, "%Y%m%d_%H%M%S");
    return out.str();
}

std::string uartAckTraceHeader() {
    return
        "seq_id,control_cycle_id,source,thread_tag,cmd_type,cmd_text,"
        "tx_steady_ns,tx_steady_ms,rx_ok_steady_ns,rx_ok_steady_ms,ack_wait_ms,"
        "result,auto_avoid_state,speed_value,steering_value,extra";
}

void ensureCsvHeaderWithRotation(
    const std::filesystem::path& path,
    const std::string& expected_header,
    const std::string& backup_stem) {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);

    bool header_matches = false;
    if (std::filesystem::exists(path, ec) &&
        std::filesystem::file_size(path, ec) > 0) {
        std::ifstream input(path);
        std::string first_line;
        if (input.is_open() && std::getline(input, first_line)) {
            header_matches =
                trimTrailingWhitespace(first_line) ==
                trimTrailingWhitespace(expected_header);
        }
        if (!header_matches) {
            const std::filesystem::path backup_path =
                path.parent_path() /
                (backup_stem + "_" + traceFileTimestampSuffix() + path.extension().string());
            std::error_code rename_ec;
            std::filesystem::rename(path, backup_path, rename_ec);
            if (rename_ec) {
                std::ifstream source(path, std::ios::binary);
                std::ofstream backup(backup_path, std::ios::binary | std::ios::trunc);
                if (source.is_open() && backup.is_open()) {
                    backup << source.rdbuf();
                }
                std::error_code remove_ec;
                std::filesystem::remove(path, remove_ec);
            }
        }
    }

    if (!header_matches) {
        std::ofstream output(path, std::ios::trunc);
        if (!output.is_open()) {
            return;
        }
        output << expected_header << "\n";
    }
}

ParsedCommand parseCommandText(const std::string& cmd_text) {
    ParsedCommand parsed;
    if (cmd_text.size() < 5 || cmd_text[0] != '#' || cmd_text[1] != 'C') {
        return parsed;
    }

    const std::size_t eq_pos = cmd_text.find('=');
    const std::size_t end_pos = cmd_text.find('!');
    if (eq_pos == std::string::npos || end_pos == std::string::npos || eq_pos + 1 >= end_pos) {
        return parsed;
    }

    const std::string cmd_id = cmd_text.substr(2, eq_pos - 2);
    try {
        parsed.raw_value = std::stoi(cmd_text.substr(eq_pos + 1, end_pos - eq_pos - 1));
        parsed.has_value = true;
    } catch (...) {
        parsed.has_value = false;
    }

    if (cmd_id == "1") {
        parsed.cmd_type = "speed";
        if (parsed.has_value) {
            parsed.has_speed_value = true;
            parsed.speed_value = parsed.raw_value;
        }
    } else if (cmd_id == "2") {
        parsed.cmd_type = "angle";
        if (parsed.has_value) {
            parsed.has_steering_value = true;
            parsed.steering_value = parsed.raw_value;
        }
    } else if (cmd_id == "3") {
        parsed.cmd_type = "mode";
    }
    return parsed;
}

void appendUartAckTraceRow(
    std::uint64_t seq_id,
    const UartTraceContext& context,
    const ParsedCommand& parsed_command,
    const std::string& cmd_text,
    std::int64_t tx_steady_ns,
    bool has_rx_ok,
    std::int64_t rx_ok_steady_ns,
    double ack_wait_ms,
    const std::string& result,
    const std::string& extra) {
    std::lock_guard<std::mutex> lock(uartAckTraceMutex());
    ensureCsvHeaderWithRotation(
        std::filesystem::path(kUartAckTracePath),
        uartAckTraceHeader(),
        "uart_ack_trace_pre_header_sync");

    std::ofstream output(kUartAckTracePath, std::ios::app);
    if (!output.is_open()) {
        return;
    }

    const bool has_speed_value = context.has_speed_value || parsed_command.has_speed_value;
    const int speed_value = context.has_speed_value ? context.speed_value : parsed_command.speed_value;
    const bool has_steering_value =
        context.has_steering_value || parsed_command.has_steering_value;
    const int steering_value =
        context.has_steering_value ? context.steering_value : parsed_command.steering_value;
    const double tx_steady_ms = static_cast<double>(tx_steady_ns) / 1e6;
    const double rx_ok_steady_ms = static_cast<double>(rx_ok_steady_ns) / 1e6;

    output
        << seq_id << ","
        << context.control_cycle_id << ","
        << csvEscape(context.source) << ","
        << csvEscape(context.thread_tag) << ","
        << csvEscape(parsed_command.cmd_type) << ","
        << csvEscape(trimTrailingWhitespace(cmd_text)) << ","
        << tx_steady_ns << ","
        << csvDoubleOrEmpty(tx_steady_ms) << ","
        << csvNumberOrEmpty(rx_ok_steady_ns, has_rx_ok) << ","
        << csvDoubleOrEmpty(rx_ok_steady_ms, has_rx_ok) << ","
        << csvDoubleOrEmpty(ack_wait_ms, has_rx_ok && ack_wait_ms >= 0.0) << ","
        << csvEscape(result) << ","
        << csvEscape(context.auto_avoid_state) << ","
        << (has_speed_value ? std::to_string(speed_value) : std::string()) << ","
        << (has_steering_value ? std::to_string(steering_value) : std::string()) << ","
        << csvEscape(extra)
        << "\n";
}

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

bool containsOkReply(const std::string& value) {
    std::string normalized;
    normalized.reserve(value.size());
    for (const unsigned char ch : value) {
        normalized.push_back(static_cast<char>(std::tolower(ch)));
    }
    return normalized.find("ok") != std::string::npos;
}

}  // namespace

ScopedUartTraceContext::ScopedUartTraceContext(const UartTraceContext& context)
    : previous_(g_uart_trace_context) {
    g_uart_trace_context = context;
}

ScopedUartTraceContext::~ScopedUartTraceContext() {
    g_uart_trace_context = previous_;
}

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
    const auto context = g_uart_trace_context;
    const auto parsed_command = parseCommandText(str);
    const std::uint64_t seq_id = uartAckTraceSequence().fetch_add(1);
    const std::int64_t tx_steady_ns = steadyClockNowNs();
    if (uart_fd_ < 0) {
        appendUartAckTraceRow(
            seq_id,
            context,
            parsed_command,
            str,
            tx_steady_ns,
            false,
            0,
            -1.0,
            "uart_closed",
            "send_string");
        if (!quiet) {
            log_warn("UART", "send fail: uart not open");
        }
        return false;
    }

    if (str.empty()) {
        appendUartAckTraceRow(
            seq_id,
            context,
            parsed_command,
            str,
            tx_steady_ns,
            false,
            0,
            -1.0,
            "skipped",
            "empty_command");
        if (!quiet) {
            log_warn("UART", "send fail: empty string");
        }
        return false;
    }

    int previous_recv_count = 0;
    {
        std::lock_guard<std::mutex> recv_lock(recv_mutex_);
        previous_recv_count = recv_counter_;
        reply_buffer_.clear();
    }

    const ssize_t sent = write(uart_fd_, str.c_str(), str.size());
    if (sent < 0) {
        appendUartAckTraceRow(
            seq_id,
            context,
            parsed_command,
            str,
            tx_steady_ns,
            false,
            0,
            -1.0,
            "write_fail",
            "errno=" + std::to_string(errno));
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
    const WaitResult wait_result =
        wait_for_reply_locked(previous_recv_count, kCommandReplyTimeoutMs);
    const bool has_rx_ok = wait_result.ok;
    const std::int64_t rx_ok_steady_ns =
        has_rx_ok ? steadyClockNs(wait_result.reply_steady_) : 0;
    const double ack_wait_ms =
        has_rx_ok ? static_cast<double>(rx_ok_steady_ns - tx_steady_ns) / 1e6 : -1.0;
    const std::string result =
        wait_result.matched ?
            (wait_result.ok ? "ok" : "unexpected_reply") :
            (wait_result.received ? "unexpected_reply" : "timeout");
    appendUartAckTraceRow(
        seq_id,
        context,
        parsed_command,
        str,
        tx_steady_ns,
        has_rx_ok,
        rx_ok_steady_ns,
        ack_wait_ms,
        result,
        "timeout_ms=" + std::to_string(kCommandReplyTimeoutMs) +
            ";reply=" + compactReplyText(wait_result.reply_text));
    if (!wait_result.matched) {
        if (!quiet) {
            log_warn("UART", "send timeout waiting reply: " + str);
        }
        return false;
    }
    return true;
}

bool UART::send_string_wait_ok(const std::string& str, bool quiet, int timeout_ms) {
    const auto context = g_uart_trace_context;
    const auto parsed_command = parseCommandText(str);
    const std::uint64_t seq_id = uartAckTraceSequence().fetch_add(1);
    const std::int64_t tx_steady_ns = steadyClockNowNs();
    if (uart_fd_ < 0) {
        appendUartAckTraceRow(
            seq_id,
            context,
            parsed_command,
            str,
            tx_steady_ns,
            false,
            0,
            -1.0,
            "uart_closed",
            "send_string_wait_ok");
        if (!quiet) {
            log_warn("UART", "send fail: uart not open");
        }
        return false;
    }

    if (str.empty()) {
        appendUartAckTraceRow(
            seq_id,
            context,
            parsed_command,
            str,
            tx_steady_ns,
            false,
            0,
            -1.0,
            "skipped",
            "empty_command");
        if (!quiet) {
            log_warn("UART", "send fail: empty string");
        }
        return false;
    }

    int previous_recv_count = 0;
    {
        std::lock_guard<std::mutex> recv_lock(recv_mutex_);
        previous_recv_count = recv_counter_;
        reply_buffer_.clear();
    }

    const ssize_t sent = write(uart_fd_, str.c_str(), str.size());
    if (sent < 0) {
        appendUartAckTraceRow(
            seq_id,
            context,
            parsed_command,
            str,
            tx_steady_ns,
            false,
            0,
            -1.0,
            "write_fail",
            "errno=" + std::to_string(errno));
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
    const WaitResult wait_result = wait_for_ok_reply_locked(previous_recv_count, timeout_ms);
    const bool has_rx_ok = wait_result.ok;
    const std::int64_t rx_ok_steady_ns =
        has_rx_ok ? steadyClockNs(wait_result.reply_steady_) : 0;
    const double ack_wait_ms =
        has_rx_ok ? static_cast<double>(rx_ok_steady_ns - tx_steady_ns) / 1e6 : -1.0;
    const std::string result =
        wait_result.matched ?
            "ok" :
            (wait_result.received ? "unexpected_reply" : "timeout");
    appendUartAckTraceRow(
        seq_id,
        context,
        parsed_command,
        str,
        tx_steady_ns,
        has_rx_ok,
        rx_ok_steady_ns,
        ack_wait_ms,
        result,
        "timeout_ms=" + std::to_string(timeout_ms) +
            ";reply=" + compactReplyText(wait_result.reply_text));
    if (!wait_result.matched) {
        if (!quiet) {
            log_warn("UART", "send timeout waiting ok reply: " + str);
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
                reply_buffer_ += data;
                if (reply_buffer_.size() > 256) {
                    reply_buffer_.erase(0, reply_buffer_.size() - 256);
                }
                recv_counter_++;
                last_receive_steady_ = std::chrono::steady_clock::now();
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

UART::WaitResult UART::wait_for_ok_reply_locked(int previous_recv_count, int timeout_ms) {
    std::unique_lock<std::mutex> lock(recv_mutex_);
    WaitResult result;
    result.matched = recv_cv_.wait_for(
        lock,
        std::chrono::milliseconds(timeout_ms),
        [this, previous_recv_count]() {
            return recv_counter_ > previous_recv_count && containsOkReply(reply_buffer_);
        });
    result.received = recv_counter_ > previous_recv_count;
    result.ok = result.received && containsOkReply(reply_buffer_);
    if (result.received) {
        result.reply_steady_ = last_receive_steady_;
        result.reply_text = reply_buffer_;
    }
    return result;
}

UART::WaitResult UART::wait_for_reply_locked(int previous_recv_count, int timeout_ms) {
    std::unique_lock<std::mutex> lock(recv_mutex_);
    WaitResult result;
    result.matched = recv_cv_.wait_for(
        lock,
        std::chrono::milliseconds(timeout_ms),
        [this, previous_recv_count]() {
            return recv_counter_ > previous_recv_count;
        });
    result.received = recv_counter_ > previous_recv_count;
    result.ok = result.received && containsOkReply(reply_buffer_);
    if (result.received) {
        result.reply_steady_ = last_receive_steady_;
        result.reply_text = reply_buffer_;
    }
    return result;
}
