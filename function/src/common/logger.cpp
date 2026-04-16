#include "common/logger.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <deque>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr size_t kMaxLogEntries = 200;
constexpr size_t kMaxModuleChars = 48;
constexpr size_t kMaxMessageChars = 240;
constexpr size_t kMaxSerializedBudgetBytes = 64 * 1024;

class LoggerState {
public:
    void append(const std::string& level, const std::string& module, const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (level == "INFO" && !info_enabled_) {
            return;
        }

        entries_.push_back(LoggerEntry {
            next_sequence_++,
            nowText(),
            level,
            limitTextSize(module, kMaxModuleChars),
            limitTextSize(message, kMaxMessageChars)
        });
        trimEntriesLocked();
        cv_.notify_all();
    }

    std::vector<LoggerEntry> snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return std::vector<LoggerEntry>(entries_.begin(), entries_.end());
    }

    std::vector<LoggerEntry> entriesAfter(std::uint64_t after_sequence) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return collectEntriesAfterLocked(after_sequence);
    }

    bool waitForEntries(
        std::uint64_t after_sequence,
        int timeout_ms,
        std::vector<LoggerEntry>& entries) const
    {
        std::unique_lock<std::mutex> lock(mutex_);
        const bool updated = cv_.wait_for(
            lock,
            std::chrono::milliseconds(std::max(timeout_ms, 0)),
            [this, after_sequence]() {
                return latestSequenceLocked() > after_sequence;
            });

        if (!updated) {
            entries.clear();
            return false;
        }

        entries = collectEntriesAfterLocked(after_sequence);
        return !entries.empty();
    }

    std::uint64_t latestSequence() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return latestSequenceLocked();
    }

    void setInfoEnabled(bool enabled) {
        std::lock_guard<std::mutex> lock(mutex_);
        info_enabled_ = enabled;
    }

    bool infoEnabled() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_enabled_;
    }

private:
    void trimEntriesLocked() {
        while (entries_.size() > kMaxLogEntries) {
            entries_.pop_front();
        }

        size_t estimated_bytes = 24;
        for (const auto& entry : entries_) {
            estimated_bytes += estimateEntryBytes(entry);
        }

        while (estimated_bytes > kMaxSerializedBudgetBytes && entries_.size() > 1) {
            estimated_bytes -= estimateEntryBytes(entries_.front());
            entries_.pop_front();
        }
    }

    std::uint64_t latestSequenceLocked() const {
        return entries_.empty() ? 0 : entries_.back().sequence;
    }

    std::vector<LoggerEntry> collectEntriesAfterLocked(std::uint64_t after_sequence) const {
        if (entries_.empty()) {
            return {};
        }

        if (after_sequence < entries_.front().sequence) {
            return std::vector<LoggerEntry>(entries_.begin(), entries_.end());
        }

        auto it = std::find_if(entries_.begin(), entries_.end(), [after_sequence](const LoggerEntry& entry) {
            return entry.sequence > after_sequence;
        });
        return std::vector<LoggerEntry>(it, entries_.end());
    }

    static std::string nowText() {
        const auto now = std::chrono::system_clock::now();
        const std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm {};
        localtime_r(&t, &tm);
        std::ostringstream out;
        out << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
        return out.str();
    }

    static std::string limitTextSize(const std::string& value, size_t max_chars) {
        if (value.size() <= max_chars) {
            return value;
        }
        if (max_chars <= 3) {
            return value.substr(0, max_chars);
        }
        return value.substr(0, max_chars - 3) + "...";
    }

    static size_t estimateEntryBytes(const LoggerEntry& entry) {
        return entry.timestamp.size() + entry.level.size() + entry.module.size() + entry.message.size() + 96;
    }

private:
    mutable std::mutex mutex_;
    mutable std::condition_variable cv_;
    std::deque<LoggerEntry> entries_;
    std::uint64_t next_sequence_ = 1;
    bool info_enabled_ = false;
};

LoggerState& loggerState() {
    static LoggerState state;
    return state;
}

}  // namespace

void log_info(const std::string& module, const std::string& msg) {
    loggerState().append("INFO", module, msg);
}

void log_warn(const std::string& module, const std::string& msg) {
    loggerState().append("WARN", module, msg);
}

void log_error(const std::string& module, const std::string& msg) {
    loggerState().append("ERROR", module, msg);
}

std::vector<LoggerEntry> logger_snapshot() {
    return loggerState().snapshot();
}

std::vector<LoggerEntry> logger_entries_after(std::uint64_t after_sequence) {
    return loggerState().entriesAfter(after_sequence);
}

bool logger_wait_for_entries(
    std::uint64_t after_sequence,
    int timeout_ms,
    std::vector<LoggerEntry>& entries) {
    return loggerState().waitForEntries(after_sequence, timeout_ms, entries);
}

std::uint64_t logger_latest_sequence() {
    return loggerState().latestSequence();
}

void logger_set_info_enabled(bool enabled) {
    loggerState().setInfoEnabled(enabled);
}

bool logger_info_enabled() {
    return loggerState().infoEnabled();
}
