#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct LoggerEntry {
    std::uint64_t sequence = 0;
    std::string timestamp;
    std::string level;
    std::string module;
    std::string message;
};

void log_info(const std::string& module, const std::string& msg);
void log_warn(const std::string& module, const std::string& msg);
void log_error(const std::string& module, const std::string& msg);

std::vector<LoggerEntry> logger_snapshot();
std::vector<LoggerEntry> logger_entries_after(std::uint64_t after_sequence);
bool logger_wait_for_entries(
    std::uint64_t after_sequence,
    int timeout_ms,
    std::vector<LoggerEntry>& entries);
std::uint64_t logger_latest_sequence();
void logger_set_info_enabled(bool enabled);
bool logger_info_enabled();
