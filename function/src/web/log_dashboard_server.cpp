#include "web/log_dashboard_server.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "common/logger.hpp"
#include "communication/uart.hpp"
#include "control/to_stm.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <cstring>
#include <deque>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

namespace fs = std::filesystem;

constexpr char kLogModule[] = "web";
constexpr char kAllfileMountPrefix[] = "/allfile";
constexpr std::chrono::seconds kRgbYoloFreshWindow(5);
constexpr std::chrono::seconds kSensorFreshWindow(3);
constexpr std::chrono::seconds kSensorStartupGrace(10);
constexpr std::chrono::seconds kStartupInfoWindow(18);
constexpr std::chrono::milliseconds kRuntimeLogPollInterval(400);
// 20 Hz auto-avoid loop: allow steering-only resend on roughly every control tick.
constexpr std::chrono::milliseconds kAutoAvoidCommandMinInterval(40);
constexpr std::size_t kRuntimeBootstrapLineCount = 120;
// 20 Hz auto-avoid loop: smaller per-tick steering deltas should still be dispatched.
constexpr int kAutoAvoidSteeringResendThreshold = 6;
constexpr int kAutoAvoidSpeedResendThreshold = 2;
// 20 Hz auto-avoid loop period while preserving the existing sleep_until pacing model.
constexpr std::chrono::milliseconds kAutoAvoidControlPeriod(50);
constexpr std::chrono::milliseconds kAutoWorkspaceControlPeriod(100);
constexpr std::size_t kGpsMedianWindowSize = 5;
constexpr std::size_t kGpsOriginStabilityWindowSize = 15;
constexpr double kGpsFilterEmaAlpha = 0.28;
constexpr double kGpsOriginStableRadiusM = 0.90;
constexpr double kGpsOriginStableHorizontalStdDevM = 4.0;
constexpr double kAutoWorkspaceHeadingCourseMinTravelM = 1.4;
constexpr double kAutoWorkspaceHeadingOffsetEmaAlpha = 0.25;
constexpr double kAutoWorkspaceGpsOffsetXM = -0.68;
constexpr double kAutoWorkspaceGpsOffsetYM = 0.28;
constexpr int kStm32StartSpeedRaw = 30;
constexpr double kStm32ToWebSpeedScale = 0.89;
constexpr int kStm32StartAngleCommand = 0;
constexpr int kAutoWorkspaceCruiseSpeedCmS = 30;
constexpr int kAutoWorkspaceApproachSpeedCmS = 18;
constexpr double kAutoWorkspaceApproachDistanceM = 4.0;
constexpr double kAutoWorkspaceArriveDistanceM = 1.8;
constexpr double kAutoWorkspacePathLookaheadM = 5.0;
constexpr double kAutoWorkspaceRoutePointSpacingM = 1.2;
constexpr std::size_t kAutoWorkspaceRouteMaxPoints = 180;
constexpr double kAutoWorkspaceHeadingDeadbandDeg = 2.0;
constexpr double kAutoWorkspaceHeadingKp = 1.6;
constexpr double kAutoWorkspaceMaxSteeringDeg = 28.0;
constexpr int kAutoWorkspaceEncoderPer10Deg = 40;
constexpr char kAutoWorkspaceDefaultOsrmBaseUrl[] = "http://router.project-osrm.org";
constexpr char kStackProcessNeedle[] = "project_stack.launch.py";
constexpr char kStackLogModule[] = "STACK";
constexpr char kStm32StatusSnapshotPath[] = "/tmp/project_stm32_status.txt";
constexpr char kAutoAvoidCycleTracePath[] = "/home/mao/use/project/Log/auto_avoid_cycle_trace.csv";

struct ComponentProcessState {
    bool running = false;
    std::chrono::system_clock::time_point started_at {};
};

struct StreamState {
    bool snapshot_sent = false;
    std::uint64_t last_sequence = 0;
};

fs::path detectSourceProjectRoot();
std::string extractJsonRawField(const std::string& json, const std::string& key);

std::string jsonEscape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 16);
    static constexpr char kHexDigits[] = "0123456789ABCDEF";
    for (const unsigned char ch : value) {
        switch (ch) {
            case '\\': escaped += "\\\\"; break;
            case '"': escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default:
                if (ch < 0x20) {
                    escaped += "\\u00";
                    escaped += kHexDigits[(ch >> 4) & 0x0F];
                    escaped += kHexDigits[ch & 0x0F];
                } else {
                    escaped += static_cast<char>(ch);
                }
                break;
        }
    }
    return escaped;
}

std::string boolJson(bool value) {
    return value ? "true" : "false";
}

int stm32SpeedToWebSpeedCmPerSec(int stm32_speed) {
    return static_cast<int>(
        std::ceil(static_cast<double>(stm32_speed) * kStm32ToWebSpeedScale));
}

std::string numberJson(double value, int precision = 3) {
    if (!std::isfinite(value)) {
        return "null";
    }

    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

double medianValue(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }

    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    if ((values.size() % 2) == 1) {
        return values[middle];
    }
    return (values[middle - 1] + values[middle]) * 0.5;
}

std::string envStringOrDefault(const char* key, const std::string& fallback) {
    if (const char* value = std::getenv(key)) {
        if (*value != '\0') {
            return value;
        }
    }
    return fallback;
}

int envIntOrDefault(const char* key, int fallback) {
    if (const char* value = std::getenv(key)) {
        try {
            return std::stoi(value);
        } catch (...) {
            return fallback;
        }
    }
    return fallback;
}

std::string trimWhitespace(const std::string& value) {
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

std::string csvField(const std::string& value) {
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

std::string csvBool(bool value) {
    return value ? "true" : "false";
}

std::string csvNumber(double value, int precision = 3) {
    if (!std::isfinite(value)) {
        return "";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

std::string csvIntOrEmpty(int value, bool valid = true) {
    return valid ? std::to_string(value) : std::string();
}

std::int64_t steadyNs(const std::chrono::steady_clock::time_point& value) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        value.time_since_epoch()).count();
}

double steadyMs(const std::chrono::steady_clock::time_point& value) {
    return static_cast<double>(steadyNs(value)) / 1e6;
}

std::string traceFileTimestampSuffix() {
    const std::time_t now = std::time(nullptr);
    std::tm local_tm {};
    localtime_r(&now, &local_tm);
    std::ostringstream out;
    out << std::put_time(&local_tm, "%Y%m%d_%H%M%S");
    return out.str();
}

void ensureCsvHeaderWithRotation(
    const fs::path& path,
    const std::string& expected_header,
    const std::string& backup_stem) {
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);

    bool header_matches = false;
    if (fs::exists(path, ec) && fs::file_size(path, ec) > 0) {
        std::ifstream input(path);
        std::string first_line;
        if (input.is_open() && std::getline(input, first_line)) {
            header_matches =
                trimWhitespace(first_line) == trimWhitespace(expected_header);
        }
        if (!header_matches) {
            const fs::path backup_path =
                path.parent_path() /
                (backup_stem + "_" + traceFileTimestampSuffix() + path.extension().string());
            std::error_code rename_ec;
            fs::rename(path, backup_path, rename_ec);
            if (rename_ec) {
                std::ifstream source(path, std::ios::binary);
                std::ofstream backup(backup_path, std::ios::binary | std::ios::trunc);
                if (source.is_open() && backup.is_open()) {
                    backup << source.rdbuf();
                }
                std::error_code remove_ec;
                fs::remove(path, remove_ec);
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

std::string autoAvoidCycleTraceHeader() {
    return
        "control_cycle_id,cycle_start_ms,cycle_end_ms,cycle_duration_ms,"
        "snapshot_valid,lidar_valid,imu_valid,auto_avoid_state_before,auto_avoid_state_after,"
        "command_valid,command_mode,command_speed,command_steering,"
        "whether_sent_start,whether_sent_angle,whether_sent_speed,whether_sent_stop,result,"
        "reason_code,fallback_reason,direction,snapshot_fresh,"
        "control_snapshot_seq,control_snapshot_stamp_ms,lidar_snapshot_seq,imu_snapshot_seq,"
        "lidar_snapshot_age_ms,imu_snapshot_age_ms,control_snapshot_consistent,"
        "control_snapshot_source,control_snapshot_fresh,front_nearest_m,front_angle_deg,"
        "front_support_points,selected_front_cluster_id,selected_front_cluster_score,"
        "selected_front_cluster_wall_like,selected_front_cluster_points,"
        "selected_front_cluster_span_deg,selected_front_cluster_median_range,"
        "selected_front_cluster_nearest_range,selected_front_cluster_is_discrete_primary,"
        "selected_front_cluster_is_wall_like,wall_like_cluster_suppressed,"
        "front_target_role,raw_zone_from_discrete_target,"
        "wall_like_suppressed_from_zone,front_target_selection_reason,"
        "raw_zone_source,raw_zone,resolved_zone,"
        "spike_suppressed,zone_stabilized,zone_ambiguous,resolved_zone_override_active,"
        "resolved_zone_override_reason,committed_direction_override_active,"
        "committed_direction_override_reason,turning_to_clearance_candidate,"
        "turning_to_clearance_confirm_ticks,turning_to_clearance_reason,"
        "center_turn_decision_mode,center_turn_bias_removed,center_turn_decision_reason,"
        "active_avoidance_commit_present,sector_buffer_active_continue,"
        "sector_buffer_continue_active,sector_buffer_observe_only,"
        "sector_buffer_redirect_to_straight,sector_buffer_redirect_reason,"
        "straight_drive_due_to_sector_buffer,sector_buffer_interrupt_reason,"
        "boundary_stop,emergency_stop,replan_triggered,active_stage_priority_mode,"
        "replan_override_active,replan_override_reason,active_stage_protection_active,"
        "active_stage_protection_reason,return_heading_protected,"
        "return_heading_protect_ticks_remaining,lateral_balance_active,"
        "lateral_balance_correction_deg,wall_constraint_active,wall_constraint_side,"
        "wall_constraint_correction_deg,boundary_recovery_active,boundary_recovery_side,"
        "boundary_recovery_level,boundary_recovery_correction_deg,"
        "boundary_recovery_limited_by_tail,boundary_override_active,"
        "boundary_override_reason,boundary_override_reduced_main_steering,"
        "boundary_override_reduced_by_deg,boundary_risk_left,boundary_risk_right,"
        "boundary_risk_delta,boundary_recovery_and_path_aligned,"
        "boundary_recovery_and_path_conflict,main_steering_deg,"
        "main_steering_source,boundary_override_applied,boundary_override_delta_deg,"
        "boundary_recovery_applied,boundary_recovery_delta_deg,"
        "smoothed_steering_deg,guarded_steering_deg,final_encoder_command,"
        "steering_direction_consistent,steering_direction_conflict_reason,"
        "path_reference_valid,"
        "reference_yaw_deg,reference_side_balance,reference_left_distance_m,"
        "reference_right_distance_m,path_reference_captured_ms,path_reference_captured_stage,"
        "path_reference_captured_this_cycle,path_reference_clear_reason,"
        "return_to_path_active,return_to_path_phase,return_to_path_fast_recenter_active,"
        "return_to_path_settling_active,return_to_path_can_settle,"
        "return_to_path_blocked_reason,yaw_recovery_correction_deg,"
        "yaw_recovery_dynamic_gain,yaw_recovery_retained_by_path,"
        "yaw_recovery_final_deg,path_recovery_correction_deg,path_recovery_balance_error,"
        "path_recovery_dynamic_gain,path_recovery_fast_recenter_boost,"
        "path_recovery_final_deg,"
        "combined_return_correction_deg,combined_return_correction_limited_by_tail,"
        "return_to_path_progress_score,return_to_path_near_reference,"
        "tail_clearance_complete,tail_clearance_blocking,"
        "path_recovery_ready,path_recovery_settled,exit_to_idle_ready,used_imu_heading,"
        "used_encoder_fallback,encoder_fallback_kind,target_yaw_valid,target_yaw_deg,"
        "target_yaw_locked_ms,target_yaw_locked_by_stage,target_yaw_locked_this_cycle";
}

void appendAutoAvoidCycleTraceCsv(const std::string& line) {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex);

    ensureCsvHeaderWithRotation(
        fs::path(kAutoAvoidCycleTracePath),
        autoAvoidCycleTraceHeader(),
        "auto_avoid_cycle_trace_pre_header_sync");

    std::ofstream output(kAutoAvoidCycleTracePath, std::ios::app);
    if (!output.is_open()) {
        return;
    }

    output << line << "\n";
}

std::chrono::system_clock::time_point fileTimeToSystemClock(fs::file_time_type value) {
    const auto now_fs = fs::file_time_type::clock::now();
    const auto now_sys = std::chrono::system_clock::now();
    return now_sys + std::chrono::duration_cast<std::chrono::system_clock::duration>(value - now_fs);
}

std::string runtimeLogText(const fs::path& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        return "";
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

bool readPidFile(const fs::path& pid_path, int& pid) {
    std::ifstream input(pid_path);
    if (!input.is_open()) {
        return false;
    }

    input >> pid;
    return input.good() || input.eof();
}

std::string processCommandLine(int pid) {
    const fs::path cmdline_path = fs::path("/proc") / std::to_string(pid) / "cmdline";
    std::ifstream input(cmdline_path, std::ios::binary);
    if (!input.is_open()) {
        return "";
    }

    std::string data((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    for (char& ch : data) {
        if (ch == '\0') {
            ch = ' ';
        }
    }
    return trimWhitespace(data);
}

bool stopProcessByPidFile(const fs::path& pid_path, const std::string& needle) {
    int pid = 0;
    if (!readPidFile(pid_path, pid) || pid <= 0) {
        return false;
    }

    if (::kill(pid, 0) != 0) {
        std::error_code remove_error;
        fs::remove(pid_path, remove_error);
        return false;
    }

    if (!needle.empty()) {
        const std::string command_line = processCommandLine(pid);
        if (command_line.empty() || command_line.find(needle) == std::string::npos) {
            return false;
        }
    }

    ::kill(-pid, SIGTERM);
    for (int i = 0; i < 10; ++i) {
        if (::kill(pid, 0) != 0) {
            std::error_code remove_error;
            fs::remove(pid_path, remove_error);
            return true;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    ::kill(-pid, SIGKILL);
    std::error_code remove_error;
    fs::remove(pid_path, remove_error);
    return true;
}

ComponentProcessState componentProcessState(const fs::path& pid_path, const std::string& needle) {
    int pid = 0;
    if (!readPidFile(pid_path, pid) || pid <= 0) {
        return {};
    }

    if (::kill(pid, 0) != 0) {
        return {};
    }

    const std::string command_line = processCommandLine(pid);
    if (command_line.empty() || command_line.find(needle) == std::string::npos) {
        return {};
    }

    std::error_code time_error;
    const auto write_time = fs::last_write_time(pid_path, time_error);
    ComponentProcessState state;
    state.running = true;
    state.started_at = time_error ? std::chrono::system_clock::now() : fileTimeToSystemClock(write_time);
    return state;
}

std::string runtimeLogsDir() {
    return (detectSourceProjectRoot() / "log" / "runtime").string();
}

std::string sanitizeLogLine(const std::string& value) {
    std::string cleaned;
    cleaned.reserve(value.size());

    for (std::size_t i = 0; i < value.size(); ++i) {
        const unsigned char ch = static_cast<unsigned char>(value[i]);
        if (ch == 0x1B && i + 1 < value.size() && value[i + 1] == '[') {
            i += 2;
            while (i < value.size()) {
                const unsigned char ansi = static_cast<unsigned char>(value[i]);
                if (ansi >= 0x40 && ansi <= 0x7E) {
                    break;
                }
                ++i;
            }
            continue;
        }
        if (ch < 0x20 && ch != '\n' && ch != '\r' && ch != '\t') {
            continue;
        }
        cleaned += static_cast<char>(ch);
    }

    return cleaned;
}

std::string detectLogLevel(const std::string& line) {
    if (line.find("[ERROR]") != std::string::npos ||
        line.find("[error]") != std::string::npos ||
        line.find("启动失败") != std::string::npos) {
        return "ERROR";
    }
    if (line.find("[WARN]") != std::string::npos ||
        line.find("[warning]") != std::string::npos ||
        line.find("invalid checksum") != std::string::npos) {
        return "WARN";
    }
    return "INFO";
}

void appendRuntimeLogLine(const std::string& module, const std::string& raw_line) {
    const std::string line = trimWhitespace(sanitizeLogLine(raw_line));
    if (line.empty()) {
        return;
    }

    const std::string level = detectLogLevel(line);
    if (level == "ERROR") {
        log_error(module, line);
        return;
    }
    if (level == "WARN") {
        log_warn(module, line);
        return;
    }
    if (logger_info_enabled()) {
        log_info(module, line);
    }
}

std::string recentRuntimeLogBootstrap(const fs::path& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        return "";
    }

    std::deque<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        lines.push_back(line);
        while (lines.size() > kRuntimeBootstrapLineCount) {
            lines.pop_front();
        }
    }

    std::ostringstream out;
    std::size_t start_index = 0;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        if (lines[i].find("starting ROS 传感器栈") != std::string::npos) {
            start_index = i;
        }
    }

    for (std::size_t i = start_index; i < lines.size(); ++i) {
        const auto& item = lines[i];
        out << item << "\n";
    }
    return out.str();
}

bool jsonRawFieldTrue(const std::string& json, const std::string& key) {
    const std::string raw = trimWhitespace(extractJsonRawField(json, key));
    return raw == "true" || raw == "1";
}

std::string deviceStateJson(const std::string& code, const std::string& text) {
    std::ostringstream out;
    out << "{"
        << "\"code\":\"" << jsonEscape(code) << "\","
        << "\"text\":\"" << jsonEscape(text) << "\""
        << "}";
    return out.str();
}

std::optional<int> parseJsonIntField(const std::string& body, const std::string& key) {
    const std::string raw = trimWhitespace(extractJsonRawField(body, key));
    if (raw.empty()) {
        return std::nullopt;
    }

    try {
        std::size_t consumed = 0;
        const int value = std::stoi(raw, &consumed, 10);
        if (consumed != raw.size()) {
            return std::nullopt;
        }
        return value;
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<double> parseJsonDoubleField(const std::string& body, const std::string& key) {
    const std::string raw = trimWhitespace(extractJsonRawField(body, key));
    if (raw.empty()) {
        return std::nullopt;
    }
    try {
        std::size_t consumed = 0;
        const double value = std::stod(raw, &consumed);
        if (consumed != raw.size() || !std::isfinite(value)) {
            return std::nullopt;
        }
        return value;
    } catch (...) {
        return std::nullopt;
    }
}

double degToRad(double value_deg) {
    constexpr double kPi = 3.14159265358979323846;
    return value_deg * kPi / 180.0;
}

double radToDeg(double value_rad) {
    constexpr double kPi = 3.14159265358979323846;
    return value_rad * 180.0 / kPi;
}

double haversineDistanceMeters(
    double lat1_deg,
    double lon1_deg,
    double lat2_deg,
    double lon2_deg) {
    constexpr double kEarthRadiusMeters = 6378137.0;
    const double lat1 = degToRad(lat1_deg);
    const double lat2 = degToRad(lat2_deg);
    const double delta_lat = degToRad(lat2_deg - lat1_deg);
    const double delta_lon = degToRad(lon2_deg - lon1_deg);
    const double h =
        std::sin(delta_lat * 0.5) * std::sin(delta_lat * 0.5) +
        std::cos(lat1) * std::cos(lat2) *
            std::sin(delta_lon * 0.5) * std::sin(delta_lon * 0.5);
    return 2.0 * kEarthRadiusMeters * std::asin(std::min(1.0, std::sqrt(h)));
}

double initialBearingDeg(
    double lat1_deg,
    double lon1_deg,
    double lat2_deg,
    double lon2_deg) {
    const double lat1 = degToRad(lat1_deg);
    const double lat2 = degToRad(lat2_deg);
    const double delta_lon = degToRad(lon2_deg - lon1_deg);
    const double y = std::sin(delta_lon) * std::cos(lat2);
    const double x =
        std::cos(lat1) * std::sin(lat2) -
        std::sin(lat1) * std::cos(lat2) * std::cos(delta_lon);
    return radToDeg(std::atan2(y, x));
}

double planarBearingDeg(double x_m, double y_m) {
    return radToDeg(std::atan2(x_m, y_m));
}

double normalizeHeadingDifferenceDeg(double value_deg) {
    while (value_deg > 180.0) {
        value_deg -= 360.0;
    }
    while (value_deg < -180.0) {
        value_deg += 360.0;
    }
    return value_deg;
}

int autoWorkspaceSteeringEncoderForHeadingError(double heading_error_deg) {
    double effective_error = heading_error_deg;
    if (std::abs(effective_error) < kAutoWorkspaceHeadingDeadbandDeg) {
        effective_error = 0.0;
    }
    const double steering_deg = std::clamp(
        effective_error * kAutoWorkspaceHeadingKp,
        -kAutoWorkspaceMaxSteeringDeg,
        kAutoWorkspaceMaxSteeringDeg);
    return static_cast<int>(std::lround(
        steering_deg * static_cast<double>(kAutoWorkspaceEncoderPer10Deg) / 10.0));
}

std::pair<double, double> planarOffsetFromBodyFrameMeters(
    double heading_deg,
    double forward_x_m,
    double left_y_m) {
    const double heading_rad = degToRad(heading_deg);
    const double east_m =
        forward_x_m * std::sin(heading_rad) -
        left_y_m * std::cos(heading_rad);
    const double north_m =
        forward_x_m * std::cos(heading_rad) +
        left_y_m * std::sin(heading_rad);
    return {east_m, north_m};
}

std::pair<double, double> localPlanarOffsetMeters(
    double origin_lat_deg,
    double origin_lon_deg,
    double target_lat_deg,
    double target_lon_deg) {
    constexpr double kEarthRadiusMeters = 6378137.0;
    const double origin_lat_rad = degToRad(origin_lat_deg);
    const double x_m =
        kEarthRadiusMeters * degToRad(target_lon_deg - origin_lon_deg) * std::cos(origin_lat_rad);
    const double y_m =
        kEarthRadiusMeters * degToRad(target_lat_deg - origin_lat_deg);
    return {x_m, y_m};
}

std::pair<double, double> latLonFromLocalPlanarOffsetMeters(
    double origin_lat_deg,
    double origin_lon_deg,
    double x_m,
    double y_m) {
    constexpr double kEarthRadiusMeters = 6378137.0;
    const double origin_lat_rad = degToRad(origin_lat_deg);
    const double latitude =
        origin_lat_deg + radToDeg(y_m / kEarthRadiusMeters);
    const double longitude =
        origin_lon_deg + radToDeg(
            x_m / std::max(1e-9, kEarthRadiusMeters * std::cos(origin_lat_rad)));
    return {latitude, longitude};
}

std::string stm32DirectResultJson(
    const std::string& command,
    const std::string& frame,
    bool ok,
    const std::string& status,
    const std::string& message) {
    std::ostringstream out;
    out << "{"
        << "\"ok\":" << boolJson(ok) << ","
        << "\"command\":\"" << jsonEscape(command) << "\","
        << "\"frame\":\"" << jsonEscape(frame) << "\","
        << "\"status\":\"" << jsonEscape(status) << "\","
        << "\"message\":\"" << jsonEscape(message) << "\""
        << "}";
    return out.str();
}

std::string normalizeWorkspaceMode(std::string value) {
    value = trimWhitespace(value);
    for (char& ch : value) {
        ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }
    return value;
}

bool isValidWorkspaceMode(const std::string& mode) {
    return mode == "STOP" ||
        mode == "AUTO" ||
        mode == "AVOIDANCE" ||
        mode == "MANUAL";
}

std::string entriesToJson(const std::vector<LoggerEntry>& entries) {
    std::ostringstream out;
    out << "[";
    for (size_t i = 0; i < entries.size(); ++i) {
        const auto& entry = entries[i];
        out << "{"
            << "\"sequence\":" << entry.sequence << ","
            << "\"timestamp\":\"" << jsonEscape(entry.timestamp) << "\","
            << "\"level\":\"" << jsonEscape(entry.level) << "\","
            << "\"module\":\"" << jsonEscape(entry.module) << "\","
            << "\"message\":\"" << jsonEscape(entry.message) << "\""
            << "}";
        if (i + 1 < entries.size()) {
            out << ",";
        }
    }
    out << "]";
    return out.str();
}

bool writeSseEvent(httplib::DataSink& sink, const std::string& event_name, const std::string& data) {
    if (sink.is_writable && !sink.is_writable()) {
        return false;
    }

    const std::string payload = "event: " + event_name + "\n" + "data: " + data + "\n\n";
    return sink.write(payload.c_str(), payload.size());
}

fs::path detectSourceProjectRoot() {
    if (const char* env_root = std::getenv("PROJECT_ROOT")) {
        const fs::path path(env_root);
        if (fs::exists(path / "source" / "web")) {
            return path;
        }
    }

    auto current = fs::current_path();
    for (int i = 0; i < 8; ++i) {
        if (fs::exists(current / "source" / "web") &&
            fs::exists(current / "obj" / "sys.hpp")) {
            return current;
        }
        if (!current.has_parent_path()) {
            break;
        }
        const auto parent = current.parent_path();
        if (parent == current) {
            break;
        }
        current = parent;
    }

    return "/home/mao/use/project";
}

std::string resolveWebRoot(const std::string& configured_root) {
    if (const char* env_web_root = std::getenv("PROJECT_WEB_ROOT")) {
        const fs::path path(env_web_root);
        if (fs::exists(path)) {
            return path.string();
        }
    }

    const std::string relative_root = configured_root.empty() ? "web" : configured_root;
    const fs::path configured_path(configured_root);
    if (configured_path.is_absolute()) {
        return configured_path.string();
    }

    try {
        const fs::path package_share =
            ament_index_cpp::get_package_share_directory("project");
        if (fs::exists(package_share / relative_root)) {
            return (package_share / relative_root).string();
        }
        if ((configured_root.empty() || configured_root == "source/web") &&
            fs::exists(package_share / "web")) {
            return (package_share / "web").string();
        }
    } catch (...) {
    }

    const auto source_root = detectSourceProjectRoot();
    if (configured_root.empty()) {
        return (source_root / "source" / "web").string();
    }
    return (source_root / configured_path).string();
}

std::string resolveStorageRoot() {
    if (const char* env_storage_root = std::getenv("PROJECT_STORAGE_ROOT")) {
        return fs::path(env_storage_root).string();
    }

    return (detectSourceProjectRoot() / "source" / "allfile").string();
}

std::string readFileText(const fs::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) {
        return {};
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

std::string mountedUrlFor(
    const fs::path& root,
    const fs::path& path,
    const std::string& mount_prefix) {
    std::error_code error;
    fs::path relative = fs::relative(path, root, error);
    if (error || relative.empty()) {
        return "";
    }

    std::string relative_text = relative.generic_string();
    if (relative_text == ".") {
        relative_text.clear();
    }
    while (!relative_text.empty() && relative_text.front() == '/') {
        relative_text.erase(relative_text.begin());
    }

    if (relative_text.empty()) {
        return mount_prefix;
    }
    return mount_prefix + "/" + relative_text;
}

fs::path metadataPathForPhoto(const fs::path& photo_path) {
    fs::path metadata_path = photo_path;
    metadata_path.replace_extension(".json");
    return metadata_path;
}

std::pair<std::size_t, std::size_t> jsonFieldValueRange(
    const std::string& json,
    const std::string& key);
std::string extractJsonRawField(const std::string& json, const std::string& key);

std::string extractJsonStringField(const std::string& json, const std::string& key) {
    const auto [begin, end] = jsonFieldValueRange(json, key);
    if (begin == std::string::npos ||
        end == std::string::npos ||
        end <= begin + 1 ||
        json[begin] != '"') {
        return "";
    }

    std::string value;
    bool escaped = false;
    for (std::size_t i = begin + 1; i < end; ++i) {
        const char ch = json[i];
        if (escaped) {
            switch (ch) {
                case 'n': value.push_back('\n'); break;
                case 'r': value.push_back('\r'); break;
                case 't': value.push_back('\t'); break;
                default: value.push_back(ch); break;
            }
            escaped = false;
            continue;
        }
        if (ch == '\\') {
            escaped = true;
            continue;
        }
        if (ch == '"') {
            break;
        }
        value.push_back(ch);
    }
    return value;
}

bool replaceJsonStringField(std::string& json, const std::string& key, const std::string& value) {
    const std::string marker = "\"" + key + "\":";
    const auto begin = json.find(marker);
    if (begin == std::string::npos) {
        return false;
    }

    std::size_t field_begin = begin + marker.size();
    while (field_begin < json.size() && std::isspace(static_cast<unsigned char>(json[field_begin]))) {
        ++field_begin;
    }
    if (field_begin >= json.size() || json[field_begin] != '"') {
        return false;
    }

    ++field_begin;
    std::size_t field_end = field_begin;
    bool escaped = false;
    for (; field_end < json.size(); ++field_end) {
        const char ch = json[field_end];
        if (escaped) {
            escaped = false;
            continue;
        }
        if (ch == '\\') {
            escaped = true;
            continue;
        }
        if (ch == '"') {
            break;
        }
    }

    if (field_end >= json.size()) {
        return false;
    }

    json.replace(field_begin, field_end - field_begin, jsonEscape(value));
    return true;
}

std::pair<std::size_t, std::size_t> jsonFieldValueRange(
    const std::string& json,
    const std::string& key) {
    const std::string marker = "\"" + key + "\":";
    const auto begin = json.find(marker);
    if (begin == std::string::npos) {
        return {std::string::npos, std::string::npos};
    }

    std::size_t value_begin = begin + marker.size();
    while (value_begin < json.size() && std::isspace(static_cast<unsigned char>(json[value_begin]))) {
        ++value_begin;
    }
    if (value_begin >= json.size()) {
        return {std::string::npos, std::string::npos};
    }

    std::size_t value_end = value_begin;
    const char first = json[value_begin];
    if (first == '"') {
        ++value_end;
        bool escaped = false;
        for (; value_end < json.size(); ++value_end) {
            const char ch = json[value_end];
            if (escaped) {
                escaped = false;
                continue;
            }
            if (ch == '\\') {
                escaped = true;
                continue;
            }
            if (ch == '"') {
                ++value_end;
                break;
            }
        }
        return {value_begin, value_end};
    }

    if (first == '[' || first == '{') {
        const char open = first;
        const char close = first == '[' ? ']' : '}';
        int depth = 0;
        bool in_string = false;
        bool escaped = false;
        for (; value_end < json.size(); ++value_end) {
            const char ch = json[value_end];
            if (in_string) {
                if (escaped) {
                    escaped = false;
                    continue;
                }
                if (ch == '\\') {
                    escaped = true;
                    continue;
                }
                if (ch == '"') {
                    in_string = false;
                }
                continue;
            }
            if (ch == '"') {
                in_string = true;
                continue;
            }
            if (ch == open) {
                ++depth;
            } else if (ch == close) {
                --depth;
                if (depth == 0) {
                    ++value_end;
                    break;
                }
            }
        }
        return {value_begin, value_end};
    }

    for (; value_end < json.size(); ++value_end) {
        const char ch = json[value_end];
        if (ch == ',' || ch == '}') {
            break;
        }
    }
    return {value_begin, value_end};
}

std::string extractJsonRawField(const std::string& json, const std::string& key) {
    const auto [begin, end] = jsonFieldValueRange(json, key);
    if (begin == std::string::npos || end == std::string::npos || end < begin) {
        return "";
    }
    return json.substr(begin, end - begin);
}

bool replaceJsonRawField(std::string& json, const std::string& key, const std::string& raw_value) {
    const auto [begin, end] = jsonFieldValueRange(json, key);
    if (begin == std::string::npos || end == std::string::npos || end < begin) {
        return false;
    }
    json.replace(begin, end - begin, raw_value);
    return true;
}

struct RouteCoordinate {
    double latitude = 0.0;
    double longitude = 0.0;
};

struct AutoWorkspaceRoutePlan {
    bool ok = false;
    std::string provider;
    std::string planner = "backend_road_route";
    std::string message;
    double distance_m = 0.0;
    double duration_s = 0.0;
    std::vector<RouteCoordinate> path;
};

std::string firstJsonObjectFromArray(const std::string& raw_array) {
    const auto begin = raw_array.find('{');
    if (begin == std::string::npos) {
        return "";
    }

    int depth = 0;
    bool in_string = false;
    bool escaped = false;
    for (std::size_t index = begin; index < raw_array.size(); ++index) {
        const char ch = raw_array[index];
        if (in_string) {
            if (escaped) {
                escaped = false;
                continue;
            }
            if (ch == '\\') {
                escaped = true;
                continue;
            }
            if (ch == '"') {
                in_string = false;
            }
            continue;
        }

        if (ch == '"') {
            in_string = true;
            continue;
        }
        if (ch == '{') {
            ++depth;
            continue;
        }
        if (ch == '}') {
            --depth;
            if (depth == 0) {
                return raw_array.substr(begin, index - begin + 1);
            }
        }
    }
    return "";
}

std::vector<RouteCoordinate> parseLonLatCoordinateArray(const std::string& raw_coordinates) {
    std::vector<double> values;
    const char* cursor = raw_coordinates.c_str();
    char* end = nullptr;
    while (*cursor != '\0') {
        while (*cursor != '\0' &&
               !std::isdigit(static_cast<unsigned char>(*cursor)) &&
               *cursor != '-' &&
               *cursor != '+') {
            ++cursor;
        }
        if (*cursor == '\0') {
            break;
        }

        const double value = std::strtod(cursor, &end);
        if (end == cursor) {
            ++cursor;
            continue;
        }
        if (std::isfinite(value)) {
            values.push_back(value);
        }
        cursor = end;
    }

    std::vector<RouteCoordinate> coordinates;
    coordinates.reserve(values.size() / 2);
    for (std::size_t index = 0; index + 1 < values.size(); index += 2) {
        RouteCoordinate point;
        point.longitude = values[index];
        point.latitude = values[index + 1];
        coordinates.push_back(point);
    }
    return coordinates;
}

std::string normalizeRouteProvider(std::string value) {
    value = trimWhitespace(value);
    for (char& ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    if (value.empty()) {
        return "auto";
    }
    if (value == "auto" || value == "osrm") {
        return value;
    }
    return "auto";
}

AutoWorkspaceRoutePlan requestOsrmRoute(
    double origin_latitude,
    double origin_longitude,
    double destination_latitude,
    double destination_longitude) {
    AutoWorkspaceRoutePlan plan;
    plan.provider = "osrm";

    const std::string base_url = trimWhitespace(
        envStringOrDefault("AUTO_WORKSPACE_OSRM_BASE_URL", kAutoWorkspaceDefaultOsrmBaseUrl));
    httplib::Client client(base_url);
    client.set_connection_timeout(std::chrono::seconds(4));
    client.set_read_timeout(std::chrono::seconds(6));
    client.set_write_timeout(std::chrono::seconds(4));
    client.set_follow_location(true);
    client.set_default_headers({
        {"User-Agent", "project-auto-workspace/1.0"},
        {"Accept", "application/json"},
    });

    std::ostringstream path;
    path << "/route/v1/driving/"
         << std::fixed << std::setprecision(7)
         << origin_longitude << "," << origin_latitude << ";"
         << destination_longitude << "," << destination_latitude
         << "?overview=full&geometries=geojson&steps=false&alternatives=false&continue_straight=true";

    const auto response = client.Get(path.str().c_str());
    if (!response) {
        plan.message = "道路路线服务无响应";
        return plan;
    }
    if (response->status != 200) {
        plan.message = "道路路线服务返回 HTTP " + std::to_string(response->status);
        return plan;
    }

    const std::string code = extractJsonStringField(response->body, "code");
    if (code != "Ok") {
        const std::string api_message = extractJsonStringField(response->body, "message");
        plan.message =
            api_message.empty() ?
                ("道路路线服务返回状态 " + (code.empty() ? std::string("unknown") : code)) :
                api_message;
        return plan;
    }

    const std::string routes_raw = extractJsonRawField(response->body, "routes");
    const std::string route_raw = firstJsonObjectFromArray(routes_raw);
    if (route_raw.empty()) {
        plan.message = "道路路线响应里没有可用路线";
        return plan;
    }

    const auto route_distance = parseJsonDoubleField(route_raw, "distance");
    const auto route_duration = parseJsonDoubleField(route_raw, "duration");
    const std::string geometry_raw = extractJsonRawField(route_raw, "geometry");
    const std::string coordinates_raw = extractJsonRawField(geometry_raw, "coordinates");
    const auto coordinates = parseLonLatCoordinateArray(coordinates_raw);
    if (coordinates.size() < 2) {
        plan.message = "道路路线几何数据为空";
        return plan;
    }

    plan.ok = true;
    plan.distance_m = route_distance.value_or(0.0);
    plan.duration_s = route_duration.value_or(0.0);
    plan.path = coordinates;
    plan.message = "道路路线规划成功";
    return plan;
}

AutoWorkspaceRoutePlan requestAutoWorkspaceRoadRoute(
    double origin_latitude,
    double origin_longitude,
    double destination_latitude,
    double destination_longitude) {
    const std::string provider = normalizeRouteProvider(
        envStringOrDefault("AUTO_WORKSPACE_ROUTE_PROVIDER", "auto"));
    if (provider == "osrm" || provider == "auto") {
        auto plan = requestOsrmRoute(
            origin_latitude,
            origin_longitude,
            destination_latitude,
            destination_longitude);
        if (plan.ok) {
            return plan;
        }
        if (provider == "osrm") {
            return plan;
        }
    }

    AutoWorkspaceRoutePlan failed;
    failed.provider = provider == "auto" ? "osrm" : provider;
    failed.message = "未配置可用的道路路线服务";
    return failed;
}

template <typename PointT>
double planarDistanceMeters(const PointT& lhs, const PointT& rhs) {
    return std::hypot(lhs.x_m - rhs.x_m, lhs.y_m - rhs.y_m);
}

template <typename PointT>
std::size_t nearestPathPointIndex(
    const std::vector<PointT>& path,
    double current_x_m,
    double current_y_m) {
    if (path.empty()) {
        return 0;
    }

    std::size_t best_index = 0;
    double best_distance_m = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < path.size(); ++index) {
        const double distance_m =
            std::hypot(path[index].x_m - current_x_m, path[index].y_m - current_y_m);
        if (distance_m < best_distance_m) {
            best_distance_m = distance_m;
            best_index = index;
        }
    }
    return best_index;
}

template <typename PointT>
std::size_t lookaheadPathPointIndex(
    const std::vector<PointT>& path,
    std::size_t nearest_index,
    double current_x_m,
    double current_y_m,
    double lookahead_distance_m) {
    if (path.empty()) {
        return 0;
    }

    nearest_index = std::min(nearest_index, path.size() - 1);
    double accumulated_distance_m =
        std::hypot(path[nearest_index].x_m - current_x_m, path[nearest_index].y_m - current_y_m);
    std::size_t target_index = nearest_index;
    for (std::size_t index = nearest_index + 1; index < path.size(); ++index) {
        accumulated_distance_m += planarDistanceMeters(path[index - 1], path[index]);
        target_index = index;
        if (accumulated_distance_m >= lookahead_distance_m) {
            break;
        }
    }
    return target_index;
}

template <typename PointT>
double remainingPathDistanceMeters(
    const std::vector<PointT>& path,
    std::size_t nearest_index,
    double current_x_m,
    double current_y_m) {
    if (path.empty()) {
        return 0.0;
    }

    nearest_index = std::min(nearest_index, path.size() - 1);
    double remaining_distance_m =
        std::hypot(path[nearest_index].x_m - current_x_m, path[nearest_index].y_m - current_y_m);
    for (std::size_t index = nearest_index + 1; index < path.size(); ++index) {
        remaining_distance_m += planarDistanceMeters(path[index - 1], path[index]);
    }
    return remaining_distance_m;
}

std::string mergeLivePayloadWithSavedPhoto(
    std::string live_payload,
    const std::string& saved_payload) {
    const std::string saved_photo_url = extractJsonStringField(saved_payload, "last_photo_url");
    if (saved_photo_url.empty()) {
        return live_payload;
    }

    replaceJsonStringField(
        live_payload,
        "last_photo_path",
        extractJsonStringField(saved_payload, "last_photo_path"));
    replaceJsonStringField(live_payload, "last_photo_url", saved_photo_url);
    replaceJsonStringField(
        live_payload,
        "last_photo_time",
        extractJsonStringField(saved_payload, "last_photo_time"));
    replaceJsonStringField(
        live_payload,
        "last_detection_time",
        extractJsonStringField(saved_payload, "last_detection_time"));
    replaceJsonRawField(live_payload, "detected", extractJsonRawField(saved_payload, "detected"));
    replaceJsonRawField(live_payload, "detections_count", extractJsonRawField(saved_payload, "detections_count"));
    replaceJsonStringField(
        live_payload,
        "class_name",
        extractJsonStringField(saved_payload, "class_name"));
    replaceJsonRawField(live_payload, "confidence", extractJsonRawField(saved_payload, "confidence"));
    replaceJsonRawField(live_payload, "bbox_x", extractJsonRawField(saved_payload, "bbox_x"));
    replaceJsonRawField(live_payload, "bbox_y", extractJsonRawField(saved_payload, "bbox_y"));
    replaceJsonRawField(live_payload, "bbox_w", extractJsonRawField(saved_payload, "bbox_w"));
    replaceJsonRawField(live_payload, "bbox_h", extractJsonRawField(saved_payload, "bbox_h"));
    replaceJsonRawField(live_payload, "detections", extractJsonRawField(saved_payload, "detections"));

    return live_payload;
}

std::string inferPhotoTime(const fs::path& photo_path) {
    const std::string stem = photo_path.stem().string();
    if (stem.size() < 15) {
        return "";
    }

    const auto digits = [&](size_t begin, size_t count) {
        for (size_t i = begin; i < begin + count; ++i) {
            if (i >= stem.size() || !std::isdigit(static_cast<unsigned char>(stem[i]))) {
                return false;
            }
        }
        return true;
    };

    if (!digits(0, 8) || stem[8] != '_' || !digits(9, 6)) {
        return "";
    }

    return stem.substr(0, 4) + "-" + stem.substr(4, 2) + "-" + stem.substr(6, 2) +
           "T" + stem.substr(9, 2) + ":" + stem.substr(11, 2) + ":" + stem.substr(13, 2);
}

std::string inferPhotoClassName(const fs::path& photo_path) {
    const std::string stem = photo_path.stem().string();
    if (stem.size() <= 16) {
        return "";
    }
    return stem.substr(16);
}

std::optional<fs::path> latestSavedPhotoPath(const fs::path& photos_dir) {
    std::error_code directory_error;
    if (!fs::exists(photos_dir, directory_error) || directory_error) {
        return std::nullopt;
    }

    std::optional<fs::path> latest_path;
    fs::file_time_type latest_time{};
    for (const auto& entry : fs::directory_iterator(photos_dir, directory_error)) {
        if (directory_error) {
            return std::nullopt;
        }
        if (!entry.is_regular_file()) {
            continue;
        }
        const fs::path path = entry.path();
        if (path.extension() != ".jpg" && path.extension() != ".jpeg") {
            continue;
        }
        std::error_code time_error;
        const auto write_time = fs::last_write_time(path, time_error);
        if (time_error) {
            continue;
        }
        if (!latest_path.has_value() || write_time > latest_time) {
            latest_path = path;
            latest_time = write_time;
        }
    }

    return latest_path;
}

}  // namespace

LogDashboardServer::LogDashboardServer(std::string web_root)
    : web_root_(resolveWebRoot(web_root)),
      storage_root_(resolveStorageRoot()) {
    try {
        fs::create_directories(fs::path(storage_root_) / "photos");
    } catch (const std::exception& ex) {
        log_warn(kLogModule, "failed to prepare allfile storage: " + std::string(ex.what()));
    }

    startStm32Bridge();
    startRosBridge();
    startRuntimeLogBridge();
    configureRoutes();

    if (!server_.set_mount_point(kAllfileMountPrefix, storage_root_)) {
        log_warn(kLogModule, "failed to mount " + std::string(kAllfileMountPrefix) + " -> " + storage_root_);
    }
}

LogDashboardServer::~LogDashboardServer() {
    stop();
}

bool LogDashboardServer::listen(const std::string& host, int port) {
    log_info(kLogModule, "dashboard listen on http://" + host + ":" + std::to_string(port));
    return server_.listen(host, port);
}

void LogDashboardServer::stop() {
    server_.stop();
    stopAutoWorkspaceControl();
    stopAutoAvoidControl();
    stopRuntimeLogBridge();
    stopRosBridge();
    stopStm32Bridge();
}

bool LogDashboardServer::stopModeSwitchReady(
    const std::string& target_mode,
    std::string& reason) const {
    const fs::path runtime_dir = fs::path(runtimeLogsDir());
    const ComponentProcessState stack_process =
        componentProcessState(runtime_dir / "pids" / "project_stack.pid", kStackProcessNeedle);
    if (!stack_process.running) {
        reason = "停止模式下传感器栈未启动，无法切换工作区";
        return false;
    }

    const auto now_steady = std::chrono::steady_clock::now();

    DepthRuntimeState depth_state;
    LidarRuntimeState lidar_state;
    LidarRuntimeState auto_workspace_lidar_state;
    ImuRuntimeState imu_state;
    GpsRuntimeState gps_state;
    ImuRuntimeState auto_workspace_imu_state;
    GpsRuntimeState auto_workspace_gps_state;
    std::string live_rgb_yolo_payload;
    std::chrono::steady_clock::time_point rgb_yolo_received_steady;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        depth_state = depth_state_;
        lidar_state = lidar_state_;
        auto_workspace_lidar_state = auto_workspace_lidar_state_;
        imu_state = imu_state_;
        gps_state = gps_state_;
        auto_workspace_imu_state = auto_workspace_imu_state_;
        auto_workspace_gps_state = auto_workspace_gps_state_;
        live_rgb_yolo_payload = latest_rgb_yolo_payload_;
        rgb_yolo_received_steady = latest_rgb_yolo_received_steady_;
    }

    const bool target_auto = target_mode == "AUTO";
    const auto& selected_lidar_state =
        target_auto ? auto_workspace_lidar_state : lidar_state;
    const auto& selected_imu_state =
        target_auto ? auto_workspace_imu_state : imu_state;
    const auto& selected_gps_state =
        target_auto ? auto_workspace_gps_state : gps_state;

    std::vector<std::string> pending_devices;
    const auto push_pending = [&](bool ready, const char* label) {
        if (!ready) {
            pending_devices.emplace_back(label);
        }
    };

    const bool gps_ready =
        selected_gps_state.filtered_valid &&
        selected_gps_state.status >= 0 &&
        selected_gps_state.message_count > 0 &&
        (now_steady - selected_gps_state.last_message_steady_) <= kSensorFreshWindow;
    const bool imu_ready =
        selected_imu_state.message_count > 0 &&
        (now_steady - selected_imu_state.last_message_steady_) <= kSensorFreshWindow;
    const bool lidar_ready =
        selected_lidar_state.message_count > 0 &&
        (now_steady - selected_lidar_state.last_message_steady_) <= kSensorFreshWindow;
    const bool depth_ready =
        depth_state.message_count > 0 &&
        (now_steady - depth_state.last_message_steady_) <= kSensorFreshWindow;

    const bool rgb_payload_fresh =
        !live_rgb_yolo_payload.empty() &&
        rgb_yolo_received_steady != std::chrono::steady_clock::time_point{} &&
        (now_steady - rgb_yolo_received_steady) <= kRgbYoloFreshWindow;
    const bool rgb_ready =
        rgb_payload_fresh &&
        jsonRawFieldTrue(live_rgb_yolo_payload, "online") &&
        jsonRawFieldTrue(live_rgb_yolo_payload, "camera_open") &&
        jsonRawFieldTrue(live_rgb_yolo_payload, "model_ready") &&
        extractJsonStringField(live_rgb_yolo_payload, "error").empty();

    push_pending(gps_ready, "GPS");
    push_pending(imu_ready, "IMU");
    push_pending(lidar_ready, "雷达");
    push_pending(depth_ready, "深度相机");
    push_pending(rgb_ready, "RGB+YOLO");

    if (pending_devices.empty()) {
        reason.clear();
        return true;
    }

    std::ostringstream out;
    out << "停止模式检测未完成，以下设备未就绪：";
    for (std::size_t i = 0; i < pending_devices.size(); ++i) {
        if (i > 0) {
            out << "、";
        }
        out << pending_devices[i];
    }
    reason = out.str();
    return false;
}

void LogDashboardServer::configureRoutes() {
    const auto setJson = [](httplib::Response& res, int status, const std::string& body) {
        res.status = status;
        res.set_content(body, "application/json; charset=UTF-8");
    };

    server_.Get("/", [this](const httplib::Request&, httplib::Response& res) {
        const auto path = fs::path(web_root_) / "unmanned_vehicle_dashboard.html";
        const auto html = readFileText(path);
        if (html.empty()) {
            res.status = 404;
            res.set_content("dashboard not found", "text/plain; charset=UTF-8");
            return;
        }
        res.set_content(html, "text/html; charset=UTF-8");
    });

    server_.Get("/unmanned_vehicle_dashboard.html", [this](const httplib::Request&, httplib::Response& res) {
        const auto path = fs::path(web_root_) / "unmanned_vehicle_dashboard.html";
        const auto html = readFileText(path);
        if (html.empty()) {
            res.status = 404;
            res.set_content("dashboard not found", "text/plain; charset=UTF-8");
            return;
        }
        res.set_content(html, "text/html; charset=UTF-8");
    });

    server_.Get("/api/state", [this](const httplib::Request&, httplib::Response& res) {
        res.set_header("Cache-Control", "no-store");
        res.set_content(
            std::string("{\"ok\":true,\"state\":") + stateJson() + "}",
            "application/json; charset=UTF-8");
    });

    server_.Get("/api/auto/workspace/config", [this](const httplib::Request&, httplib::Response& res) {
        const std::string tianditu_map_tk =
            trimWhitespace(envStringOrDefault("TIANDITU_MAP_TK", ""));
        const std::string route_provider =
            normalizeRouteProvider(envStringOrDefault("AUTO_WORKSPACE_ROUTE_PROVIDER", "auto"));
        const std::string effective_route_provider =
            route_provider == "auto" ? "osrm" : route_provider;
        AutoWorkspaceLocalFrameState local_frame_state;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            local_frame_state = auto_workspace_local_frame_state_;
        }

        std::ostringstream out;
        out << "{"
            << "\"ok\":true,"
            << "\"workspace\":\"AUTO\","
            << "\"map\":{"
            << "\"provider\":\"" << jsonEscape(
                effective_route_provider == "osrm" ? "osm" : "tianditu") << "\","
            << "\"configured\":" << boolJson(
                effective_route_provider == "osrm" || !tianditu_map_tk.empty()) << ","
            << "\"tk\":\"" << jsonEscape(
                effective_route_provider == "osrm" ? "" : tianditu_map_tk) << "\","
            << "\"tiles_url\":\"" << jsonEscape(
                effective_route_provider == "osrm" ?
                    "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" :
                    "") << "\","
            << "\"radius_m\":100"
            << "},"
            << "\"gps_topic\":\"/fix\","
            << "\"imu_topic\":\"/imu/data_corrected\","
            << "\"planning_mode\":\"backend_road_route\","
            << "\"route_provider\":\"" << jsonEscape(effective_route_provider) << "\","
            << "\"coordinate_frame\":{"
            << "\"mode\":\"local_xy\","
            << "\"x_axis\":\"east_m\","
            << "\"y_axis\":\"north_m\","
            << "\"origin_valid\":" << boolJson(local_frame_state.valid) << ","
            << "\"origin_latitude\":" << (local_frame_state.valid ? numberJson(local_frame_state.origin_latitude, 7) : "null") << ","
            << "\"origin_longitude\":" << (local_frame_state.valid ? numberJson(local_frame_state.origin_longitude, 7) : "null") << ","
            << "\"origin_altitude_m\":" << (local_frame_state.valid ? numberJson(local_frame_state.origin_altitude, 2) : "null")
            << "}"
            << "}";
        res.set_header("Cache-Control", "no-store");
        res.set_content(out.str(), "application/json; charset=UTF-8");
    });

    server_.Post("/api/auto/plan_preview", [this, setJson](const httplib::Request& req, httplib::Response& res) {
        const auto destination_x_m = parseJsonDoubleField(req.body, "x_m");
        const auto destination_y_m = parseJsonDoubleField(req.body, "y_m");
        if (!destination_x_m.has_value() || !destination_y_m.has_value()) {
            setJson(
                res,
                400,
                "{\"ok\":false,\"message\":\"终点 XY 参数非法\",\"status\":\"bad_request\"}");
            return;
        }

        GpsRuntimeState auto_workspace_gps_state;
        ImuRuntimeState auto_workspace_imu_state;
        AutoWorkspaceLocalFrameState local_frame_state;
        AutoWorkspaceRuntimeState auto_workspace_runtime_state;
        const auto now_steady = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            auto_workspace_gps_state = auto_workspace_gps_state_;
            auto_workspace_imu_state = auto_workspace_imu_state_;
            local_frame_state = auto_workspace_local_frame_state_;
            auto_workspace_runtime_state = auto_workspace_runtime_state_;
        }

        if (auto_workspace_runtime_state.task_running) {
            setJson(
                res,
                409,
                "{\"ok\":false,\"message\":\"自动任务运行中，请先停止任务再重新规划\",\"status\":\"auto_task_running\"}");
            return;
        }

        const bool gps_ready =
            auto_workspace_gps_state.filtered_valid &&
            auto_workspace_gps_state.status >= 0 &&
            auto_workspace_gps_state.message_count > 0 &&
            (now_steady - auto_workspace_gps_state.last_message_steady_) <= kSensorFreshWindow &&
            std::isfinite(auto_workspace_gps_state.latitude) &&
            std::isfinite(auto_workspace_gps_state.longitude);
        if (!gps_ready) {
            setJson(
                res,
                409,
                "{\"ok\":false,\"message\":\"当前 GPS 未就绪，无法生成自动工作区路径\",\"status\":\"gps_not_ready\"}");
            return;
        }
        if (!local_frame_state.valid) {
            setJson(
                res,
                409,
                "{\"ok\":false,\"message\":\"自动工作区 XY 原点尚未锁定\",\"status\":\"xy_origin_not_ready\"}");
            return;
        }

        const double origin_lat =
            auto_workspace_gps_state.base_link_valid ?
                auto_workspace_gps_state.base_link_latitude :
                auto_workspace_gps_state.latitude;
        const double origin_lon =
            auto_workspace_gps_state.base_link_valid ?
                auto_workspace_gps_state.base_link_longitude :
                auto_workspace_gps_state.longitude;
        const auto [requested_destination_lat, requested_destination_lon] =
            latLonFromLocalPlanarOffsetMeters(
            local_frame_state.origin_latitude,
            local_frame_state.origin_longitude,
            *destination_x_m,
            *destination_y_m);
        const auto [current_x_m, current_y_m] = localPlanarOffsetMeters(
            local_frame_state.origin_latitude,
            local_frame_state.origin_longitude,
            origin_lat,
            origin_lon);
        const auto route_plan = requestAutoWorkspaceRoadRoute(
            origin_lat,
            origin_lon,
            requested_destination_lat,
            requested_destination_lon);
        if (!route_plan.ok) {
            setJson(
                res,
                502,
                std::string("{\"ok\":false,\"message\":\"") +
                    jsonEscape(route_plan.message.empty() ? "道路路线规划失败" : route_plan.message) +
                    "\",\"status\":\"route_unavailable\"}");
            return;
        }

        std::vector<AutoWorkspacePlanState::PreviewPoint> preview_path;
        preview_path.reserve(route_plan.path.size() + 1);
        const auto append_preview_point =
            [&](double latitude, double longitude, bool force) {
                if (!std::isfinite(latitude) || !std::isfinite(longitude)) {
                    return;
                }
                const auto [x_m, y_m] = localPlanarOffsetMeters(
                    local_frame_state.origin_latitude,
                    local_frame_state.origin_longitude,
                    latitude,
                    longitude);
                AutoWorkspacePlanState::PreviewPoint preview_point;
                preview_point.latitude = latitude;
                preview_point.longitude = longitude;
                preview_point.x_m = x_m;
                preview_point.y_m = y_m;
                if (!preview_path.empty() && !force) {
                    const auto& last_point = preview_path.back();
                    if (std::hypot(
                            preview_point.x_m - last_point.x_m,
                            preview_point.y_m - last_point.y_m) < kAutoWorkspaceRoutePointSpacingM) {
                        return;
                    }
                }
                preview_path.push_back(preview_point);
            };

        if (!route_plan.path.empty()) {
            const auto& first_route_point = route_plan.path.front();
            if (haversineDistanceMeters(
                    origin_lat,
                    origin_lon,
                    first_route_point.latitude,
                    first_route_point.longitude) > 0.8) {
                append_preview_point(origin_lat, origin_lon, true);
            }
        }
        for (const auto& point : route_plan.path) {
            append_preview_point(point.latitude, point.longitude, false);
        }
        if (preview_path.size() == 1) {
            append_preview_point(requested_destination_lat, requested_destination_lon, true);
        }
        if (preview_path.empty()) {
            setJson(
                res,
                502,
                "{\"ok\":false,\"message\":\"道路路线规划没有返回可执行路径\",\"status\":\"route_empty\"}");
            return;
        }
        if (preview_path.size() > kAutoWorkspaceRouteMaxPoints) {
            std::vector<AutoWorkspacePlanState::PreviewPoint> reduced_path;
            reduced_path.reserve(kAutoWorkspaceRouteMaxPoints);
            reduced_path.push_back(preview_path.front());
            const std::size_t interior_points = kAutoWorkspaceRouteMaxPoints - 2;
            const std::size_t denominator = std::max<std::size_t>(1, interior_points);
            for (std::size_t index = 1; index + 1 < kAutoWorkspaceRouteMaxPoints; ++index) {
                const std::size_t source_index =
                    1 + ((preview_path.size() - 2) * index) / denominator;
                reduced_path.push_back(preview_path[std::min(source_index, preview_path.size() - 2)]);
            }
            reduced_path.push_back(preview_path.back());
            preview_path = std::move(reduced_path);
        }

        const auto& routed_destination = preview_path.back();
        double distance_m = route_plan.distance_m;
        if (!std::isfinite(distance_m) || distance_m <= 0.0) {
            distance_m = remainingPathDistanceMeters(preview_path, 0, current_x_m, current_y_m);
        }
        double bearing_deg =
            initialBearingDeg(origin_lat, origin_lon, routed_destination.latitude, routed_destination.longitude);
        if (preview_path.size() >= 2) {
            const std::size_t target_index =
                lookaheadPathPointIndex(preview_path, 0, current_x_m, current_y_m, 3.0);
            bearing_deg = initialBearingDeg(
                origin_lat,
                origin_lon,
                preview_path[target_index].latitude,
                preview_path[target_index].longitude);
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            updateAutoWorkspaceHeadingFromRouteLocked(bearing_deg);
        }

        std::ostringstream path_json;
        path_json << "[";
        for (std::size_t index = 0; index < preview_path.size(); ++index) {
            if (index > 0) {
                path_json << ",";
            }
            const auto& point = preview_path[index];
            path_json << "{"
                      << "\"latitude\":" << numberJson(point.latitude, 7) << ","
                      << "\"longitude\":" << numberJson(point.longitude, 7) << ","
                      << "\"x_m\":" << numberJson(point.x_m, 3) << ","
                      << "\"y_m\":" << numberJson(point.y_m, 3)
                      << "}";
        }
        path_json << "]";

        std::ostringstream out;
        out << "{"
            << "\"ok\":true,"
            << "\"workspace\":\"AUTO\","
            << "\"planner\":\"" << jsonEscape(route_plan.planner) << "\","
            << "\"route_provider\":\"" << jsonEscape(route_plan.provider) << "\","
            << "\"distance_m\":" << numberJson(distance_m, 2) << ","
            << "\"duration_s\":" << numberJson(route_plan.duration_s, 1) << ","
            << "\"bearing_deg\":" << numberJson(bearing_deg, 2) << ","
            << "\"coordinate_frame\":{"
            << "\"mode\":\"local_xy\","
            << "\"x_axis\":\"east_m\","
            << "\"y_axis\":\"north_m\""
            << "},"
            << "\"origin\":{"
            << "\"latitude\":" << numberJson(local_frame_state.origin_latitude, 7) << ","
            << "\"longitude\":" << numberJson(local_frame_state.origin_longitude, 7) << ","
            << "\"altitude_m\":" << numberJson(local_frame_state.origin_altitude, 2)
            << "},"
            << "\"current\":{"
            << "\"latitude\":" << numberJson(origin_lat, 7) << ","
            << "\"longitude\":" << numberJson(origin_lon, 7) << ","
            << "\"x_m\":" << numberJson(current_x_m, 3) << ","
            << "\"y_m\":" << numberJson(current_y_m, 3)
            << "},"
            << "\"destination\":{"
            << "\"latitude\":" << numberJson(routed_destination.latitude, 7) << ","
            << "\"longitude\":" << numberJson(routed_destination.longitude, 7) << ","
            << "\"x_m\":" << numberJson(routed_destination.x_m, 3) << ","
            << "\"y_m\":" << numberJson(routed_destination.y_m, 3)
            << "},"
            << "\"requested_destination\":{"
            << "\"latitude\":" << numberJson(requested_destination_lat, 7) << ","
            << "\"longitude\":" << numberJson(requested_destination_lon, 7) << ","
            << "\"x_m\":" << numberJson(*destination_x_m, 3) << ","
            << "\"y_m\":" << numberJson(*destination_y_m, 3)
            << "},"
            << "\"imu\":{"
            << "\"valid\":" << boolJson(
                auto_workspace_imu_state.has_attitude &&
                auto_workspace_imu_state.message_count > 0 &&
                (now_steady - auto_workspace_imu_state.last_message_steady_) <= kSensorFreshWindow) << ","
            << "\"yaw_deg\":" << numberJson(auto_workspace_imu_state.yaw_deg, 1)
            << "},"
            << "\"path\":" << path_json.str()
            << "}";
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            auto_workspace_plan_state_.valid = true;
            auto_workspace_plan_state_.origin_latitude = local_frame_state.origin_latitude;
            auto_workspace_plan_state_.origin_longitude = local_frame_state.origin_longitude;
            auto_workspace_plan_state_.destination_latitude = routed_destination.latitude;
            auto_workspace_plan_state_.destination_longitude = routed_destination.longitude;
            auto_workspace_plan_state_.destination_x_m = routed_destination.x_m;
            auto_workspace_plan_state_.destination_y_m = routed_destination.y_m;
            auto_workspace_plan_state_.preview_distance_m = distance_m;
            auto_workspace_plan_state_.preview_duration_s = route_plan.duration_s;
            auto_workspace_plan_state_.preview_bearing_deg = bearing_deg;
            auto_workspace_plan_state_.planner = route_plan.planner;
            auto_workspace_plan_state_.route_provider = route_plan.provider;
            auto_workspace_plan_state_.preview_path = std::move(preview_path);
            auto_workspace_runtime_state_.plan_ready = true;
            auto_workspace_runtime_state_.target_reached = false;
            auto_workspace_runtime_state_.destination_x_m = routed_destination.x_m;
            auto_workspace_runtime_state_.destination_y_m = routed_destination.y_m;
            auto_workspace_runtime_state_.remaining_distance_m = distance_m;
            auto_workspace_runtime_state_.target_bearing_deg = bearing_deg;
            auto_workspace_runtime_state_.active_path_index = 0;
            auto_workspace_runtime_state_.route_provider = route_plan.provider;
            auto_workspace_runtime_state_.phase = "planned";
            auto_workspace_runtime_state_.message = "自动道路路径已生成，等待开始任务";
        }
        res.set_header("Cache-Control", "no-store");
        res.set_content(out.str(), "application/json; charset=UTF-8");
    });

    server_.Post("/api/auto/start", [this, setJson](const httplib::Request&, httplib::Response& res) {
        bool ready = false;
        {
            std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
            ready = static_cast<bool>(to_stm_);
        }
        if (!ready) {
            setJson(
                res,
                503,
                "{\"ok\":false,\"message\":\"STM32 未初始化\",\"status\":\"failed_not_ready\"}");
            return;
        }

        std::string active_mode;
        AutoWorkspacePlanState auto_workspace_plan_state;
        AutoWorkspaceRuntimeState auto_workspace_runtime_state;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            active_mode = active_workspace_mode_;
            auto_workspace_plan_state = auto_workspace_plan_state_;
            auto_workspace_runtime_state = auto_workspace_runtime_state_;
        }
        if (active_mode != "AUTO") {
            setJson(
                res,
                409,
                "{\"ok\":false,\"message\":\"当前不在自动工作区\",\"status\":\"wrong_workspace\"}");
            return;
        }
        if (!auto_workspace_plan_state.valid) {
            setJson(
                res,
                409,
                "{\"ok\":false,\"message\":\"请先完成自动道路路径规划\",\"status\":\"plan_not_ready\"}");
            return;
        }
        if (auto_workspace_runtime_state.task_running) {
            setJson(
                res,
                409,
                "{\"ok\":false,\"message\":\"自动任务已经在运行\",\"status\":\"auto_task_running\"}");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            auto_workspace_runtime_state_.awaiting_start_ack = true;
            auto_workspace_runtime_state_.target_reached = false;
            auto_workspace_runtime_state_.phase = "awaiting_start_ack";
            auto_workspace_runtime_state_.message = "自动任务正在等待 start ACK";
        }
        log_info(kLogModule, "auto workspace start requested; awaiting start ACK");

        bool ok = false;
        {
            std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
            if (to_stm_) {
                UartTraceContext trace_context;
                trace_context.source = "auto_workspace_start";
                trace_context.thread_tag = "api_handler";
                ScopedUartTraceContext trace_scope(trace_context);
                ok = to_stm_->sendStartWaitOk();
            }
        }

        bool enter_working_mode = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            enter_working_mode =
                ok &&
                auto_workspace_runtime_state_.awaiting_start_ack &&
                active_workspace_mode_ == "AUTO" &&
                auto_workspace_plan_state_.valid;
            auto_workspace_runtime_state_.awaiting_start_ack = false;
            if (enter_working_mode) {
                auto_workspace_runtime_state_.task_running = true;
                auto_workspace_runtime_state_.phase = "tracking";
                auto_workspace_runtime_state_.message = "自动任务已启动，准备沿道路路径前进";
            } else if (ok) {
                auto_workspace_runtime_state_.message = "start ACK 已收到，但自动任务未进入运行态";
            } else {
                auto_workspace_runtime_state_.message = "自动任务 start ACK 失败，未进入运行态";
            }
        }

        if (enter_working_mode) {
            auto_avoid_stm32_drive_enabled_.store(true);
            startAutoWorkspaceControl();
            setJson(
                res,
                200,
                stm32DirectResultJson(
                    "auto_workspace_start",
                    "#C3=1!",
                    true,
                    "sent",
                    "自动任务已启动"));
            return;
        }

        auto_avoid_stm32_drive_enabled_.store(false);
        setJson(
            res,
            ok ? 409 : 500,
            stm32DirectResultJson(
                "auto_workspace_start",
                "#C3=1!",
                false,
                ok ? "start_canceled" : "failed_send",
                ok ? "start ACK 已收到，但未进入自动任务运行态" : "自动任务 start ACK 失败"));
    });

    server_.Post("/api/auto/clear_plan", [this, setJson](const httplib::Request&, httplib::Response& res) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (auto_workspace_runtime_state_.task_running) {
                setJson(
                    res,
                    409,
                    "{\"ok\":false,\"message\":\"自动任务运行中，请先停止任务再清除路径\",\"status\":\"auto_task_running\"}");
                return;
            }
            auto_workspace_plan_state_ = AutoWorkspacePlanState{};
            auto_workspace_runtime_state_.plan_ready = false;
            auto_workspace_runtime_state_.target_reached = false;
            auto_workspace_runtime_state_.destination_x_m = 0.0;
            auto_workspace_runtime_state_.destination_y_m = 0.0;
            auto_workspace_runtime_state_.remaining_distance_m = 0.0;
            auto_workspace_runtime_state_.target_bearing_deg = 0.0;
            auto_workspace_runtime_state_.heading_error_deg = 0.0;
            auto_workspace_runtime_state_.active_path_index = 0;
            auto_workspace_runtime_state_.route_provider.clear();
            auto_workspace_runtime_state_.phase = "idle";
            auto_workspace_runtime_state_.message = "自动道路路径已清除";
        }
        setJson(
            res,
            200,
            "{\"ok\":true,\"message\":\"自动道路路径已清除\",\"status\":\"cleared\"}");
    });

    server_.Post("/api/auto/stop", [this, setJson](const httplib::Request&, httplib::Response& res) {
        std::string active_mode;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            active_mode = active_workspace_mode_;
        }
        if (active_mode != "AUTO") {
            setJson(
                res,
                409,
                "{\"ok\":false,\"message\":\"当前不在自动工作区\",\"status\":\"wrong_workspace\"}");
            return;
        }

        stopAutoWorkspaceControl(false);

        bool ready = false;
        bool angle_ok = false;
        bool stop_ok = false;
        {
            std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
            ready = static_cast<bool>(to_stm_);
            if (to_stm_) {
                UartTraceContext angle_context;
                angle_context.source = "auto_workspace_stop";
                angle_context.thread_tag = "api_handler";
                angle_context.has_steering_value = true;
                angle_context.steering_value = 0;
                ScopedUartTraceContext angle_scope(angle_context);
                angle_ok = to_stm_->sendAngleWaitOk(0);

                UartTraceContext stop_context;
                stop_context.source = "auto_workspace_stop";
                stop_context.thread_tag = "api_handler";
                ScopedUartTraceContext stop_scope(stop_context);
                stop_ok = to_stm_->sendStopWaitOk();
            }
        }
        auto_avoid_stm32_drive_enabled_.store(false);
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            auto_workspace_runtime_state_.awaiting_start_ack = false;
            auto_workspace_runtime_state_.task_running = false;
            auto_workspace_runtime_state_.avoidance_active = false;
            auto_workspace_runtime_state_.active_path_index = 0;
            auto_workspace_runtime_state_.phase = "stopped";
            auto_workspace_runtime_state_.message = "自动任务已停止";
            auto_workspace_runtime_state_.command_speed_cm_s = 0;
            auto_workspace_runtime_state_.command_steering_encoder = 0;
            vehicle_command_state_.has_speed = true;
            vehicle_command_state_.speed = 0;
            vehicle_command_state_.has_angle = true;
            vehicle_command_state_.angle = 0;
        }

        if (!ready) {
            setJson(
                res,
                503,
                stm32DirectResultJson(
                    "auto_workspace_stop",
                    "#C3=0!",
                    false,
                    "failed_not_ready",
                    "STM32 未初始化"));
            return;
        }

        setJson(
            res,
            (stop_ok ? 200 : 500),
            stm32DirectResultJson(
                "auto_workspace_stop",
                "#C3=0!",
                stop_ok,
                stop_ok ? (angle_ok ? "sent" : "sent_recenter_failed") : "failed_send",
                stop_ok ? "自动任务已停止" : "自动任务停止发送失败"));
    });

    server_.Get("/api/lidar/sectors", [this](const httplib::Request& req, httplib::Response& res) {
        const fs::path runtime_dir = fs::path(runtimeLogsDir());
        const ComponentProcessState stack_process =
            componentProcessState(runtime_dir / "pids" / "project_stack.pid", kStackProcessNeedle);
        const auto now_steady = std::chrono::steady_clock::now();

        LidarRuntimeState lidar_state;
        LidarRuntimeState auto_workspace_lidar_state;
        ImuRuntimeState imu_state;
        ImuRuntimeState auto_workspace_imu_state;
        AutoAvoidRuntimeState auto_avoid_runtime_state;
        AutoAvoidRuntimeState auto_workspace_avoid_runtime_state;
        std::string active_workspace_mode;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            lidar_state = lidar_state_;
            auto_workspace_lidar_state = auto_workspace_lidar_state_;
            imu_state = imu_state_;
            auto_workspace_imu_state = auto_workspace_imu_state_;
            auto_avoid_runtime_state = auto_avoid_runtime_state_;
            auto_workspace_avoid_runtime_state = auto_workspace_avoid_runtime_state_;
            active_workspace_mode = active_workspace_mode_;
        }

        const std::string requested_workspace =
            normalizeWorkspaceMode(
                req.has_param("workspace") ? req.get_param_value("workspace") : "");
        const bool use_auto_workspace =
            requested_workspace == "AUTO" ||
            (requested_workspace != "AVOIDANCE" && active_workspace_mode == "AUTO");
        const std::string workspace_name = use_auto_workspace ? "AUTO" : "AVOIDANCE";
        const auto& selected_lidar_state =
            use_auto_workspace ? auto_workspace_lidar_state : lidar_state;
        const auto& selected_imu_state =
            use_auto_workspace ? auto_workspace_imu_state : imu_state;
        const auto& selected_avoid_runtime_state =
            use_auto_workspace ? auto_workspace_avoid_runtime_state : auto_avoid_runtime_state;

        const bool lidar_key_valid =
            stack_process.running &&
            selected_lidar_state.valid &&
            selected_lidar_state.valid_points > 0 &&
            selected_lidar_state.message_count > 0 &&
            (now_steady - selected_lidar_state.last_message_steady_) <= kSensorFreshWindow;
        const bool imu_key_valid =
            stack_process.running &&
            selected_imu_state.has_attitude &&
            selected_imu_state.message_count > 0 &&
            (now_steady - selected_imu_state.last_message_steady_) <= kSensorFreshWindow;
        const auto& filtered_front_debug = selected_avoid_runtime_state.last_decision.debug;
        const bool control_front_valid =
            stack_process.running &&
            selected_avoid_runtime_state.has_decision &&
            selected_avoid_runtime_state.lidar_valid &&
            filtered_front_debug.control_snapshot_fresh &&
            filtered_front_debug.front_nearest_valid &&
            std::isfinite(filtered_front_debug.front_nearest_m) &&
            std::isfinite(filtered_front_debug.front_angle_deg);
        const std::string front_nearest_zone =
            control_front_valid ?
                AutoAvoidInputBuilder::frontNearestZoneFromAngle(
                    filtered_front_debug.front_angle_deg,
                    selected_lidar_state.filtered_front_nearest_zone) :
                (lidar_key_valid ? selected_lidar_state.filtered_front_nearest_zone : std::string());
        const int front_valid_points =
            control_front_valid ?
                filtered_front_debug.front_support_points :
                selected_lidar_state.filtered_auto_avoid_front_sector.valid_points;
        const int front_support_points =
            control_front_valid ?
                filtered_front_debug.front_support_points :
                selected_lidar_state.filtered_auto_avoid_front_sector.support_points;
        const auto& front_target_selection =
            control_front_valid ?
                filtered_front_debug.front_target_selection :
                selected_lidar_state.front_target_selection;

        std::ostringstream out;
        out << "{"
            << "\"ok\":true,"
            << "\"workspace\":\"" << jsonEscape(workspace_name) << "\","
            << "\"imu\":{"
            << "\"valid\":" << boolJson(imu_key_valid) << ","
            << "\"roll_deg\":" << (imu_key_valid ? numberJson(selected_imu_state.roll_deg, 1) : "null") << ","
            << "\"pitch_deg\":" << (imu_key_valid ? numberJson(selected_imu_state.pitch_deg, 1) : "null") << ","
            << "\"yaw_deg\":" << (imu_key_valid ? numberJson(selected_imu_state.yaw_deg, 1) : "null")
            << "},"
            << "\"lidar\":{"
            << "\"valid\":" << boolJson(lidar_key_valid) << ","
            << "\"front_nearest_zone\":\"" << jsonEscape(front_nearest_zone) << "\","
            << "\"sectors\":{"
            << "\"negative_front\":{"
            << "\"valid\":" << boolJson(lidar_key_valid && selected_lidar_state.negative_front_sector.valid) << ","
            << "\"nearest_m\":" << (lidar_key_valid && selected_lidar_state.negative_front_sector.valid ? numberJson(selected_lidar_state.negative_front_sector.nearest_m, 2) : "null") << ","
            << "\"nearest_angle_deg\":" << (lidar_key_valid && selected_lidar_state.negative_front_sector.valid ? numberJson(selected_lidar_state.negative_front_sector.nearest_angle_deg, 1) : "null") << ","
            << "\"valid_points\":" << selected_lidar_state.negative_front_sector.valid_points << ","
            << "\"support_points\":" << selected_lidar_state.negative_front_sector.support_points
            << "},"
            << "\"front\":{"
            << "\"valid\":" << boolJson(
                    control_front_valid ||
                    (lidar_key_valid && selected_lidar_state.filtered_auto_avoid_front_sector.valid)) << ","
            << "\"nearest_m\":" << (
                    control_front_valid ?
                        numberJson(filtered_front_debug.front_nearest_m, 2) :
                        (lidar_key_valid && selected_lidar_state.filtered_auto_avoid_front_sector.valid ?
                            numberJson(selected_lidar_state.filtered_auto_avoid_front_sector.nearest_m, 2) :
                            "null")) << ","
            << "\"nearest_angle_deg\":" << (
                    control_front_valid ?
                        numberJson(filtered_front_debug.front_angle_deg, 1) :
                        (lidar_key_valid && selected_lidar_state.filtered_auto_avoid_front_sector.valid ?
                            numberJson(selected_lidar_state.filtered_auto_avoid_front_sector.nearest_angle_deg, 1) :
                            "null")) << ","
            << "\"valid_points\":" << front_valid_points << ","
            << "\"support_points\":" << front_support_points
            << "},"
            << "\"positive_front\":{"
            << "\"valid\":" << boolJson(lidar_key_valid && selected_lidar_state.positive_front_sector.valid) << ","
            << "\"nearest_m\":" << (lidar_key_valid && selected_lidar_state.positive_front_sector.valid ? numberJson(selected_lidar_state.positive_front_sector.nearest_m, 2) : "null") << ","
            << "\"nearest_angle_deg\":" << (lidar_key_valid && selected_lidar_state.positive_front_sector.valid ? numberJson(selected_lidar_state.positive_front_sector.nearest_angle_deg, 1) : "null") << ","
            << "\"valid_points\":" << selected_lidar_state.positive_front_sector.valid_points << ","
            << "\"support_points\":" << selected_lidar_state.positive_front_sector.support_points
            << "}"
            << "},"
            << "\"front_target_selection\":{"
            << "\"valid\":" << boolJson(front_target_selection.valid) << ","
            << "\"selected_front_cluster_id\":" << (
                    front_target_selection.valid ?
                        std::to_string(front_target_selection.selected_front_cluster_id) :
                        "null") << ","
            << "\"selected_front_cluster_score\":" << (
                    front_target_selection.valid ?
                        numberJson(front_target_selection.selected_front_cluster_score, 2) :
                        "null") << ","
            << "\"selected_front_cluster_wall_like\":" <<
                boolJson(front_target_selection.selected_front_cluster_wall_like) << ","
            << "\"selected_front_cluster_is_discrete_primary\":" <<
                boolJson(front_target_selection.selected_front_cluster_is_discrete_primary) << ","
            << "\"selected_front_cluster_is_wall_like\":" <<
                boolJson(front_target_selection.selected_front_cluster_is_wall_like) << ","
            << "\"selected_front_cluster_points\":" <<
                front_target_selection.selected_front_cluster_points << ","
            << "\"selected_front_cluster_span_deg\":" << (
                    front_target_selection.valid ?
                        numberJson(front_target_selection.selected_front_cluster_span_deg, 1) :
                        "null") << ","
            << "\"selected_front_cluster_median_range\":" << (
                    front_target_selection.valid ?
                        numberJson(front_target_selection.selected_front_cluster_median_range, 2) :
                        "null") << ","
            << "\"selected_front_cluster_nearest_range\":" << (
                    front_target_selection.valid ?
                        numberJson(front_target_selection.selected_front_cluster_nearest_range, 2) :
                        "null") << ","
            << "\"wall_like_cluster_suppressed\":" <<
                boolJson(front_target_selection.wall_like_cluster_suppressed) << ","
            << "\"raw_zone_from_discrete_target\":" <<
                boolJson(front_target_selection.raw_zone_from_discrete_target) << ","
            << "\"wall_like_suppressed_from_zone\":" <<
                boolJson(front_target_selection.wall_like_suppressed_from_zone) << ","
            << "\"front_target_role\":\"" <<
                jsonEscape(front_target_selection.front_target_role) << "\","
            << "\"front_target_selection_reason\":\"" <<
                jsonEscape(front_target_selection.front_target_selection_reason) << "\","
            << "\"raw_zone_source\":\"" <<
                jsonEscape(front_target_selection.raw_zone_source) << "\""
            << "}"
            << "}"
            << "}";

        res.set_header("Cache-Control", "no-store");
        res.set_content(out.str(), "application/json; charset=UTF-8");
    });

    server_.Post("/api/system/restart", [setJson](const httplib::Request&, httplib::Response& res) {
        const fs::path script_path = detectSourceProjectRoot() / "source" / "launch" / "start_project.sh";
        std::error_code exists_error;
        if (!fs::exists(script_path, exists_error) || exists_error) {
            setJson(
                res,
                500,
                "{\"ok\":false,\"message\":\"重启脚本不存在\",\"status\":\"missing_script\"}");
            return;
        }

        const std::string command =
            "setsid bash -lc '" + script_path.string() + " restart' >/dev/null 2>&1 &";
        std::thread([command]() { std::system(command.c_str()); }).detach();
        setJson(res, 202, "{\"ok\":true,\"message\":\"系统重启已触发\"}");
    });

    server_.Post("/api/system/start", [setJson](const httplib::Request&, httplib::Response& res) {
        const fs::path script_path = detectSourceProjectRoot() / "source" / "launch" / "start_project.sh";
        std::error_code exists_error;
        if (!fs::exists(script_path, exists_error) || exists_error) {
            setJson(
                res,
                500,
                "{\"ok\":false,\"message\":\"启动脚本不存在\",\"status\":\"missing_script\"}");
            return;
        }

        const std::string command =
            "setsid bash -lc '" + script_path.string() + " start' >/dev/null 2>&1 &";
        std::thread([command]() { std::system(command.c_str()); }).detach();
        setJson(res, 202, "{\"ok\":true,\"message\":\"系统启动已触发\"}");
    });

    server_.Post("/api/system/emergency_stop", [this, setJson](const httplib::Request&, httplib::Response& res) {
        bool ready = false;
        bool stm32_sent = false;
        stopAutoWorkspaceControl(false);
        stopAutoAvoidControl();
        auto_avoid_stm32_drive_enabled_.store(false);
        {
            std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
            ready = static_cast<bool>(to_stm_);
            if (to_stm_) {
                UartTraceContext trace_context;
                trace_context.source = "emergency_stop";
                trace_context.thread_tag = "api_handler";
                ScopedUartTraceContext trace_scope(trace_context);
                stm32_sent = to_stm_->sendEmergencyStop();
            }
        }
        stm32_emergency_active_.store(ready);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            active_workspace_mode_ = "STOP";
            manual_workspace_working_ = false;
            auto_workspace_runtime_state_.awaiting_start_ack = false;
            auto_workspace_runtime_state_.task_running = false;
            auto_workspace_runtime_state_.avoidance_active = false;
            auto_workspace_runtime_state_.active_path_index = 0;
            auto_workspace_runtime_state_.phase = "stopped";
            auto_workspace_runtime_state_.message = "系统急停，自动任务已停止";
            auto_workspace_runtime_state_.command_speed_cm_s = 0;
            auto_workspace_runtime_state_.command_steering_encoder = 0;
            vehicle_command_state_.has_speed = true;
            vehicle_command_state_.speed = 0;
        }

        const fs::path pid_path = fs::path(runtimeLogsDir()) / "pids" / "project_stack.pid";
        std::thread([pid_path]() {
            stopProcessByPidFile(pid_path, kStackProcessNeedle);
        }).detach();

        if (!ready) {
            setJson(
                res,
                503,
                stm32DirectResultJson("emergency_stop", "#C3=4!", false, "failed_not_ready", "STM32 未初始化"));
            return;
        }

        setJson(
            res,
            stm32_sent ? 202 : 500,
            stm32DirectResultJson(
                "emergency_stop",
                "#C3=4!",
                stm32_sent,
                stm32_sent ? "sent" : "failed_send",
                stm32_sent ? "STM32 急停已触发" : "STM32 急停发送失败"));
    });

    server_.Post("/api/system/emergency_release", [this, setJson](const httplib::Request&, httplib::Response& res) {
        bool ready = false;
        bool stm32_sent = false;
        {
            std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
            ready = static_cast<bool>(to_stm_);
            if (to_stm_) {
                UartTraceContext trace_context;
                trace_context.source = "emergency_release";
                trace_context.thread_tag = "api_handler";
                ScopedUartTraceContext trace_scope(trace_context);
                stm32_sent = to_stm_->sendEmergencyRelease();
            }
        }
        if (stm32_sent) {
            stm32_emergency_active_.store(false);
        }

        if (!ready) {
            setJson(
                res,
                503,
                stm32DirectResultJson("emergency_release", "#C3=5!", false, "failed_not_ready", "STM32 未初始化"));
            return;
        }
        setJson(
            res,
            stm32_sent ? 202 : 500,
            stm32DirectResultJson(
                "emergency_release",
                "#C3=5!",
                stm32_sent,
                stm32_sent ? "sent" : "failed_send",
                stm32_sent ? "STM32 急停已解除" : "STM32 解除急停发送失败"));
    });

    server_.Post("/api/workspace/switch", [this, setJson](const httplib::Request& req, httplib::Response& res) {
        const std::string target_mode = normalizeWorkspaceMode(extractJsonStringField(req.body, "mode"));
        if (!isValidWorkspaceMode(target_mode)) {
            setJson(
                res,
                400,
                "{\"ok\":false,\"message\":\"工作区模式非法\",\"status\":\"bad_request\"}");
            return;
        }

        std::string current_mode;
        bool manual_working = false;
        bool auto_task_running = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_mode = active_workspace_mode_;
            manual_working = manual_workspace_working_;
            auto_task_running = auto_workspace_runtime_state_.task_running;
        }

        if (current_mode == target_mode) {
            std::ostringstream out;
            out << "{"
                << "\"ok\":true,"
                << "\"mode\":\"" << jsonEscape(target_mode) << "\","
                << "\"message\":\"工作区未变化\""
                << "}";
            setJson(res, 200, out.str());
            return;
        }

        if (current_mode == "STOP" && target_mode != "STOP") {
            std::string reason;
            if (!stopModeSwitchReady(target_mode, reason)) {
                setJson(
                    res,
                    409,
                    std::string("{\"ok\":false,\"status\":\"stop_mode_not_ready\",\"message\":\"") +
                        jsonEscape(reason) + "\"}");
                return;
            }
        }

        if (current_mode == "MANUAL" && manual_working && target_mode != "MANUAL") {
            bool ready = false;
            bool stop_ok = false;
            {
                std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
                ready = static_cast<bool>(to_stm_);
                if (ready) {
                    UartTraceContext trace_context;
                    trace_context.source = "workspace_switch";
                    trace_context.thread_tag = "api_handler";
                    ScopedUartTraceContext trace_scope(trace_context);
                    stop_ok = to_stm_->sendStop();
                }
            }

            if (!ready) {
                setJson(
                    res,
                    503,
                    "{\"ok\":false,\"message\":\"切换前无法停止手动工作区，STM32 未初始化\",\"status\":\"failed_not_ready\"}");
                return;
            }

            if (!stop_ok) {
                setJson(
                    res,
                    500,
                    stm32DirectResultJson("stop", "#C3=0!", false, "failed_send", "STM32 发送失败"));
                return;
            }
        }

        if (current_mode == "AVOIDANCE" &&
            target_mode != "AVOIDANCE" &&
            (auto_avoid_control_running_.load() || auto_avoid_stm32_drive_enabled_.load())) {
            setJson(
                res,
                409,
                "{\"ok\":false,\"message\":\"避障任务运行中，请先停止任务再切换工作区\",\"status\":\"avoidance_task_running\"}");
            return;
        }
        if (current_mode == "AUTO" &&
            target_mode != "AUTO" &&
            auto_task_running) {
            setJson(
                res,
                409,
                "{\"ok\":false,\"message\":\"自动任务运行中，请先停止任务再切换工作区\",\"status\":\"auto_task_running\"}");
            return;
        }

        bool cleared_avoidance_start_ack_pending = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            active_workspace_mode_ = target_mode;
            if (current_mode == "MANUAL" && manual_working && target_mode != "MANUAL") {
                manual_workspace_working_ = false;
            }
            if (current_mode == "AVOIDANCE" && target_mode != "AVOIDANCE") {
                cleared_avoidance_start_ack_pending = avoidance_start_ack_pending_;
                avoidance_start_ack_pending_ = false;
                vehicle_command_state_.has_speed = true;
                vehicle_command_state_.speed = 0;
            }
            if (current_mode == "AUTO" && target_mode != "AUTO") {
                auto_workspace_runtime_state_.awaiting_start_ack = false;
                auto_workspace_runtime_state_.task_running = false;
                auto_workspace_runtime_state_.avoidance_active = false;
                auto_workspace_runtime_state_.active_path_index = 0;
                auto_workspace_avoid_runtime_state_.active = false;
                auto_workspace_avoid_runtime_state_.has_decision = false;
                auto_workspace_avoid_runtime_state_.last_decision = AutoAvoidController::Command{};
                auto_workspace_avoid_runtime_state_.last_apply_result = "inactive";
                if (auto_workspace_runtime_state_.phase != "arrived") {
                    auto_workspace_runtime_state_.phase = "idle";
                    auto_workspace_runtime_state_.message = "自动工作区待命";
                }
            }
        }

        if (cleared_avoidance_start_ack_pending) {
            log_info(
                kLogModule,
                std::string("workspace switched away from avoidance; cleared start ACK pending state"));
        }
        if (target_mode == "AUTO" || current_mode == "AUTO") {
            auto_workspace_avoid_controller_.reset();
            auto_workspace_avoid_control_snapshot_pool_.clear();
        }

        std::ostringstream out;
        out << "{"
            << "\"ok\":true,"
            << "\"mode\":\"" << jsonEscape(target_mode) << "\","
            << "\"message\":\"工作区已切换\""
            << "}";
        setJson(res, 200, out.str());
    });

    const auto handleStm32Direct =
        [this, setJson](
            const httplib::Request&,
            httplib::Response& res,
            const std::string& command,
            const std::string& frame,
            bool (ToStm::*fn)()) {
            bool ready = false;
            {
                std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
                ready = static_cast<bool>(to_stm_);
            }
            if (!ready) {
                setJson(
                    res,
                    503,
                    "{\"ok\":false,\"message\":\"STM32 未初始化\",\"status\":\"failed_not_ready\"}");
                return;
            }

            std::string active_mode;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                active_mode = active_workspace_mode_;
            }
            if (active_mode != "MANUAL") {
                setJson(
                    res,
                    409,
                    "{\"ok\":false,\"message\":\"当前不在手动工作区\",\"status\":\"wrong_workspace\"}");
                return;
            }

            bool ok = false;
            {
                std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
                if (to_stm_) {
                    UartTraceContext trace_context;
                    trace_context.source = "manual_api";
                    trace_context.thread_tag = "api_handler";
                    ScopedUartTraceContext trace_scope(trace_context);
                    ok = (to_stm_.get()->*fn)();
                }
            }

            if (ok) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                manual_workspace_working_ = command != "stop";
                if (command == "start") {
                    // STM32 start branch sets raw mytarget[0] to 30. Web displays real cm/s:
                    // web_speed = ceil(stm32_speed * 0.89) = 27. Neutral C2=0 is only
                    // the initial Web display value before any explicit #C2 command is sent.
                    // This only synchronizes Web display/cache state; it does not send C1/C2.
                    vehicle_command_state_.has_speed = true;
                    vehicle_command_state_.speed = stm32SpeedToWebSpeedCmPerSec(kStm32StartSpeedRaw);
                    if (!vehicle_command_state_.has_angle) {
                        vehicle_command_state_.has_angle = true;
                        vehicle_command_state_.angle = kStm32StartAngleCommand;
                    }
                } else if (command == "stop") {
                    vehicle_command_state_.has_speed = true;
                    vehicle_command_state_.speed = 0;
                }
            }
            setJson(
                res,
                ok ? 200 : 500,
                stm32DirectResultJson(
                    command,
                    frame,
                    ok,
                    ok ? "sent" : "failed_send",
                    ok ? "指令已发送" : "STM32 发送失败"));
        };

    const auto handleStm32ValueDirect =
        [this, setJson](
            const httplib::Request& req,
            httplib::Response& res,
            const std::string& command,
            const std::string& frame_prefix,
            const std::string& field_name,
            int min_value,
            int max_value,
            bool (ToStm::*fn)(int)) {
            bool ready = false;
            {
                std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
                ready = static_cast<bool>(to_stm_);
            }
            if (!ready) {
                setJson(
                    res,
                    503,
                    "{\"ok\":false,\"message\":\"STM32 未初始化\",\"status\":\"failed_not_ready\"}");
                return;
            }

            std::string active_mode;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                active_mode = active_workspace_mode_;
            }
            if (active_mode != "MANUAL") {
                setJson(
                    res,
                    409,
                    "{\"ok\":false,\"message\":\"当前不在手动工作区\",\"status\":\"wrong_workspace\"}");
                return;
            }

            const auto parsed_value = parseJsonIntField(req.body, field_name);
            if (!parsed_value.has_value()) {
                setJson(
                    res,
                    400,
                    "{\"ok\":false,\"message\":\"参数 value 非法\",\"status\":\"bad_request\"}");
                return;
            }

            const int value = *parsed_value;
            if (value < min_value || value > max_value) {
                std::ostringstream out;
                out << "{"
                    << "\"ok\":false,"
                    << "\"message\":\"参数超出范围\","
                    << "\"status\":\"out_of_range\""
                    << "}";
                setJson(res, 400, out.str());
                return;
            }

            bool ok = false;
            {
                std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
                if (to_stm_) {
                    UartTraceContext trace_context;
                    trace_context.source = "manual_api";
                    trace_context.thread_tag = "api_handler";
                    if (command == "speed") {
                        trace_context.has_speed_value = true;
                        trace_context.speed_value = value;
                    } else if (command == "angle") {
                        trace_context.has_steering_value = true;
                        trace_context.steering_value = value;
                    }
                    ScopedUartTraceContext trace_scope(trace_context);
                    ok = (to_stm_.get()->*fn)(value);
                }
            }

            if (ok) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                manual_workspace_working_ = true;
                if (command == "speed") {
                    vehicle_command_state_.has_speed = true;
                    vehicle_command_state_.speed = value;
                } else if (command == "angle") {
                    vehicle_command_state_.has_angle = true;
                    vehicle_command_state_.angle = value;
                }
            }
            const std::string frame = frame_prefix + std::to_string(value) + "!";
            setJson(
                res,
                ok ? 200 : 500,
                stm32DirectResultJson(
                    command,
                    frame,
                    ok,
                    ok ? "sent" : "failed_send",
                    ok ? "指令已发送" : "STM32 发送失败"));
        };

    const auto handleAvoidanceTaskDirect =
        [this, setJson](
            const httplib::Request&,
            httplib::Response& res,
            const std::string& command,
            const std::string& frame,
            bool (ToStm::*fn)()) {
            bool ready = false;
            {
                std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
                ready = static_cast<bool>(to_stm_);
            }
            if (!ready) {
                setJson(
                    res,
                    503,
                    "{\"ok\":false,\"message\":\"STM32 未初始化\",\"status\":\"failed_not_ready\"}");
                return false;
            }

            std::string active_mode;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                active_mode = active_workspace_mode_;
            }
            if (active_mode != "AVOIDANCE") {
                setJson(
                    res,
                    409,
                    "{\"ok\":false,\"message\":\"当前不在避障工作区\",\"status\":\"wrong_workspace\"}");
                return false;
            }

            bool ok = false;
            {
                std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
                if (to_stm_) {
                    UartTraceContext trace_context;
                    trace_context.source = command;
                    trace_context.thread_tag = "api_handler";
                    ScopedUartTraceContext trace_scope(trace_context);
                    ok = (to_stm_.get()->*fn)();
                }
            }

            setJson(
                res,
                ok ? 200 : 500,
                stm32DirectResultJson(
                    command,
                    frame,
                    ok,
                    ok ? "sent" : "failed_send",
                    ok ? "避障任务指令已发送" : "STM32 发送失败"));
            return ok;
        };

    server_.Post("/api/stm32/start", [handleStm32Direct](const httplib::Request& req, httplib::Response& res) {
        handleStm32Direct(req, res, "start", "#C3=1!", &ToStm::sendStart);
    });
    server_.Post("/api/stm32/stop", [handleStm32Direct](const httplib::Request& req, httplib::Response& res) {
        handleStm32Direct(req, res, "stop", "#C3=0!", &ToStm::sendStop);
    });
    server_.Post("/api/stm32/restart", [handleStm32Direct](const httplib::Request& req, httplib::Response& res) {
        handleStm32Direct(req, res, "restart", "#C3=2!", &ToStm::sendRestart);
    });
    server_.Post("/api/stm32/turnback", [handleStm32Direct](const httplib::Request& req, httplib::Response& res) {
        handleStm32Direct(req, res, "turnback", "#C3=3!", &ToStm::sendTurnback);
    });
    server_.Post("/api/stm32/speed", [handleStm32ValueDirect](const httplib::Request& req, httplib::Response& res) {
        handleStm32ValueDirect(req, res, "speed", "#C1=", "value", 0, 100, &ToStm::sendSpeed);
    });
    server_.Post("/api/stm32/angle", [handleStm32ValueDirect](const httplib::Request& req, httplib::Response& res) {
        handleStm32ValueDirect(req, res, "angle", "#C2=", "value", -230, 230, &ToStm::sendAngle);
    });
    server_.Post("/api/avoidance/start", [this, setJson](const httplib::Request&, httplib::Response& res) {
        bool ready = false;
        {
            std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
            ready = static_cast<bool>(to_stm_);
        }
        if (!ready) {
            setJson(
                res,
                503,
                "{\"ok\":false,\"message\":\"STM32 未初始化\",\"status\":\"failed_not_ready\"}");
            return;
        }

        std::string active_mode;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            active_mode = active_workspace_mode_;
        }
        if (active_mode != "AVOIDANCE") {
            setJson(
                res,
                409,
                "{\"ok\":false,\"message\":\"当前不在避障工作区\",\"status\":\"wrong_workspace\"}");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            avoidance_start_ack_pending_ = true;
        }
        log_info(
            kLogModule,
            "avoidance start requested; awaiting start ACK before entering working mode");

        bool ok = false;
        {
            std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
            if (to_stm_) {
                UartTraceContext trace_context;
                trace_context.source = "avoidance_start";
                trace_context.thread_tag = "api_handler";
                ScopedUartTraceContext trace_scope(trace_context);
                ok = to_stm_->sendStartWaitOk();
            }
        }

        bool enter_working_mode = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            enter_working_mode =
                ok &&
                avoidance_start_ack_pending_ &&
                active_workspace_mode_ == "AVOIDANCE";
            avoidance_start_ack_pending_ = false;
        }

        if (enter_working_mode) {
            auto_avoid_stm32_drive_enabled_.store(true);
            startAutoAvoidControl();
            log_info(
                kLogModule,
                "avoidance start ACK confirmed; entering working mode");
            setJson(
                res,
                200,
                stm32DirectResultJson(
                    "avoidance_start",
                    "#C3=1!",
                    true,
                    "sent",
                    "避障任务指令已发送"));
            return;
        }

        auto_avoid_stm32_drive_enabled_.store(false);
        if (ok) {
            log_warn(
                kLogModule,
                "avoidance start ACK confirmed after pending state cleared; staying out of working mode");
            setJson(
                res,
                409,
                stm32DirectResultJson(
                    "avoidance_start",
                    "#C3=1!",
                    false,
                    "start_canceled",
                    "start ACK 已收到，但未进入工作模式"));
        } else {
            log_warn(
                kLogModule,
                "avoidance start ACK failed; start not confirmed, staying out of working mode");
            setJson(
                res,
                500,
                stm32DirectResultJson(
                    "avoidance_start",
                    "#C3=1!",
                    false,
                    "failed_send",
                    "STM32 发送失败"));
        }
    });
    server_.Post("/api/avoidance/stop", [this, handleAvoidanceTaskDirect](const httplib::Request& req, httplib::Response& res) {
        stopAutoAvoidControl();
        const bool ok = handleAvoidanceTaskDirect(req, res, "avoidance_stop", "#C3=0!", &ToStm::sendStopWaitOk);
        if (ok) {
            auto_avoid_stm32_drive_enabled_.store(false);
            std::lock_guard<std::mutex> lock(state_mutex_);
            vehicle_command_state_.has_speed = true;
            vehicle_command_state_.speed = 0;
        }
    });

    server_.Get("/api/logs/stream", [](const httplib::Request&, httplib::Response& res) {
        std::ostringstream out;
        out << "retry: 30000\n"
            << "event: snapshot\n"
            << "data: " << entriesToJson(logger_snapshot()) << "\n\n";
        res.set_header("Cache-Control", "no-store");
        res.set_content(out.str(), "text/event-stream; charset=UTF-8");
    });

    server_.Get("/api/logs/recent", [](const httplib::Request& req, httplib::Response& res) {
        std::uint64_t after_sequence = 0;
        if (req.has_param("after")) {
            try {
                after_sequence = static_cast<std::uint64_t>(std::stoull(req.get_param_value("after")));
            } catch (...) {
                after_sequence = 0;
            }
        }

        const auto entries = after_sequence > 0
            ? logger_entries_after(after_sequence)
            : logger_snapshot();
        const auto latest_sequence = logger_latest_sequence();

        std::ostringstream out;
        out << "{"
            << "\"ok\":true,"
            << "\"latest_sequence\":" << latest_sequence << ","
            << "\"entries\":" << entriesToJson(entries)
            << "}";

        res.set_header("Cache-Control", "no-store");
        res.set_content(out.str(), "application/json; charset=UTF-8");
    });

    server_.Get("/api/logs/config", [](const httplib::Request&, httplib::Response& res) {
        std::ostringstream out;
        out << "{"
            << "\"info_enabled\":" << (logger_info_enabled() ? "true" : "false")
            << "}";
        res.set_content(out.str(), "application/json; charset=UTF-8");
    });
}

void LogDashboardServer::startRosBridge() {
    if (!rclcpp::ok()) {
        log_warn(kLogModule, "rclcpp is not initialized, rgb_yolo bridge disabled");
        return;
    }
    if (ros_node_) {
        return;
    }

    ros_node_ = std::make_shared<rclcpp::Node>("project_web_bridge");
    depth_sub_ = ros_node_->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&LogDashboardServer::onDepthImage, this, std::placeholders::_1));
    lidar_sub_ = ros_node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&LogDashboardServer::onLidarScan, this, std::placeholders::_1));
    imu_sub_ = ros_node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data_corrected",
        rclcpp::SensorDataQoS(),
        std::bind(&LogDashboardServer::onImu, this, std::placeholders::_1));
    gps_sub_ = ros_node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/fix",
        rclcpp::SensorDataQoS(),
        std::bind(&LogDashboardServer::onGpsFix, this, std::placeholders::_1));
    rgb_yolo_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
        "/rgb_yolo/result",
        rclcpp::QoS(10),
        std::bind(&LogDashboardServer::onRgbYolo, this, std::placeholders::_1));

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(ros_node_);
    spin_thread_ = std::thread([this]() {
        executor_->spin();
    });
}

void LogDashboardServer::stopRosBridge() {
    if (executor_) {
        executor_->cancel();
    }
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    if (executor_ && ros_node_) {
        executor_->remove_node(ros_node_);
    }
    depth_sub_.reset();
    lidar_sub_.reset();
    imu_sub_.reset();
    gps_sub_.reset();
    rgb_yolo_sub_.reset();
    ros_node_.reset();
    executor_.reset();
}

void LogDashboardServer::startAutoAvoidControl() {
    if (auto_avoid_control_running_.exchange(true)) {
        return;
    }

    auto_avoid_control_thread_ = std::thread(&LogDashboardServer::runAutoAvoidControlLoop, this);
    log_info(kLogModule, "auto avoidance control loop started");
}

void LogDashboardServer::stopAutoAvoidControl() {
    bool cleared_start_ack_pending = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (avoidance_start_ack_pending_) {
            avoidance_start_ack_pending_ = false;
            cleared_start_ack_pending = true;
        }
    }
    auto_avoid_controller_.reset();
    const bool was_running = auto_avoid_control_running_.exchange(false);

    if (was_running && auto_avoid_control_thread_.joinable()) {
        auto_avoid_control_thread_.join();
    }
    if (cleared_start_ack_pending) {
        log_info(kLogModule, "avoidance start ACK pending cleared while stopping/exiting");
    }
    if (was_running) {
        log_info(kLogModule, "auto avoidance control loop stopped");
    }
}

AutoAvoidController::SensorSnapshot LogDashboardServer::autoAvoidSensorSnapshot() const {
    const auto now = std::chrono::steady_clock::now();
    const auto control_snapshot =
        auto_avoid_control_snapshot_pool_.buildControlSnapshot(
            auto_avoid_input_builder_,
            now,
            kSensorFreshWindow);
    return control_snapshot.snapshot;
}

AutoAvoidController::SensorSnapshot LogDashboardServer::autoWorkspaceAvoidSensorSnapshot() const {
    const auto now = std::chrono::steady_clock::now();
    const auto control_snapshot =
        auto_workspace_avoid_control_snapshot_pool_.buildControlSnapshot(
            auto_workspace_avoid_input_builder_,
            now,
            kSensorFreshWindow);
    return control_snapshot.snapshot;
}

LogDashboardServer::AutoAvoidCommandTrace LogDashboardServer::applyAutoAvoidCommand(
    const AutoAvoidController::Command& command,
    AutoAvoidController::Command& last_command,
    bool& has_last_command,
    std::chrono::steady_clock::time_point& last_command_sent_steady,
    std::int64_t control_cycle_id) {
    AutoAvoidCommandTrace trace;
    const auto now = std::chrono::steady_clock::now();
    AutoAvoidController::Command effective_command = command;
    if (!effective_command.valid) {
        effective_command.valid = true;
        effective_command.safety_fallback = true;
        effective_command.mode = AutoAvoidController::MotionMode::Stop;
        effective_command.state = auto_avoid_controller_.currentAvoidanceStage();
        effective_command.direction = AutoAvoidController::TurnDirection::Stop;
        effective_command.speed_cm_s = 0;
        effective_command.steering_encoder = 0;
        effective_command.steering_angle_deg = 0.0;
        effective_command.reason_code =
            AutoAvoidController::DecisionReason::InvalidDecision;
        effective_command.fallback_reason =
            AutoAvoidController::FallbackReason::InvalidDecision;
        effective_command.debug.state = effective_command.state;
        effective_command.debug.direction = effective_command.direction;
        effective_command.debug.reason_code = effective_command.reason_code;
        effective_command.debug.fallback_reason = effective_command.fallback_reason;
        effective_command.debug.safety_fallback = true;
        if (effective_command.debug_text.empty()) {
            effective_command.debug_text = "invalid decision -> safety fallback";
        }
        trace.result = "invalid_command_safe_stop";
    }

    const bool mode_changed =
        has_last_command && last_command.mode != effective_command.mode;
    const bool direction_changed =
        has_last_command && last_command.direction != effective_command.direction;
    const int speed_delta =
        has_last_command ? std::abs(last_command.speed_cm_s - effective_command.speed_cm_s) : 0;
    const int steering_delta =
        has_last_command ?
            std::abs(last_command.steering_encoder - effective_command.steering_encoder) :
            0;
    const bool speed_changed =
        !has_last_command || speed_delta >= kAutoAvoidSpeedResendThreshold;
    const bool steering_changed =
        !has_last_command || steering_delta >= kAutoAvoidSteeringResendThreshold;
    const bool command_changed =
        !has_last_command ||
        mode_changed ||
        direction_changed ||
        speed_changed ||
        steering_changed;
    if (!command_changed) {
        trace.result = "skipped_no_change";
        return trace;
    }

    const bool steering_only_update =
        has_last_command &&
        !mode_changed &&
        !direction_changed &&
        !speed_changed &&
        steering_changed;
    if (steering_only_update &&
        last_command_sent_steady.time_since_epoch().count() > 0 &&
        (now - last_command_sent_steady) < kAutoAvoidCommandMinInterval) {
        trace.result = "skipped_rate_limited";
        return trace;
    }

    const std::string auto_avoid_state =
        AutoAvoidController::avoidanceStageName(auto_avoid_controller_.currentAvoidanceStage());

    log_info(
        kLogModule,
        std::string("auto avoid command: mode=") +
            AutoAvoidController::motionModeName(effective_command.mode) +
            ", direction=" + AutoAvoidController::turnDirectionName(effective_command.direction) +
            ", speed=" + std::to_string(effective_command.speed_cm_s) +
            ", angle_encoder=" + std::to_string(effective_command.steering_encoder) +
            ", reason_code=" +
            AutoAvoidController::decisionReasonName(effective_command.reason_code) +
            ", fallback=" +
            AutoAvoidController::fallbackReasonName(effective_command.fallback_reason) +
            ", reason=" + effective_command.debug_text);

    if (effective_command.mode == AutoAvoidController::MotionMode::Stop) {
        bool stop_ok = false;
        bool angle_ok = false;
        {
            std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
            if (to_stm_) {
                UartTraceContext angle_context;
                angle_context.control_cycle_id = control_cycle_id;
                angle_context.source = "auto_avoid";
                angle_context.thread_tag = "auto_avoid_control_thread";
                angle_context.auto_avoid_state = auto_avoid_state;
                angle_context.has_steering_value = true;
                angle_context.steering_value = 0;
                ScopedUartTraceContext angle_scope(angle_context);
                trace.sent_angle = true;
                angle_ok = to_stm_->sendAngleWaitOk(0);
                UartTraceContext stop_context;
                stop_context.control_cycle_id = control_cycle_id;
                stop_context.source = "auto_avoid";
                stop_context.thread_tag = "auto_avoid_control_thread";
                stop_context.auto_avoid_state = auto_avoid_state;
                ScopedUartTraceContext stop_scope(stop_context);
                trace.sent_stop = true;
                stop_ok = to_stm_->sendStopWaitOk();
            }
        }

        if (!stop_ok) {
            log_warn(kLogModule, "auto avoid stop command failed");
            trace.result = "failed_stop";
            return trace;
        }
        if (!angle_ok) {
            log_warn(kLogModule, "auto avoid stop recenter failed before stop");
        }

        auto_avoid_stm32_drive_enabled_.store(false);
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            vehicle_command_state_.has_speed = true;
            vehicle_command_state_.speed = 0;
            vehicle_command_state_.has_angle = true;
            vehicle_command_state_.angle = 0;
        }
        last_command_sent_steady = now;
        last_command = effective_command;
        has_last_command = true;
        trace.success = true;
        trace.result = angle_ok ? "ok" : "ok_stop_recenter_failed";
        return trace;
    }

    bool start_ok = true;
    bool angle_ok = true;
    bool speed_ok = true;
    {
        std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
        if (!to_stm_) {
            log_warn(kLogModule, "auto avoid command skipped: STM32 not ready");
            trace.result = "skipped_stm32_not_ready";
            return trace;
        }

        if (!auto_avoid_stm32_drive_enabled_.load()) {
            UartTraceContext start_context;
            start_context.control_cycle_id = control_cycle_id;
            start_context.source = "auto_avoid";
            start_context.thread_tag = "auto_avoid_control_thread";
            start_context.auto_avoid_state = auto_avoid_state;
            ScopedUartTraceContext start_scope(start_context);
            trace.sent_start = true;
            start_ok = to_stm_->sendStartWaitOk();
            if (start_ok) {
                auto_avoid_stm32_drive_enabled_.store(true);
            }
        }

        if (start_ok) {
            UartTraceContext angle_context;
            angle_context.control_cycle_id = control_cycle_id;
            angle_context.source = "auto_avoid";
            angle_context.thread_tag = "auto_avoid_control_thread";
            angle_context.auto_avoid_state = auto_avoid_state;
            angle_context.has_speed_value = true;
            angle_context.speed_value = effective_command.speed_cm_s;
            angle_context.has_steering_value = true;
            angle_context.steering_value = effective_command.steering_encoder;
            ScopedUartTraceContext angle_scope(angle_context);
            trace.sent_angle = true;
            angle_ok = to_stm_->sendAngleWaitOk(effective_command.steering_encoder);
            UartTraceContext speed_context = angle_context;
            ScopedUartTraceContext speed_scope(speed_context);
            trace.sent_speed = true;
            speed_ok = to_stm_->sendSpeedWaitOk(effective_command.speed_cm_s);
        }
    }

    if (!start_ok || !angle_ok || !speed_ok) {
        log_warn(kLogModule, "auto avoid drive command failed");
        trace.result =
            !start_ok ? "failed_start" :
            (!angle_ok ? "failed_angle" : "failed_speed");
        return trace;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        vehicle_command_state_.has_speed = true;
        vehicle_command_state_.speed = effective_command.speed_cm_s;
        vehicle_command_state_.has_angle = true;
        vehicle_command_state_.angle = effective_command.steering_encoder;
    }
    last_command_sent_steady = now;
    last_command = effective_command;
    has_last_command = true;
    trace.success = true;
    trace.result = "ok";
    return trace;
}

void LogDashboardServer::runAutoAvoidControlLoop() {
    using namespace std::chrono_literals;

    AutoAvoidController::Command last_command;
    bool has_last_command = false;
    std::chrono::steady_clock::time_point last_command_sent_steady{};
    auto next_tick = std::chrono::steady_clock::now();
    std::int64_t control_cycle_id = 0;

    while (auto_avoid_control_running_.load()) {
        const auto cycle_start = std::chrono::steady_clock::now();
        next_tick += kAutoAvoidControlPeriod;
        ++control_cycle_id;

        std::string active_mode;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            active_mode = active_workspace_mode_;
        }

        const std::string auto_avoid_state_before =
            AutoAvoidController::avoidanceStageName(auto_avoid_controller_.currentAvoidanceStage());
        bool snapshot_valid = false;
        bool lidar_valid = false;
        bool imu_valid = false;
        AutoAvoidController::Command command;
        AutoAvoidCommandTrace command_trace;

        if (active_mode == "AVOIDANCE") {
            auto snapshot = autoAvoidSensorSnapshot();
            lidar_valid = snapshot.lidar_valid;
            imu_valid = snapshot.imu_valid;
            snapshot_valid =
                snapshot.lidar_valid &&
                snapshot.front.valid &&
                snapshot.negative_front.valid &&
                snapshot.positive_front.valid;
            command = auto_avoid_controller_.decide(snapshot);
            command_trace = applyAutoAvoidCommand(
                command,
                last_command,
                has_last_command,
                last_command_sent_steady,
                control_cycle_id);
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                auto_avoid_runtime_state_.active = true;
                auto_avoid_runtime_state_.has_decision = true;
                auto_avoid_runtime_state_.last_control_cycle_id = control_cycle_id;
                auto_avoid_runtime_state_.snapshot_valid = snapshot_valid;
                auto_avoid_runtime_state_.lidar_valid = lidar_valid;
                auto_avoid_runtime_state_.imu_valid = imu_valid;
                auto_avoid_runtime_state_.last_decision = command;
                auto_avoid_runtime_state_.last_apply_result = command_trace.result;
            }
        } else {
            has_last_command = false;
            last_command = AutoAvoidController::Command{};
            last_command_sent_steady = std::chrono::steady_clock::time_point{};
            auto_avoid_controller_.reset();
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                auto_avoid_runtime_state_.active = false;
                auto_avoid_runtime_state_.has_decision = false;
                auto_avoid_runtime_state_.last_control_cycle_id = control_cycle_id;
                auto_avoid_runtime_state_.snapshot_valid = false;
                auto_avoid_runtime_state_.lidar_valid = false;
                auto_avoid_runtime_state_.imu_valid = false;
                auto_avoid_runtime_state_.last_decision = AutoAvoidController::Command{};
                auto_avoid_runtime_state_.last_apply_result = "inactive";
            }
        }

        const std::string auto_avoid_state_after =
            AutoAvoidController::avoidanceStageName(auto_avoid_controller_.currentAvoidanceStage());
        const auto cycle_end = std::chrono::steady_clock::now();
        std::ostringstream cycle_row;
        cycle_row
            << control_cycle_id << ","
            << csvNumber(steadyMs(cycle_start)) << ","
            << csvNumber(steadyMs(cycle_end)) << ","
            << csvNumber(
                   std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                       cycle_end - cycle_start)
                       .count()) << ","
            << csvBool(snapshot_valid) << ","
            << csvBool(lidar_valid) << ","
            << csvBool(imu_valid) << ","
            << csvField(auto_avoid_state_before) << ","
            << csvField(auto_avoid_state_after) << ","
            << csvBool(command.valid) << ","
            << csvField(command.valid ? AutoAvoidController::motionModeName(command.mode) : "") << ","
            << csvIntOrEmpty(command.speed_cm_s, command.valid) << ","
            << csvIntOrEmpty(command.steering_encoder, command.valid) << ","
            << csvBool(command_trace.sent_start) << ","
            << csvBool(command_trace.sent_angle) << ","
            << csvBool(command_trace.sent_speed) << ","
            << csvBool(command_trace.sent_stop) << ","
            << csvField(command_trace.result) << ","
            << csvField(AutoAvoidController::decisionReasonName(command.reason_code)) << ","
            << csvField(AutoAvoidController::fallbackReasonName(command.fallback_reason)) << ","
            << csvField(AutoAvoidController::turnDirectionName(command.direction)) << ","
            << csvBool(command.debug.snapshot_fresh) << ","
            << command.debug.control_snapshot_seq << ","
            << command.debug.control_snapshot_stamp_ms << ","
            << command.debug.lidar_snapshot_seq << ","
            << command.debug.imu_snapshot_seq << ","
            << csvNumber(command.debug.lidar_snapshot_age_ms, 1) << ","
            << csvNumber(command.debug.imu_snapshot_age_ms, 1) << ","
            << csvBool(command.debug.control_snapshot_consistent) << ","
            << csvField(command.debug.control_snapshot_source) << ","
            << csvBool(command.debug.control_snapshot_fresh) << ","
            << csvNumber(command.debug.front_nearest_m, 3) << ","
            << csvNumber(command.debug.front_angle_deg, 3) << ","
            << csvIntOrEmpty(command.debug.front_support_points, command.debug.front_nearest_valid) << ","
            << csvIntOrEmpty(
                   command.debug.front_target_selection.selected_front_cluster_id,
                   command.debug.front_target_selection.valid) << ","
            << csvNumber(
                   command.debug.front_target_selection.selected_front_cluster_score,
                   3) << ","
            << csvBool(
                   command.debug.front_target_selection.selected_front_cluster_wall_like) << ","
            << csvIntOrEmpty(
                   command.debug.front_target_selection.selected_front_cluster_points,
                   command.debug.front_target_selection.valid) << ","
            << csvNumber(
                   command.debug.front_target_selection.selected_front_cluster_span_deg,
                   3) << ","
            << csvNumber(
                   command.debug.front_target_selection.selected_front_cluster_median_range,
                   3) << ","
            << csvNumber(
                   command.debug.front_target_selection.selected_front_cluster_nearest_range,
                   3) << ","
            << csvBool(
                   command.debug.front_target_selection.selected_front_cluster_is_discrete_primary) << ","
            << csvBool(
                   command.debug.front_target_selection.selected_front_cluster_is_wall_like) << ","
            << csvBool(command.debug.front_target_selection.wall_like_cluster_suppressed) << ","
            << csvField(command.debug.front_target_selection.front_target_role) << ","
            << csvBool(command.debug.front_target_selection.raw_zone_from_discrete_target) << ","
            << csvBool(command.debug.front_target_selection.wall_like_suppressed_from_zone) << ","
            << csvField(command.debug.front_target_selection.front_target_selection_reason) << ","
            << csvField(command.debug.front_target_selection.raw_zone_source) << ","
            << csvField(Judgment::frontObstacleZoneName(command.debug.raw_zone)) << ","
            << csvField(Judgment::frontObstacleZoneName(command.debug.resolved_zone)) << ","
            << csvBool(command.debug.spike_suppressed) << ","
            << csvBool(command.debug.zone_stabilized) << ","
            << csvBool(command.debug.zone_ambiguous) << ","
            << csvBool(command.debug.resolved_zone_override_active) << ","
            << csvField(command.debug.resolved_zone_override_reason) << ","
            << csvBool(command.debug.committed_direction_override_active) << ","
            << csvField(command.debug.committed_direction_override_reason) << ","
            << csvBool(command.debug.turning_to_clearance_candidate) << ","
            << command.debug.turning_to_clearance_confirm_ticks << ","
            << csvField(command.debug.turning_to_clearance_reason) << ","
            << csvField(command.debug.center_turn_decision_mode) << ","
            << csvBool(command.debug.center_turn_bias_removed) << ","
            << csvField(command.debug.center_turn_decision_reason) << ","
            << csvBool(command.debug.active_avoidance_commit_present) << ","
            << csvBool(command.debug.sector_buffer_active_continue) << ","
            << csvBool(command.debug.sector_buffer_active_continue) << ","
            << csvBool(command.debug.sector_buffer_observe_only) << ","
            << csvBool(command.debug.sector_buffer_redirect_to_straight) << ","
            << csvField(command.debug.sector_buffer_redirect_reason) << ","
            << csvBool(command.debug.straight_drive_due_to_sector_buffer) << ","
            << csvField(command.debug.sector_buffer_interrupt_reason) << ","
            << csvBool(command.debug.boundary_stop) << ","
            << csvBool(command.debug.emergency_stop) << ","
            << csvBool(command.debug.replan_triggered) << ","
            << csvField(command.debug.active_stage_priority_mode) << ","
            << csvBool(command.debug.replan_override_active) << ","
            << csvField(command.debug.replan_override_reason) << ","
            << csvBool(command.debug.active_stage_protection_active) << ","
            << csvField(command.debug.active_stage_protection_reason) << ","
            << csvBool(command.debug.return_heading_protected) << ","
            << command.debug.return_heading_protect_ticks_remaining << ","
            << csvBool(command.debug.lateral_balance_active) << ","
            << csvNumber(command.debug.lateral_balance_correction_deg, 3) << ","
            << csvBool(command.debug.wall_constraint_active) << ","
            << csvField(command.debug.wall_constraint_side) << ","
            << csvNumber(command.debug.wall_constraint_correction_deg, 3) << ","
            << csvBool(command.debug.boundary_recovery_active) << ","
            << csvField(
                   AutoAvoidController::boundaryRiskSideName(
                       command.debug.boundary_recovery_side)) << ","
            << csvField(
                   AutoAvoidController::boundaryRecoveryLevelName(
                       command.debug.boundary_recovery_level)) << ","
            << csvNumber(command.debug.boundary_recovery_correction_deg, 3) << ","
            << csvBool(command.debug.boundary_recovery_limited_by_tail) << ","
            << csvBool(command.debug.boundary_override_active) << ","
            << csvField(command.debug.boundary_override_reason) << ","
            << csvBool(command.debug.boundary_override_reduced_main_steering) << ","
            << csvNumber(command.debug.boundary_override_reduced_by_deg, 3) << ","
            << csvNumber(command.debug.boundary_risk_left, 3) << ","
            << csvNumber(command.debug.boundary_risk_right, 3) << ","
            << csvNumber(command.debug.boundary_risk_delta, 3) << ","
            << csvBool(command.debug.boundary_recovery_and_path_aligned) << ","
            << csvBool(command.debug.boundary_recovery_and_path_conflict) << ","
            << csvNumber(command.debug.main_steering_deg, 3) << ","
            << csvField(command.debug.main_steering_source) << ","
            << csvBool(command.debug.boundary_override_applied) << ","
            << csvNumber(command.debug.boundary_override_delta_deg, 3) << ","
            << csvBool(command.debug.boundary_recovery_applied) << ","
            << csvNumber(command.debug.boundary_recovery_delta_deg, 3) << ","
            << csvNumber(command.debug.smoothed_steering_deg, 3) << ","
            << csvNumber(command.debug.guarded_steering_deg, 3) << ","
            << command.debug.final_encoder_command << ","
            << csvBool(command.debug.steering_direction_consistent) << ","
            << csvField(command.debug.steering_direction_conflict_reason) << ","
            << csvBool(command.debug.path_reference_valid) << ","
            << csvNumber(command.debug.reference_yaw_deg, 3) << ","
            << csvNumber(command.debug.reference_side_balance, 3) << ","
            << csvNumber(command.debug.reference_left_distance_m, 3) << ","
            << csvNumber(command.debug.reference_right_distance_m, 3) << ","
            << command.debug.path_reference_captured_ms << ","
            << csvField(AutoAvoidController::avoidanceStageName(command.debug.path_reference_captured_stage)) << ","
            << csvBool(command.debug.path_reference_captured_this_cycle) << ","
            << csvField(AutoAvoidController::pathReferenceClearReasonName(command.debug.path_reference_clear_reason)) << ","
            << csvBool(command.debug.return_to_path_active) << ","
            << csvField(command.debug.return_to_path_phase) << ","
            << csvBool(command.debug.return_to_path_fast_recenter_active) << ","
            << csvBool(command.debug.return_to_path_settling_active) << ","
            << csvBool(command.debug.return_to_path_can_settle) << ","
            << csvField(command.debug.return_to_path_blocked_reason) << ","
            << csvNumber(command.debug.yaw_recovery_correction_deg, 3) << ","
            << csvNumber(command.debug.yaw_recovery_dynamic_gain, 3) << ","
            << csvBool(command.debug.yaw_recovery_retained_by_path) << ","
            << csvNumber(command.debug.yaw_recovery_final_deg, 3) << ","
            << csvNumber(command.debug.path_recovery_correction_deg, 3) << ","
            << csvNumber(command.debug.path_recovery_balance_error, 3) << ","
            << csvNumber(command.debug.path_recovery_dynamic_gain, 3) << ","
            << csvNumber(command.debug.path_recovery_fast_recenter_boost, 3) << ","
            << csvNumber(command.debug.path_recovery_final_deg, 3) << ","
            << csvNumber(command.debug.combined_return_correction_deg, 3) << ","
            << csvBool(command.debug.combined_return_correction_limited_by_tail) << ","
            << csvNumber(command.debug.return_to_path_progress_score, 3) << ","
            << csvBool(command.debug.return_to_path_near_reference) << ","
            << csvBool(command.debug.tail_clearance_complete) << ","
            << csvBool(command.debug.tail_clearance_blocking) << ","
            << csvBool(command.debug.path_recovery_ready) << ","
            << csvBool(command.debug.path_recovery_settled) << ","
            << csvBool(command.debug.exit_to_idle_ready) << ","
            << csvBool(command.debug.used_imu_heading) << ","
            << csvBool(command.debug.used_encoder_fallback) << ","
            << csvField(AutoAvoidController::encoderFallbackKindName(command.debug.encoder_fallback_kind)) << ","
            << csvBool(command.debug.target_yaw_valid) << ","
            << csvNumber(command.debug.target_yaw_deg, 3) << ","
            << command.debug.target_yaw_locked_ms << ","
            << csvField(AutoAvoidController::avoidanceStageName(command.debug.target_yaw_locked_by_stage)) << ","
            << csvBool(command.debug.target_yaw_locked_this_cycle);
        appendAutoAvoidCycleTraceCsv(cycle_row.str());

        std::this_thread::sleep_until(next_tick);
        if (std::chrono::steady_clock::now() > next_tick + 200ms) {
            next_tick = std::chrono::steady_clock::now();
        }
    }
}

void LogDashboardServer::startAutoWorkspaceControl() {
    if (auto_workspace_control_running_.exchange(true)) {
        return;
    }

    auto_workspace_control_thread_ =
        std::thread(&LogDashboardServer::runAutoWorkspaceControlLoop, this);
    log_info(kLogModule, "auto workspace control loop started");
}

void LogDashboardServer::stopAutoWorkspaceControl(bool send_stop) {
    const bool was_running = auto_workspace_control_running_.exchange(false);
    if (was_running && auto_workspace_control_thread_.joinable()) {
        auto_workspace_control_thread_.join();
    }

    auto_workspace_avoid_controller_.reset();
    auto_workspace_avoid_control_snapshot_pool_.clear();
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        auto_workspace_runtime_state_.awaiting_start_ack = false;
        auto_workspace_runtime_state_.task_running = false;
        auto_workspace_runtime_state_.avoidance_active = false;
        auto_workspace_runtime_state_.active_path_index = 0;
        auto_workspace_runtime_state_.command_speed_cm_s = 0;
        auto_workspace_runtime_state_.command_steering_encoder = 0;
        auto_workspace_avoid_runtime_state_.active = false;
        auto_workspace_avoid_runtime_state_.has_decision = false;
        auto_workspace_avoid_runtime_state_.snapshot_valid = false;
        auto_workspace_avoid_runtime_state_.lidar_valid = false;
        auto_workspace_avoid_runtime_state_.imu_valid = false;
        auto_workspace_avoid_runtime_state_.last_decision = AutoAvoidController::Command{};
        auto_workspace_avoid_runtime_state_.last_apply_result = "stopped";
    }

    if (send_stop) {
        bool angle_ok = false;
        bool stop_ok = false;
        {
            std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
            if (to_stm_) {
                UartTraceContext angle_context;
                angle_context.source = "auto_workspace_stop";
                angle_context.thread_tag = "auto_workspace_control_thread";
                angle_context.has_steering_value = true;
                angle_context.steering_value = 0;
                ScopedUartTraceContext angle_scope(angle_context);
                angle_ok = to_stm_->sendAngleWaitOk(0);

                UartTraceContext stop_context;
                stop_context.source = "auto_workspace_stop";
                stop_context.thread_tag = "auto_workspace_control_thread";
                ScopedUartTraceContext stop_scope(stop_context);
                stop_ok = to_stm_->sendStopWaitOk();
            }
        }
        (void)angle_ok;
        (void)stop_ok;
        auto_avoid_stm32_drive_enabled_.store(false);
        std::lock_guard<std::mutex> lock(state_mutex_);
        vehicle_command_state_.has_speed = true;
        vehicle_command_state_.speed = 0;
        vehicle_command_state_.has_angle = true;
        vehicle_command_state_.angle = 0;
    }

    if (was_running) {
        log_info(kLogModule, "auto workspace control loop stopped");
    }
}

LogDashboardServer::AutoAvoidCommandTrace LogDashboardServer::applyAutoWorkspaceDriveCommand(
    bool stop_motion,
    int speed_cm_s,
    int steering_encoder,
    const std::string& phase,
    int& last_speed_cm_s,
    int& last_steering_encoder,
    bool& has_last_command,
    std::chrono::steady_clock::time_point& last_command_sent_steady,
    std::int64_t control_cycle_id) {
    AutoAvoidCommandTrace trace;
    const auto now = std::chrono::steady_clock::now();

    const bool changed =
        !has_last_command ||
        std::abs(last_speed_cm_s - speed_cm_s) >= kAutoAvoidSpeedResendThreshold ||
        std::abs(last_steering_encoder - steering_encoder) >= kAutoAvoidSteeringResendThreshold ||
        stop_motion;
    if (!changed && !stop_motion) {
        trace.result = "skipped_no_change";
        return trace;
    }

    const bool steering_only_update =
        has_last_command &&
        last_speed_cm_s == speed_cm_s &&
        last_steering_encoder != steering_encoder;
    if (!stop_motion &&
        steering_only_update &&
        last_command_sent_steady.time_since_epoch().count() > 0 &&
        (now - last_command_sent_steady) < kAutoAvoidCommandMinInterval) {
        trace.result = "skipped_rate_limited";
        return trace;
    }

    if (stop_motion) {
        bool angle_ok = false;
        bool stop_ok = false;
        {
            std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
            if (to_stm_) {
                UartTraceContext angle_context;
                angle_context.control_cycle_id = control_cycle_id;
                angle_context.source = "auto_workspace";
                angle_context.thread_tag = "auto_workspace_control_thread";
                angle_context.auto_avoid_state = phase;
                angle_context.has_steering_value = true;
                angle_context.steering_value = 0;
                ScopedUartTraceContext angle_scope(angle_context);
                trace.sent_angle = true;
                angle_ok = to_stm_->sendAngleWaitOk(0);

                UartTraceContext stop_context;
                stop_context.control_cycle_id = control_cycle_id;
                stop_context.source = "auto_workspace";
                stop_context.thread_tag = "auto_workspace_control_thread";
                stop_context.auto_avoid_state = phase;
                ScopedUartTraceContext stop_scope(stop_context);
                trace.sent_stop = true;
                stop_ok = to_stm_->sendStopWaitOk();
            }
        }

        if (!stop_ok) {
            trace.result = "failed_stop";
            return trace;
        }
        auto_avoid_stm32_drive_enabled_.store(false);
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            vehicle_command_state_.has_speed = true;
            vehicle_command_state_.speed = 0;
            vehicle_command_state_.has_angle = true;
            vehicle_command_state_.angle = 0;
        }
        last_command_sent_steady = now;
        last_speed_cm_s = 0;
        last_steering_encoder = 0;
        has_last_command = true;
        trace.success = true;
        trace.result = angle_ok ? "ok" : "ok_stop_recenter_failed";
        return trace;
    }

    bool start_ok = true;
    bool angle_ok = true;
    bool speed_ok = true;
    {
        std::lock_guard<std::mutex> stm32_lock(stm32_mutex_);
        if (!to_stm_) {
            trace.result = "skipped_stm32_not_ready";
            return trace;
        }

        if (!auto_avoid_stm32_drive_enabled_.load()) {
            UartTraceContext start_context;
            start_context.control_cycle_id = control_cycle_id;
            start_context.source = "auto_workspace";
            start_context.thread_tag = "auto_workspace_control_thread";
            start_context.auto_avoid_state = phase;
            ScopedUartTraceContext start_scope(start_context);
            trace.sent_start = true;
            start_ok = to_stm_->sendStartWaitOk();
            if (start_ok) {
                auto_avoid_stm32_drive_enabled_.store(true);
            }
        }

        if (start_ok) {
            UartTraceContext angle_context;
            angle_context.control_cycle_id = control_cycle_id;
            angle_context.source = "auto_workspace";
            angle_context.thread_tag = "auto_workspace_control_thread";
            angle_context.auto_avoid_state = phase;
            angle_context.has_speed_value = true;
            angle_context.speed_value = speed_cm_s;
            angle_context.has_steering_value = true;
            angle_context.steering_value = steering_encoder;
            ScopedUartTraceContext angle_scope(angle_context);
            trace.sent_angle = true;
            angle_ok = to_stm_->sendAngleWaitOk(steering_encoder);

            UartTraceContext speed_context = angle_context;
            ScopedUartTraceContext speed_scope(speed_context);
            trace.sent_speed = true;
            speed_ok = to_stm_->sendSpeedWaitOk(speed_cm_s);
        }
    }

    if (!start_ok || !angle_ok || !speed_ok) {
        trace.result =
            !start_ok ? "failed_start" :
            (!angle_ok ? "failed_angle" : "failed_speed");
        return trace;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        vehicle_command_state_.has_speed = true;
        vehicle_command_state_.speed = speed_cm_s;
        vehicle_command_state_.has_angle = true;
        vehicle_command_state_.angle = steering_encoder;
    }
    last_command_sent_steady = now;
    last_speed_cm_s = speed_cm_s;
    last_steering_encoder = steering_encoder;
    has_last_command = true;
    trace.success = true;
    trace.result = "ok";
    return trace;
}

void LogDashboardServer::runAutoWorkspaceControlLoop() {
    using namespace std::chrono_literals;

    int last_speed_cm_s = 0;
    int last_steering_encoder = 0;
    bool has_last_navigation_command = false;
    AutoAvoidController::Command last_avoidance_command;
    bool has_last_avoidance_command = false;
    std::chrono::steady_clock::time_point last_command_sent_steady{};
    auto next_tick = std::chrono::steady_clock::now();
    std::int64_t control_cycle_id = 0;

    while (auto_workspace_control_running_.load()) {
        next_tick += kAutoWorkspaceControlPeriod;
        ++control_cycle_id;

        std::string active_mode;
        GpsRuntimeState auto_workspace_gps_state;
        ImuRuntimeState auto_workspace_imu_state;
        AutoWorkspaceLocalFrameState local_frame_state;
        AutoWorkspacePlanState plan_state;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            active_mode = active_workspace_mode_;
            auto_workspace_gps_state = auto_workspace_gps_state_;
            auto_workspace_imu_state = auto_workspace_imu_state_;
            local_frame_state = auto_workspace_local_frame_state_;
            plan_state = auto_workspace_plan_state_;
        }

        if (active_mode != "AUTO") {
            break;
        }

        const auto now = std::chrono::steady_clock::now();
        const bool gps_valid =
            auto_workspace_gps_state.filtered_valid &&
            auto_workspace_gps_state.status >= 0 &&
            auto_workspace_gps_state.message_count > 0 &&
            (now - auto_workspace_gps_state.last_message_steady_) <= kSensorFreshWindow &&
            auto_workspace_gps_state.base_link_valid &&
            std::isfinite(auto_workspace_gps_state.base_link_latitude) &&
            std::isfinite(auto_workspace_gps_state.base_link_longitude);
        const bool imu_valid =
            auto_workspace_imu_state.has_attitude &&
            auto_workspace_imu_state.message_count > 0 &&
            (now - auto_workspace_imu_state.last_message_steady_) <= kSensorFreshWindow &&
            std::isfinite(auto_workspace_imu_state.yaw_deg);
        const bool heading_valid =
            imu_valid &&
            auto_workspace_gps_state.heading_valid &&
            std::isfinite(auto_workspace_gps_state.heading_deg);
        const bool local_frame_valid = local_frame_state.valid;
        const double current_latitude =
            gps_valid ? auto_workspace_gps_state.base_link_latitude : 0.0;
        const double current_longitude =
            gps_valid ? auto_workspace_gps_state.base_link_longitude : 0.0;
        const double current_yaw_deg =
            heading_valid ? auto_workspace_gps_state.heading_deg : 0.0;

        double current_x_m = 0.0;
        double current_y_m = 0.0;
        if (gps_valid && local_frame_valid) {
            std::tie(current_x_m, current_y_m) = localPlanarOffsetMeters(
                local_frame_state.origin_latitude,
                local_frame_state.origin_longitude,
                current_latitude,
                current_longitude);
        }

        AutoWorkspaceRuntimeState runtime_update;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            runtime_update = auto_workspace_runtime_state_;
        }
        runtime_update.plan_ready = plan_state.valid;
        runtime_update.gps_valid = gps_valid;
        runtime_update.imu_valid = imu_valid;
        runtime_update.heading_valid = heading_valid;
        runtime_update.local_frame_valid = local_frame_valid;
        runtime_update.last_control_cycle_id = control_cycle_id;
        runtime_update.current_latitude = current_latitude;
        runtime_update.current_longitude = current_longitude;
        runtime_update.current_x_m = current_x_m;
        runtime_update.current_y_m = current_y_m;
        runtime_update.current_yaw_deg = current_yaw_deg;
        runtime_update.heading_source = auto_workspace_gps_state.heading_source;
        runtime_update.destination_x_m = plan_state.destination_x_m;
        runtime_update.destination_y_m = plan_state.destination_y_m;
        runtime_update.route_provider = plan_state.route_provider;

        if (!plan_state.valid) {
            applyAutoWorkspaceDriveCommand(
                true,
                0,
                0,
                "plan_missing",
                last_speed_cm_s,
                last_steering_encoder,
                has_last_navigation_command,
                last_command_sent_steady,
                control_cycle_id);
            runtime_update.task_running = false;
            runtime_update.avoidance_active = false;
            runtime_update.phase = "plan_missing";
            runtime_update.message = "自动道路路径已失效，请重新规划";
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                auto_workspace_runtime_state_ = runtime_update;
                auto_workspace_avoid_runtime_state_.active = false;
                auto_workspace_avoid_runtime_state_.has_decision = false;
                auto_workspace_avoid_runtime_state_.last_apply_result = "plan_missing";
            }
            auto_workspace_control_running_.store(false);
            break;
        }

        if (!gps_valid || !imu_valid || !heading_valid || !local_frame_valid) {
            applyAutoWorkspaceDriveCommand(
                true,
                0,
                0,
                "waiting_sensor",
                last_speed_cm_s,
                last_steering_encoder,
                has_last_navigation_command,
                last_command_sent_steady,
                control_cycle_id);
            runtime_update.task_running = true;
            runtime_update.avoidance_active = false;
            runtime_update.phase = "waiting_sensor";
            runtime_update.message =
                !gps_valid ? "自动任务等待 GPS" :
                (!imu_valid ? "自动任务等待 IMU" :
                    (!heading_valid ? "自动任务等待 GPS/IMU 航向对齐" :
                        "自动任务等待 XY 原点"));
            runtime_update.command_speed_cm_s = 0;
            runtime_update.command_steering_encoder = 0;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                auto_workspace_runtime_state_ = runtime_update;
                auto_workspace_avoid_runtime_state_.active = false;
                auto_workspace_avoid_runtime_state_.has_decision = false;
                auto_workspace_avoid_runtime_state_.last_apply_result = "waiting_sensor";
            }
            std::this_thread::sleep_until(next_tick);
            continue;
        }

        const bool has_route_path = !plan_state.preview_path.empty();
        const std::size_t nearest_path_index =
            has_route_path ? nearestPathPointIndex(
                plan_state.preview_path,
                current_x_m,
                current_y_m) : 0;
        const std::size_t target_path_index =
            has_route_path ? lookaheadPathPointIndex(
                plan_state.preview_path,
                nearest_path_index,
                current_x_m,
                current_y_m,
                kAutoWorkspacePathLookaheadM) : 0;
        double final_target_x_m = plan_state.destination_x_m;
        double final_target_y_m = plan_state.destination_y_m;
        double tracking_target_latitude = plan_state.destination_latitude;
        double tracking_target_longitude = plan_state.destination_longitude;
        if (has_route_path) {
            final_target_x_m = plan_state.preview_path.back().x_m;
            final_target_y_m = plan_state.preview_path.back().y_m;
            tracking_target_latitude = plan_state.preview_path[target_path_index].latitude;
            tracking_target_longitude = plan_state.preview_path[target_path_index].longitude;
        }
        const double arrival_distance_m =
            has_route_path ?
                std::hypot(final_target_x_m - current_x_m, final_target_y_m - current_y_m) :
                std::hypot(plan_state.destination_x_m - current_x_m, plan_state.destination_y_m - current_y_m);
        const double remaining_distance_m =
            has_route_path ?
                remainingPathDistanceMeters(
                    plan_state.preview_path,
                    nearest_path_index,
                    current_x_m,
                    current_y_m) :
                std::hypot(plan_state.destination_x_m - current_x_m, plan_state.destination_y_m - current_y_m);
        const double target_bearing_deg = initialBearingDeg(
            current_latitude,
            current_longitude,
            tracking_target_latitude,
            tracking_target_longitude);
        const double heading_error_deg =
            normalizeHeadingDifferenceDeg(target_bearing_deg - current_yaw_deg);

        runtime_update.remaining_distance_m = remaining_distance_m;
        runtime_update.target_bearing_deg = target_bearing_deg;
        runtime_update.heading_error_deg = heading_error_deg;
        runtime_update.active_path_index = target_path_index;

        if (arrival_distance_m <= kAutoWorkspaceArriveDistanceM) {
            applyAutoWorkspaceDriveCommand(
                true,
                0,
                0,
                "arrived",
                last_speed_cm_s,
                last_steering_encoder,
                has_last_navigation_command,
                last_command_sent_steady,
                control_cycle_id);
            runtime_update.task_running = false;
            runtime_update.avoidance_active = false;
            runtime_update.target_reached = true;
            runtime_update.active_path_index =
                has_route_path && !plan_state.preview_path.empty() ?
                    plan_state.preview_path.size() - 1 :
                    0;
            runtime_update.phase = "arrived";
            runtime_update.message = "已到达目标点，车辆已停止";
            runtime_update.command_speed_cm_s = 0;
            runtime_update.command_steering_encoder = 0;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                auto_workspace_runtime_state_ = runtime_update;
                auto_workspace_avoid_runtime_state_.active = false;
                auto_workspace_avoid_runtime_state_.has_decision = false;
                auto_workspace_avoid_runtime_state_.last_apply_result = "arrived";
            }
            auto_workspace_control_running_.store(false);
            break;
        }

        auto snapshot = autoWorkspaceAvoidSensorSnapshot();
        const bool front_close =
            snapshot.lidar_valid &&
            snapshot.front.valid &&
            std::isfinite(snapshot.front.nearest_m) &&
            snapshot.front.nearest_m <
                auto_workspace_avoid_controller_.config().avoidance_turn_max_distance_m;
        const bool avoidance_active =
            auto_workspace_avoid_controller_.currentAvoidanceStage() !=
                AutoAvoidController::AvoidanceStage::Idle ||
            front_close;

        if (avoidance_active) {
            has_last_navigation_command = false;
            const auto command = auto_workspace_avoid_controller_.decide(snapshot);
            const auto command_trace = applyAutoAvoidCommand(
                command,
                last_avoidance_command,
                has_last_avoidance_command,
                last_command_sent_steady,
                control_cycle_id);
            (void)command_trace;
            runtime_update.task_running = true;
            runtime_update.avoidance_active = true;
            runtime_update.phase = "avoiding";
            runtime_update.message = "前向近障，自动工作区正在调用避障链";
            runtime_update.command_speed_cm_s = command.speed_cm_s;
            runtime_update.command_steering_encoder = command.steering_encoder;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                auto_workspace_runtime_state_ = runtime_update;
                auto_workspace_avoid_runtime_state_.active = true;
                auto_workspace_avoid_runtime_state_.has_decision = true;
                auto_workspace_avoid_runtime_state_.last_control_cycle_id = control_cycle_id;
                auto_workspace_avoid_runtime_state_.snapshot_valid =
                    snapshot.lidar_valid && snapshot.imu_valid;
                auto_workspace_avoid_runtime_state_.lidar_valid = snapshot.lidar_valid;
                auto_workspace_avoid_runtime_state_.imu_valid = snapshot.imu_valid;
                auto_workspace_avoid_runtime_state_.last_decision = command;
                auto_workspace_avoid_runtime_state_.last_apply_result = command_trace.result;
            }
        } else {
            auto_workspace_avoid_controller_.reset();
            has_last_avoidance_command = false;
            last_avoidance_command = AutoAvoidController::Command{};

            const int speed_cm_s =
                remaining_distance_m <= kAutoWorkspaceApproachDistanceM ?
                    kAutoWorkspaceApproachSpeedCmS :
                    kAutoWorkspaceCruiseSpeedCmS;
            const int steering_encoder =
                autoWorkspaceSteeringEncoderForHeadingError(heading_error_deg);
            applyAutoWorkspaceDriveCommand(
                false,
                speed_cm_s,
                steering_encoder,
                "tracking",
                last_speed_cm_s,
                last_steering_encoder,
                has_last_navigation_command,
                last_command_sent_steady,
                control_cycle_id);
            runtime_update.task_running = true;
            runtime_update.avoidance_active = false;
            runtime_update.phase = "tracking";
            runtime_update.message = "自动任务沿道路路径前进中";
            runtime_update.command_speed_cm_s = speed_cm_s;
            runtime_update.command_steering_encoder = steering_encoder;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                auto_workspace_runtime_state_ = runtime_update;
                auto_workspace_avoid_runtime_state_.active = false;
                auto_workspace_avoid_runtime_state_.has_decision = false;
                auto_workspace_avoid_runtime_state_.last_control_cycle_id = control_cycle_id;
                auto_workspace_avoid_runtime_state_.snapshot_valid = false;
                auto_workspace_avoid_runtime_state_.lidar_valid = false;
                auto_workspace_avoid_runtime_state_.imu_valid = false;
                auto_workspace_avoid_runtime_state_.last_decision =
                    AutoAvoidController::Command{};
                auto_workspace_avoid_runtime_state_.last_apply_result = "inactive";
            }
        }

        std::this_thread::sleep_until(next_tick);
        if (std::chrono::steady_clock::now() > next_tick + 200ms) {
            next_tick = std::chrono::steady_clock::now();
        }
    }
}

void LogDashboardServer::startStm32Bridge() {
    std::lock_guard<std::mutex> lock(stm32_mutex_);
    if (to_stm_) {
        return;
    }

    std::error_code remove_error;
    fs::remove(fs::path(kStm32StatusSnapshotPath), remove_error);

    const std::string device = envStringOrDefault("PROJECT_STM32_PORT", "/dev/stm32");
    const int baudrate = envIntOrDefault("PROJECT_STM32_BAUD", 115200);

    auto uart = std::make_unique<UART>(device, baudrate);
    if (!uart->init()) {
        log_warn(
            kLogModule,
            "stm32 serial init failed: device=" + device + ", baud=" + std::to_string(baudrate));
        return;
    }

    uart->start_receive_thread();
    stm32_uart_ = std::move(uart);
    to_stm_ = std::make_unique<ToStm>(*stm32_uart_);
    log_info(kLogModule, "stm32 serial ready: device=" + device + ", baud=" + std::to_string(baudrate));
}

void LogDashboardServer::stopStm32Bridge() {
    std::lock_guard<std::mutex> lock(stm32_mutex_);
    to_stm_.reset();
    if (stm32_uart_) {
        stm32_uart_->stop_receive_thread();
        stm32_uart_->close_uart();
        stm32_uart_.reset();
    }
}

void LogDashboardServer::startRuntimeLogBridge() {
    if (runtime_log_bridge_running_.exchange(true)) {
        return;
    }

    const fs::path runtime_dir = fs::path(runtimeLogsDir());
    const fs::path stack_log_path = runtime_dir / "project_stack.log";
    const fs::path stack_pid_path = runtime_dir / "pids" / "project_stack.pid";

    runtime_log_bridge_thread_ = std::thread([this, stack_log_path, stack_pid_path]() {
        std::uintmax_t last_size = 0;
        bool bootstrapped = false;

        while (runtime_log_bridge_running_.load()) {
            const ComponentProcessState stack_process =
                componentProcessState(stack_pid_path, kStackProcessNeedle);
            if (stack_process.running &&
                (std::chrono::system_clock::now() - stack_process.started_at) <= kStartupInfoWindow) {
                logger_set_info_enabled(true);
            } else {
                logger_set_info_enabled(false);
            }

            std::error_code exists_error;
            if (fs::exists(stack_log_path, exists_error) && !exists_error) {
                std::error_code size_error;
                const std::uintmax_t current_size = fs::file_size(stack_log_path, size_error);
                if (!size_error) {
                    if (!bootstrapped) {
                        std::istringstream bootstrap_stream(recentRuntimeLogBootstrap(stack_log_path));
                        std::string line;
                        while (std::getline(bootstrap_stream, line)) {
                            appendRuntimeLogLine(kStackLogModule, line);
                        }
                        last_size = current_size;
                        bootstrapped = true;
                    } else {
                        if (current_size < last_size) {
                            last_size = 0;
                        }

                        std::ifstream input(stack_log_path);
                        if (input.is_open()) {
                            input.seekg(static_cast<std::streamoff>(last_size), std::ios::beg);
                            std::string line;
                            while (std::getline(input, line)) {
                                appendRuntimeLogLine(kStackLogModule, line);
                            }
                            last_size = current_size;
                        }
                    }
                }
            }

            std::this_thread::sleep_for(kRuntimeLogPollInterval);
        }

        logger_set_info_enabled(false);
    });
}

void LogDashboardServer::stopRuntimeLogBridge() {
    runtime_log_bridge_running_.store(false);
    if (runtime_log_bridge_thread_.joinable()) {
        runtime_log_bridge_thread_.join();
    }
}

void LogDashboardServer::onDepthImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    const auto now = std::chrono::steady_clock::now();
    if (!msg) {
        return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    depth_state_.message_count += 1;
    depth_state_.last_message_steady_ = now;
}

void LogDashboardServer::resetLidarDisplayFilterLocked(
    LidarDisplayFilterState& filter_state) {
    filter_state = LidarDisplayFilterState{};
}

LogDashboardServer::LidarRuntimeState::SectorRuntimeState
LogDashboardServer::filteredLidarDisplaySectorLocked(
    const LidarRuntimeState::SectorRuntimeState& sample,
    std::deque<LidarRuntimeState::SectorRuntimeState>& history,
    int filter_window) const {
    const bool sample_valid =
        sample.valid &&
        std::isfinite(sample.nearest_m) &&
        std::isfinite(sample.nearest_angle_deg);
    if (!sample_valid) {
        history.clear();
        return sample;
    }

    history.push_back(sample);
    while (static_cast<int>(history.size()) > filter_window) {
        history.pop_front();
    }

    std::vector<LidarRuntimeState::SectorRuntimeState> ordered(history.begin(), history.end());
    std::sort(
        ordered.begin(),
        ordered.end(),
        [](const auto& lhs, const auto& rhs) {
            return lhs.nearest_m < rhs.nearest_m;
        });
    return ordered[ordered.size() / 2];
}

LogDashboardServer::LidarRuntimeState::SectorRuntimeState
LogDashboardServer::filterLidarDisplayFrontLocked(
    const LidarRuntimeState::SectorRuntimeState& front,
    const LidarRuntimeState::SectorRuntimeState& negative_front,
    const LidarRuntimeState::SectorRuntimeState& positive_front,
    LidarDisplayFilterState& filter_state,
    const AutoAvoidController::Config& config) {
    const int filter_window = std::max(1, config.lidar_filter_window);
    const auto filtered_negative_front = filteredLidarDisplaySectorLocked(
        negative_front,
        filter_state.negative_front_history,
        filter_window);
    const auto filtered_positive_front = filteredLidarDisplaySectorLocked(
        positive_front,
        filter_state.positive_front_history,
        filter_window);
    auto filtered_front = filteredLidarDisplaySectorLocked(
        front,
        filter_state.front_history,
        filter_window);

    if (!filtered_front.valid ||
        !std::isfinite(filtered_front.nearest_m) ||
        !std::isfinite(filtered_front.nearest_angle_deg)) {
        filter_state.has_last_stable_front = false;
        filter_state.last_stable_front = LidarRuntimeState::SectorRuntimeState{};
        filter_state.has_pending_front_spike = false;
        filter_state.pending_front_spike = LidarRuntimeState::SectorRuntimeState{};
        filter_state.pending_front_spike_ticks = 0;
        return filtered_front;
    }

    const bool side_clear =
        (!filtered_negative_front.valid ||
            !std::isfinite(filtered_negative_front.nearest_m) ||
            filtered_negative_front.nearest_m >= config.front_spike_side_clear_distance_m) &&
        (!filtered_positive_front.valid ||
            !std::isfinite(filtered_positive_front.nearest_m) ||
            filtered_positive_front.nearest_m >= config.front_spike_side_clear_distance_m);
    const bool suspicious_center_spike =
        filter_state.has_last_stable_front &&
        std::isfinite(filter_state.last_stable_front.nearest_m) &&
        filter_state.last_stable_front.nearest_m > config.avoidance_turn_max_distance_m &&
        std::abs(filtered_front.nearest_angle_deg) <= config.front_spike_center_half_width_deg &&
        filtered_front.nearest_m + config.front_spike_jump_distance_m <
            filter_state.last_stable_front.nearest_m &&
        filtered_front.support_points > 0 &&
        filtered_front.support_points <= config.front_spike_max_support_points &&
        side_clear;

    if (suspicious_center_spike) {
        const int confirm_ticks =
            filtered_front.nearest_m <= config.emergency_stop_distance_m ?
                1 :
                std::max(1, config.front_spike_confirm_ticks);
        const bool similar_pending_spike =
            filter_state.has_pending_front_spike &&
            std::isfinite(filter_state.pending_front_spike.nearest_m) &&
            std::isfinite(filter_state.pending_front_spike.nearest_angle_deg) &&
            std::abs(filtered_front.nearest_m - filter_state.pending_front_spike.nearest_m) <=
                0.30 &&
            std::abs(
                filtered_front.nearest_angle_deg -
                filter_state.pending_front_spike.nearest_angle_deg) <= 8.0;
        if (!similar_pending_spike) {
            filter_state.pending_front_spike = filtered_front;
            filter_state.has_pending_front_spike = true;
            filter_state.pending_front_spike_ticks = 1;
        } else {
            ++filter_state.pending_front_spike_ticks;
        }

        if (filter_state.pending_front_spike_ticks < confirm_ticks) {
            return filter_state.last_stable_front;
        }
    } else {
        filter_state.has_pending_front_spike = false;
        filter_state.pending_front_spike = LidarRuntimeState::SectorRuntimeState{};
        filter_state.pending_front_spike_ticks = 0;
    }

    filter_state.has_last_stable_front = true;
    filter_state.last_stable_front = filtered_front;
    filter_state.has_pending_front_spike = false;
    filter_state.pending_front_spike = LidarRuntimeState::SectorRuntimeState{};
    filter_state.pending_front_spike_ticks = 0;
    return filtered_front;
}

void LogDashboardServer::updateLidarRuntimeLocked(
    LidarRuntimeState& runtime_state,
    LidarDisplayFilterState& filter_state,
    const AutoAvoidInputBuilder::LidarInputFrame& lidar_input_frame,
    const AutoAvoidController::Config& config) {
    runtime_state.message_count += 1;
    runtime_state.last_message_steady_ = std::chrono::steady_clock::now();
    runtime_state.valid = lidar_input_frame.valid;
    runtime_state.valid_points = lidar_input_frame.valid_points;
    runtime_state.negative_front_sector = lidar_input_frame.negative_front_sector;
    runtime_state.front_sector = lidar_input_frame.front_sector;
    runtime_state.auto_avoid_front_sector = lidar_input_frame.auto_avoid_front_sector;
    runtime_state.positive_front_sector = lidar_input_frame.positive_front_sector;
    runtime_state.avoidance_buffer_sector = lidar_input_frame.avoidance_buffer_sector;
    runtime_state.front_target_selection = lidar_input_frame.front_target_selection;
    runtime_state.front_nearest_zone = lidar_input_frame.front_nearest_zone;
    if (lidar_input_frame.valid) {
        runtime_state.filtered_auto_avoid_front_sector = filterLidarDisplayFrontLocked(
            lidar_input_frame.auto_avoid_front_sector,
            lidar_input_frame.negative_front_sector,
            lidar_input_frame.positive_front_sector,
            filter_state,
            config);
        runtime_state.filtered_front_nearest_zone =
            runtime_state.filtered_auto_avoid_front_sector.valid ?
                AutoAvoidInputBuilder::frontNearestZoneFromAngle(
                    runtime_state.filtered_auto_avoid_front_sector.nearest_angle_deg,
                    lidar_input_frame.front_nearest_zone) :
                lidar_input_frame.front_nearest_zone;
    } else {
        resetLidarDisplayFilterLocked(filter_state);
        runtime_state.filtered_auto_avoid_front_sector = lidar_input_frame.auto_avoid_front_sector;
        runtime_state.filtered_front_nearest_zone = lidar_input_frame.front_nearest_zone;
    }
}

void LogDashboardServer::onLidarScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const auto now = std::chrono::steady_clock::now();
    const std::string previous_front_nearest_zone =
        auto_avoid_control_snapshot_pool_.latestFrontNearestZone();
    const auto avoidance_lidar_input_frame =
        auto_avoid_input_builder_.buildLidarInputFrame(msg, previous_front_nearest_zone);
    auto_avoid_control_snapshot_pool_.updateLidar(avoidance_lidar_input_frame, now);
    const std::string auto_workspace_previous_zone =
        auto_workspace_avoid_control_snapshot_pool_.latestFrontNearestZone();
    const auto auto_workspace_lidar_input_frame =
        auto_workspace_avoid_input_builder_.buildLidarInputFrame(
            msg,
            auto_workspace_previous_zone);
    auto_workspace_avoid_control_snapshot_pool_.updateLidar(
        auto_workspace_lidar_input_frame,
        now);

    std::lock_guard<std::mutex> lock(state_mutex_);
    updateLidarRuntimeLocked(
        lidar_state_,
        lidar_display_filter_state_,
        avoidance_lidar_input_frame,
        auto_avoid_controller_.config());
    lidar_state_.last_message_steady_ = now;
    updateLidarRuntimeLocked(
        auto_workspace_lidar_state_,
        auto_workspace_lidar_display_filter_state_,
        auto_workspace_lidar_input_frame,
        auto_workspace_avoid_controller_.config());
    auto_workspace_lidar_state_.last_message_steady_ = now;
}

void LogDashboardServer::onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const auto now = std::chrono::steady_clock::now();
    bool has_attitude = false;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    if (!msg) {
        auto_avoid_control_snapshot_pool_.updateImu(
            false,
            roll_deg,
            pitch_deg,
            yaw_deg,
            now);
        auto_workspace_avoid_control_snapshot_pool_.updateImu(
            false,
            roll_deg,
            pitch_deg,
            yaw_deg,
            now);
    } else {
        const double x = msg->orientation.x;
        const double y = msg->orientation.y;
        const double z = msg->orientation.z;
        const double w = msg->orientation.w;
        const double norm = std::sqrt(x * x + y * y + z * z + w * w);
        if (std::isfinite(norm) && norm >= 1e-6) {
            const double qx = x / norm;
            const double qy = y / norm;
            const double qz = z / norm;
            const double qw = w / norm;
            const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
            const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
            const double sinp = 2.0 * (qw * qy - qz * qx);
            const double siny_cosp = 2.0 * (qw * qz + qx * qy);
            const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            const double rad_to_deg = 180.0 / std::acos(-1.0);

            roll_deg = std::atan2(sinr_cosp, cosr_cosp) * rad_to_deg;
            pitch_deg =
                std::abs(sinp) >= 1.0 ?
                    std::copysign(90.0, sinp) :
                    std::asin(sinp) * rad_to_deg;
            yaw_deg = std::atan2(siny_cosp, cosy_cosp) * rad_to_deg;
            has_attitude = true;
        }
        auto_avoid_control_snapshot_pool_.updateImu(
            has_attitude,
            roll_deg,
            pitch_deg,
            yaw_deg,
            now);
        auto_workspace_avoid_control_snapshot_pool_.updateImu(
            has_attitude,
            roll_deg,
            pitch_deg,
            yaw_deg,
            now);
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    imu_state_.message_count += 1;
    imu_state_.last_message_steady_ = now;
    imu_state_.roll_deg = roll_deg;
    imu_state_.pitch_deg = pitch_deg;
    imu_state_.yaw_deg = yaw_deg;
    imu_state_.has_attitude = has_attitude;
    auto_workspace_imu_state_.message_count += 1;
    auto_workspace_imu_state_.last_message_steady_ = now;
    auto_workspace_imu_state_.roll_deg = roll_deg;
    auto_workspace_imu_state_.pitch_deg = pitch_deg;
    auto_workspace_imu_state_.yaw_deg = yaw_deg;
    auto_workspace_imu_state_.has_attitude = has_attitude;
    if (auto_workspace_gps_filter_state_.heading_offset_valid) {
        auto_workspace_gps_state_.heading_valid = has_attitude;
        auto_workspace_gps_state_.heading_deg =
            normalizeHeadingDifferenceDeg(
                yaw_deg + auto_workspace_gps_filter_state_.heading_offset_deg);
        auto_workspace_gps_state_.heading_source =
            auto_workspace_gps_filter_state_.heading_source;
        if (auto_workspace_gps_state_.filtered_valid) {
            const auto [gps_offset_x_m, gps_offset_y_m] = planarOffsetFromBodyFrameMeters(
                auto_workspace_gps_state_.heading_deg,
                kAutoWorkspaceGpsOffsetXM,
                kAutoWorkspaceGpsOffsetYM);
            const auto [base_link_latitude, base_link_longitude] =
                latLonFromLocalPlanarOffsetMeters(
                    auto_workspace_gps_filter_state_.anchor_latitude,
                    auto_workspace_gps_filter_state_.anchor_longitude,
                    auto_workspace_gps_filter_state_.filtered_x_m - gps_offset_x_m,
                    auto_workspace_gps_filter_state_.filtered_y_m - gps_offset_y_m);
            auto_workspace_gps_state_.base_link_valid = true;
            auto_workspace_gps_state_.base_link_latitude = base_link_latitude;
            auto_workspace_gps_state_.base_link_longitude = base_link_longitude;
        }
    }
}

void LogDashboardServer::resetGpsFilterLocked() {
    gps_filter_state_ = GpsFilterState {};
    gps_state_.filtered_valid = false;
    gps_state_.origin_stable = false;
    gps_state_.origin_stability_radius_m = 0.0;
    gps_state_.origin_stability_samples = 0;
}

void LogDashboardServer::resetAutoWorkspaceGpsFilterLocked() {
    auto_workspace_gps_filter_state_ = GpsFilterState {};
    auto_workspace_gps_state_.filtered_valid = false;
    auto_workspace_gps_state_.origin_stable = false;
    auto_workspace_gps_state_.origin_stability_radius_m = 0.0;
    auto_workspace_gps_state_.origin_stability_samples = 0;
    auto_workspace_gps_state_.heading_valid = false;
    auto_workspace_gps_state_.heading_deg = 0.0;
    auto_workspace_gps_state_.heading_source.clear();
    auto_workspace_gps_state_.base_link_valid = false;
    auto_workspace_gps_state_.base_link_latitude = 0.0;
    auto_workspace_gps_state_.base_link_longitude = 0.0;
}

void LogDashboardServer::updateFilteredGpsLocked(
    double latitude,
    double longitude,
    double altitude,
    double horizontal_stddev_m) {
    if (!gps_filter_state_.anchor_valid) {
        gps_filter_state_.anchor_valid = true;
        gps_filter_state_.anchor_latitude = latitude;
        gps_filter_state_.anchor_longitude = longitude;
    }

    const auto [raw_x_m, raw_y_m] = localPlanarOffsetMeters(
        gps_filter_state_.anchor_latitude,
        gps_filter_state_.anchor_longitude,
        latitude,
        longitude);

    GpsFilterSample raw_sample;
    raw_sample.x_m = raw_x_m;
    raw_sample.y_m = raw_y_m;
    raw_sample.altitude_m = altitude;
    raw_sample.horizontal_stddev_m = horizontal_stddev_m;
    gps_filter_state_.raw_window.push_back(raw_sample);
    while (gps_filter_state_.raw_window.size() > kGpsMedianWindowSize) {
        gps_filter_state_.raw_window.pop_front();
    }

    std::vector<double> recent_x_m;
    std::vector<double> recent_y_m;
    std::vector<double> recent_altitude_m;
    std::vector<double> recent_horizontal_stddev_m;
    recent_x_m.reserve(gps_filter_state_.raw_window.size());
    recent_y_m.reserve(gps_filter_state_.raw_window.size());
    recent_altitude_m.reserve(gps_filter_state_.raw_window.size());
    recent_horizontal_stddev_m.reserve(gps_filter_state_.raw_window.size());
    for (const auto& sample : gps_filter_state_.raw_window) {
        recent_x_m.push_back(sample.x_m);
        recent_y_m.push_back(sample.y_m);
        recent_altitude_m.push_back(sample.altitude_m);
        if (std::isfinite(sample.horizontal_stddev_m) && sample.horizontal_stddev_m > 0.0) {
            recent_horizontal_stddev_m.push_back(sample.horizontal_stddev_m);
        }
    }

    const double median_x_m = medianValue(recent_x_m);
    const double median_y_m = medianValue(recent_y_m);
    const double median_altitude_m = medianValue(recent_altitude_m);
    const double filtered_horizontal_stddev_m =
        recent_horizontal_stddev_m.empty() ?
            horizontal_stddev_m :
            medianValue(recent_horizontal_stddev_m);

    if (!gps_filter_state_.filtered_valid) {
        gps_filter_state_.filtered_x_m = median_x_m;
        gps_filter_state_.filtered_y_m = median_y_m;
        gps_filter_state_.filtered_altitude_m = median_altitude_m;
        gps_filter_state_.filtered_valid = true;
    } else {
        gps_filter_state_.filtered_x_m +=
            (median_x_m - gps_filter_state_.filtered_x_m) * kGpsFilterEmaAlpha;
        gps_filter_state_.filtered_y_m +=
            (median_y_m - gps_filter_state_.filtered_y_m) * kGpsFilterEmaAlpha;
        gps_filter_state_.filtered_altitude_m +=
            (median_altitude_m - gps_filter_state_.filtered_altitude_m) * kGpsFilterEmaAlpha;
    }

    GpsFilterSample filtered_sample;
    filtered_sample.x_m = gps_filter_state_.filtered_x_m;
    filtered_sample.y_m = gps_filter_state_.filtered_y_m;
    filtered_sample.altitude_m = gps_filter_state_.filtered_altitude_m;
    filtered_sample.horizontal_stddev_m = filtered_horizontal_stddev_m;
    gps_filter_state_.filtered_window.push_back(filtered_sample);
    while (gps_filter_state_.filtered_window.size() > kGpsOriginStabilityWindowSize) {
        gps_filter_state_.filtered_window.pop_front();
    }

    const auto [filtered_latitude, filtered_longitude] = latLonFromLocalPlanarOffsetMeters(
        gps_filter_state_.anchor_latitude,
        gps_filter_state_.anchor_longitude,
        gps_filter_state_.filtered_x_m,
        gps_filter_state_.filtered_y_m);

    gps_state_.filtered_valid = true;
    gps_state_.latitude = filtered_latitude;
    gps_state_.longitude = filtered_longitude;
    gps_state_.altitude = gps_filter_state_.filtered_altitude_m;
    gps_state_.horizontal_stddev_m = filtered_horizontal_stddev_m;

    double centroid_x_m = 0.0;
    double centroid_y_m = 0.0;
    double centroid_altitude_m = 0.0;
    double cluster_radius_m = 0.0;
    double mean_horizontal_stddev_m = 0.0;
    std::size_t accuracy_sample_count = 0;
    for (const auto& sample : gps_filter_state_.filtered_window) {
        centroid_x_m += sample.x_m;
        centroid_y_m += sample.y_m;
        centroid_altitude_m += sample.altitude_m;
        if (std::isfinite(sample.horizontal_stddev_m) && sample.horizontal_stddev_m > 0.0) {
            mean_horizontal_stddev_m += sample.horizontal_stddev_m;
            ++accuracy_sample_count;
        }
    }
    if (!gps_filter_state_.filtered_window.empty()) {
        centroid_x_m /= static_cast<double>(gps_filter_state_.filtered_window.size());
        centroid_y_m /= static_cast<double>(gps_filter_state_.filtered_window.size());
        centroid_altitude_m /= static_cast<double>(gps_filter_state_.filtered_window.size());
        for (const auto& sample : gps_filter_state_.filtered_window) {
            cluster_radius_m = std::max(
                cluster_radius_m,
                std::hypot(sample.x_m - centroid_x_m, sample.y_m - centroid_y_m));
        }
    }
    if (accuracy_sample_count > 0) {
        mean_horizontal_stddev_m /= static_cast<double>(accuracy_sample_count);
    }

    gps_state_.origin_stability_radius_m = cluster_radius_m;
    gps_state_.origin_stability_samples = gps_filter_state_.filtered_window.size();

    const bool enough_samples =
        gps_filter_state_.filtered_window.size() >= kGpsOriginStabilityWindowSize;
    const bool accuracy_ok =
        accuracy_sample_count == 0 ||
        mean_horizontal_stddev_m <= kGpsOriginStableHorizontalStdDevM;
    const bool origin_stable =
        enough_samples &&
        accuracy_ok &&
        cluster_radius_m <= kGpsOriginStableRadiusM;
    gps_state_.origin_stable = origin_stable;

    if (origin_stable && !auto_workspace_local_frame_state_.valid) {
        const auto [origin_latitude, origin_longitude] = latLonFromLocalPlanarOffsetMeters(
            gps_filter_state_.anchor_latitude,
            gps_filter_state_.anchor_longitude,
            centroid_x_m,
            centroid_y_m);
        auto_workspace_local_frame_state_.valid = true;
        auto_workspace_local_frame_state_.origin_latitude = origin_latitude;
        auto_workspace_local_frame_state_.origin_longitude = origin_longitude;
        auto_workspace_local_frame_state_.origin_altitude = centroid_altitude_m;
    }
}

void LogDashboardServer::updateAutoWorkspaceHeadingFromRouteLocked(double route_bearing_deg) {
    const auto now = std::chrono::steady_clock::now();
    const bool imu_valid =
        auto_workspace_imu_state_.has_attitude &&
        auto_workspace_imu_state_.message_count > 0 &&
        (now - auto_workspace_imu_state_.last_message_steady_) <= kSensorFreshWindow &&
        std::isfinite(auto_workspace_imu_state_.yaw_deg) &&
        std::isfinite(route_bearing_deg);
    if (!imu_valid) {
        return;
    }

    auto_workspace_gps_filter_state_.heading_offset_valid = true;
    auto_workspace_gps_filter_state_.heading_offset_deg =
        normalizeHeadingDifferenceDeg(route_bearing_deg - auto_workspace_imu_state_.yaw_deg);
    auto_workspace_gps_filter_state_.heading_source = "route_seed";
    auto_workspace_gps_state_.heading_valid = true;
    auto_workspace_gps_state_.heading_deg =
        normalizeHeadingDifferenceDeg(
            auto_workspace_imu_state_.yaw_deg +
            auto_workspace_gps_filter_state_.heading_offset_deg);
    auto_workspace_gps_state_.heading_source = auto_workspace_gps_filter_state_.heading_source;

    if (auto_workspace_gps_state_.filtered_valid) {
        const auto [gps_offset_x_m, gps_offset_y_m] = planarOffsetFromBodyFrameMeters(
            auto_workspace_gps_state_.heading_deg,
            kAutoWorkspaceGpsOffsetXM,
            kAutoWorkspaceGpsOffsetYM);
        const auto [base_link_latitude, base_link_longitude] =
            latLonFromLocalPlanarOffsetMeters(
                auto_workspace_gps_filter_state_.anchor_latitude,
                auto_workspace_gps_filter_state_.anchor_longitude,
                auto_workspace_gps_filter_state_.filtered_x_m - gps_offset_x_m,
                auto_workspace_gps_filter_state_.filtered_y_m - gps_offset_y_m);
        auto_workspace_gps_state_.base_link_valid = true;
        auto_workspace_gps_state_.base_link_latitude = base_link_latitude;
        auto_workspace_gps_state_.base_link_longitude = base_link_longitude;
    }
}

void LogDashboardServer::updateAutoWorkspaceFilteredGpsLocked(
    double latitude,
    double longitude,
    double altitude,
    double horizontal_stddev_m,
    const std::chrono::steady_clock::time_point& now) {
    if (!auto_workspace_gps_filter_state_.anchor_valid) {
        auto_workspace_gps_filter_state_.anchor_valid = true;
        auto_workspace_gps_filter_state_.anchor_latitude = latitude;
        auto_workspace_gps_filter_state_.anchor_longitude = longitude;
    }

    const auto [raw_x_m, raw_y_m] = localPlanarOffsetMeters(
        auto_workspace_gps_filter_state_.anchor_latitude,
        auto_workspace_gps_filter_state_.anchor_longitude,
        latitude,
        longitude);

    GpsFilterSample raw_sample;
    raw_sample.x_m = raw_x_m;
    raw_sample.y_m = raw_y_m;
    raw_sample.altitude_m = altitude;
    raw_sample.horizontal_stddev_m = horizontal_stddev_m;
    auto_workspace_gps_filter_state_.raw_window.push_back(raw_sample);
    while (auto_workspace_gps_filter_state_.raw_window.size() > kGpsMedianWindowSize) {
        auto_workspace_gps_filter_state_.raw_window.pop_front();
    }

    std::vector<double> recent_x_m;
    std::vector<double> recent_y_m;
    std::vector<double> recent_altitude_m;
    std::vector<double> recent_horizontal_stddev_m;
    recent_x_m.reserve(auto_workspace_gps_filter_state_.raw_window.size());
    recent_y_m.reserve(auto_workspace_gps_filter_state_.raw_window.size());
    recent_altitude_m.reserve(auto_workspace_gps_filter_state_.raw_window.size());
    recent_horizontal_stddev_m.reserve(auto_workspace_gps_filter_state_.raw_window.size());
    for (const auto& sample : auto_workspace_gps_filter_state_.raw_window) {
        recent_x_m.push_back(sample.x_m);
        recent_y_m.push_back(sample.y_m);
        recent_altitude_m.push_back(sample.altitude_m);
        if (std::isfinite(sample.horizontal_stddev_m) && sample.horizontal_stddev_m > 0.0) {
            recent_horizontal_stddev_m.push_back(sample.horizontal_stddev_m);
        }
    }

    const double median_x_m = medianValue(recent_x_m);
    const double median_y_m = medianValue(recent_y_m);
    const double median_altitude_m = medianValue(recent_altitude_m);
    const double filtered_horizontal_stddev_m =
        recent_horizontal_stddev_m.empty() ?
            horizontal_stddev_m :
            medianValue(recent_horizontal_stddev_m);

    if (!auto_workspace_gps_filter_state_.filtered_valid) {
        auto_workspace_gps_filter_state_.filtered_x_m = median_x_m;
        auto_workspace_gps_filter_state_.filtered_y_m = median_y_m;
        auto_workspace_gps_filter_state_.filtered_altitude_m = median_altitude_m;
        auto_workspace_gps_filter_state_.filtered_valid = true;
    } else {
        auto_workspace_gps_filter_state_.filtered_x_m +=
            (median_x_m - auto_workspace_gps_filter_state_.filtered_x_m) * kGpsFilterEmaAlpha;
        auto_workspace_gps_filter_state_.filtered_y_m +=
            (median_y_m - auto_workspace_gps_filter_state_.filtered_y_m) * kGpsFilterEmaAlpha;
        auto_workspace_gps_filter_state_.filtered_altitude_m +=
            (median_altitude_m - auto_workspace_gps_filter_state_.filtered_altitude_m) * kGpsFilterEmaAlpha;
    }

    GpsFilterSample filtered_sample;
    filtered_sample.x_m = auto_workspace_gps_filter_state_.filtered_x_m;
    filtered_sample.y_m = auto_workspace_gps_filter_state_.filtered_y_m;
    filtered_sample.altitude_m = auto_workspace_gps_filter_state_.filtered_altitude_m;
    filtered_sample.horizontal_stddev_m = filtered_horizontal_stddev_m;
    auto_workspace_gps_filter_state_.filtered_window.push_back(filtered_sample);
    while (auto_workspace_gps_filter_state_.filtered_window.size() > kGpsOriginStabilityWindowSize) {
        auto_workspace_gps_filter_state_.filtered_window.pop_front();
    }

    const auto [filtered_latitude, filtered_longitude] = latLonFromLocalPlanarOffsetMeters(
        auto_workspace_gps_filter_state_.anchor_latitude,
        auto_workspace_gps_filter_state_.anchor_longitude,
        auto_workspace_gps_filter_state_.filtered_x_m,
        auto_workspace_gps_filter_state_.filtered_y_m);

    auto_workspace_gps_state_.filtered_valid = true;
    auto_workspace_gps_state_.latitude = filtered_latitude;
    auto_workspace_gps_state_.longitude = filtered_longitude;
    auto_workspace_gps_state_.altitude = auto_workspace_gps_filter_state_.filtered_altitude_m;
    auto_workspace_gps_state_.horizontal_stddev_m = filtered_horizontal_stddev_m;

    double centroid_x_m = 0.0;
    double centroid_y_m = 0.0;
    double centroid_altitude_m = 0.0;
    double cluster_radius_m = 0.0;
    double mean_horizontal_stddev_m = 0.0;
    std::size_t accuracy_sample_count = 0;
    for (const auto& sample : auto_workspace_gps_filter_state_.filtered_window) {
        centroid_x_m += sample.x_m;
        centroid_y_m += sample.y_m;
        centroid_altitude_m += sample.altitude_m;
        if (std::isfinite(sample.horizontal_stddev_m) && sample.horizontal_stddev_m > 0.0) {
            mean_horizontal_stddev_m += sample.horizontal_stddev_m;
            ++accuracy_sample_count;
        }
    }
    if (!auto_workspace_gps_filter_state_.filtered_window.empty()) {
        centroid_x_m /= static_cast<double>(auto_workspace_gps_filter_state_.filtered_window.size());
        centroid_y_m /= static_cast<double>(auto_workspace_gps_filter_state_.filtered_window.size());
        centroid_altitude_m /= static_cast<double>(auto_workspace_gps_filter_state_.filtered_window.size());
        for (const auto& sample : auto_workspace_gps_filter_state_.filtered_window) {
            cluster_radius_m = std::max(
                cluster_radius_m,
                std::hypot(sample.x_m - centroid_x_m, sample.y_m - centroid_y_m));
        }
    }
    if (accuracy_sample_count > 0) {
        mean_horizontal_stddev_m /= static_cast<double>(accuracy_sample_count);
    }

    auto_workspace_gps_state_.origin_stability_radius_m = cluster_radius_m;
    auto_workspace_gps_state_.origin_stability_samples =
        auto_workspace_gps_filter_state_.filtered_window.size();

    const bool enough_samples =
        auto_workspace_gps_filter_state_.filtered_window.size() >= kGpsOriginStabilityWindowSize;
    const bool accuracy_ok =
        accuracy_sample_count == 0 ||
        mean_horizontal_stddev_m <= kGpsOriginStableHorizontalStdDevM;
    const bool origin_stable =
        enough_samples &&
        accuracy_ok &&
        cluster_radius_m <= kGpsOriginStableRadiusM;
    auto_workspace_gps_state_.origin_stable = origin_stable;

    if (origin_stable && !auto_workspace_local_frame_state_.valid) {
        const auto [origin_latitude, origin_longitude] = latLonFromLocalPlanarOffsetMeters(
            auto_workspace_gps_filter_state_.anchor_latitude,
            auto_workspace_gps_filter_state_.anchor_longitude,
            centroid_x_m,
            centroid_y_m);
        auto_workspace_local_frame_state_.valid = true;
        auto_workspace_local_frame_state_.origin_latitude = origin_latitude;
        auto_workspace_local_frame_state_.origin_longitude = origin_longitude;
        auto_workspace_local_frame_state_.origin_altitude = centroid_altitude_m;
    }

    const bool imu_valid =
        auto_workspace_imu_state_.has_attitude &&
        auto_workspace_imu_state_.message_count > 0 &&
        (now - auto_workspace_imu_state_.last_message_steady_) <= kSensorFreshWindow &&
        std::isfinite(auto_workspace_imu_state_.yaw_deg);
    if (imu_valid) {
        if (!auto_workspace_gps_filter_state_.course_anchor_valid) {
            auto_workspace_gps_filter_state_.course_anchor_valid = true;
            auto_workspace_gps_filter_state_.course_anchor_x_m =
                auto_workspace_gps_filter_state_.filtered_x_m;
            auto_workspace_gps_filter_state_.course_anchor_y_m =
                auto_workspace_gps_filter_state_.filtered_y_m;
        } else {
            const double dx_m =
                auto_workspace_gps_filter_state_.filtered_x_m -
                auto_workspace_gps_filter_state_.course_anchor_x_m;
            const double dy_m =
                auto_workspace_gps_filter_state_.filtered_y_m -
                auto_workspace_gps_filter_state_.course_anchor_y_m;
            const double travel_m = std::hypot(dx_m, dy_m);
            if (travel_m >= kAutoWorkspaceHeadingCourseMinTravelM) {
                const double course_bearing_deg = planarBearingDeg(dx_m, dy_m);
                const double measured_offset_deg = normalizeHeadingDifferenceDeg(
                    course_bearing_deg - auto_workspace_imu_state_.yaw_deg);
                if (!auto_workspace_gps_filter_state_.heading_offset_valid) {
                    auto_workspace_gps_filter_state_.heading_offset_deg = measured_offset_deg;
                } else {
                    const double offset_delta_deg = normalizeHeadingDifferenceDeg(
                        measured_offset_deg - auto_workspace_gps_filter_state_.heading_offset_deg);
                    auto_workspace_gps_filter_state_.heading_offset_deg =
                        normalizeHeadingDifferenceDeg(
                            auto_workspace_gps_filter_state_.heading_offset_deg +
                            offset_delta_deg * kAutoWorkspaceHeadingOffsetEmaAlpha);
                }
                auto_workspace_gps_filter_state_.heading_offset_valid = true;
                auto_workspace_gps_filter_state_.heading_source = "gps_course";
                auto_workspace_gps_filter_state_.course_anchor_x_m =
                    auto_workspace_gps_filter_state_.filtered_x_m;
                auto_workspace_gps_filter_state_.course_anchor_y_m =
                    auto_workspace_gps_filter_state_.filtered_y_m;
            }
        }
    } else {
        auto_workspace_gps_filter_state_.course_anchor_valid = false;
    }

    const bool heading_valid =
        imu_valid && auto_workspace_gps_filter_state_.heading_offset_valid;
    auto_workspace_gps_state_.heading_valid = heading_valid;
    auto_workspace_gps_state_.heading_source =
        heading_valid ? auto_workspace_gps_filter_state_.heading_source : "";
    if (heading_valid) {
        auto_workspace_gps_state_.heading_deg =
            normalizeHeadingDifferenceDeg(
                auto_workspace_imu_state_.yaw_deg +
                auto_workspace_gps_filter_state_.heading_offset_deg);
    }

    const bool pose_heading_valid = imu_valid;
    if (pose_heading_valid) {
        const double pose_heading_deg =
            heading_valid ?
                auto_workspace_gps_state_.heading_deg :
                normalizeHeadingDifferenceDeg(auto_workspace_imu_state_.yaw_deg);
        const auto [gps_offset_x_m, gps_offset_y_m] = planarOffsetFromBodyFrameMeters(
            pose_heading_deg,
            kAutoWorkspaceGpsOffsetXM,
            kAutoWorkspaceGpsOffsetYM);
        const auto [base_link_latitude, base_link_longitude] =
            latLonFromLocalPlanarOffsetMeters(
                auto_workspace_gps_filter_state_.anchor_latitude,
                auto_workspace_gps_filter_state_.anchor_longitude,
                auto_workspace_gps_filter_state_.filtered_x_m - gps_offset_x_m,
                auto_workspace_gps_filter_state_.filtered_y_m - gps_offset_y_m);
        auto_workspace_gps_state_.base_link_valid = true;
        auto_workspace_gps_state_.base_link_latitude = base_link_latitude;
        auto_workspace_gps_state_.base_link_longitude = base_link_longitude;
    } else {
        auto_workspace_gps_state_.base_link_valid = false;
        auto_workspace_gps_state_.base_link_latitude = 0.0;
        auto_workspace_gps_state_.base_link_longitude = 0.0;
    }
}

void LogDashboardServer::onGpsFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    const auto now = std::chrono::steady_clock::now();
    const int status = msg ? msg->status.status : -1;
    const bool has_position =
        msg &&
        std::isfinite(msg->latitude) &&
        std::isfinite(msg->longitude);
    const bool valid_fix_position = has_position && status >= 0;
    std::lock_guard<std::mutex> lock(state_mutex_);
    gps_state_.message_count += 1;
    gps_state_.last_message_steady_ = now;
    gps_state_.status = status;
    gps_state_.has_position = has_position;
    if (has_position) {
        gps_state_.raw_latitude = msg->latitude;
        gps_state_.raw_longitude = msg->longitude;
        gps_state_.raw_altitude = std::isfinite(msg->altitude) ? msg->altitude : 0.0;
    }

    if (!valid_fix_position) {
        resetGpsFilterLocked();
        auto_workspace_gps_state_.message_count += 1;
        auto_workspace_gps_state_.last_message_steady_ = now;
        auto_workspace_gps_state_.status = status;
        auto_workspace_gps_state_.has_position = has_position;
        resetAutoWorkspaceGpsFilterLocked();
        return;
    }

    double horizontal_stddev_m = std::numeric_limits<double>::quiet_NaN();
    if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
        const double east_cov = msg->position_covariance[0];
        const double north_cov = msg->position_covariance[4];
        if (std::isfinite(east_cov) && east_cov >= 0.0 &&
            std::isfinite(north_cov) && north_cov >= 0.0) {
            horizontal_stddev_m = std::sqrt(std::max(east_cov, north_cov));
        }
    }

    updateFilteredGpsLocked(
        msg->latitude,
        msg->longitude,
        std::isfinite(msg->altitude) ? msg->altitude : 0.0,
        horizontal_stddev_m);

    auto_workspace_gps_state_.message_count += 1;
    auto_workspace_gps_state_.last_message_steady_ = now;
    auto_workspace_gps_state_.status = status;
    auto_workspace_gps_state_.has_position = has_position;
    auto_workspace_gps_state_.raw_latitude = msg->latitude;
    auto_workspace_gps_state_.raw_longitude = msg->longitude;
    auto_workspace_gps_state_.raw_altitude =
        std::isfinite(msg->altitude) ? msg->altitude : 0.0;
    updateAutoWorkspaceFilteredGpsLocked(
        msg->latitude,
        msg->longitude,
        std::isfinite(msg->altitude) ? msg->altitude : 0.0,
        horizontal_stddev_m,
        now);
}

void LogDashboardServer::onRgbYolo(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_rgb_yolo_payload_ = msg ? msg->data : std::string();
    latest_rgb_yolo_received_steady_ = std::chrono::steady_clock::now();
}

std::string LogDashboardServer::stateJson() const {
    const std::string rgb_yolo_payload = latestRgbYoloPayloadJson();
    const fs::path runtime_dir = fs::path(runtimeLogsDir());
    const ComponentProcessState stack_process =
        componentProcessState(runtime_dir / "pids" / "project_stack.pid", kStackProcessNeedle);
    const auto now_steady = std::chrono::steady_clock::now();
    const auto now_sys = std::chrono::system_clock::now();
    const bool startup_grace_active =
        stack_process.running &&
        (now_sys - stack_process.started_at) <= kSensorStartupGrace;

    DepthRuntimeState depth_state;
    LidarRuntimeState lidar_state;
    LidarRuntimeState auto_workspace_lidar_state;
    ImuRuntimeState imu_state;
    GpsRuntimeState gps_state;
    ImuRuntimeState auto_workspace_imu_state;
    GpsRuntimeState auto_workspace_gps_state;
    std::string live_rgb_yolo_payload;
    std::chrono::steady_clock::time_point rgb_yolo_received_steady;
    std::string active_workspace_mode;
    bool manual_workspace_working = false;
    bool avoidance_start_ack_pending = false;
    VehicleCommandState vehicle_command_state;
    AutoAvoidRuntimeState auto_avoid_runtime_state;
    AutoWorkspacePlanState auto_workspace_plan_state;
    AutoWorkspaceRuntimeState auto_workspace_runtime_state;
    AutoWorkspaceLocalFrameState auto_workspace_local_frame_state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        depth_state = depth_state_;
        lidar_state = lidar_state_;
        auto_workspace_lidar_state = auto_workspace_lidar_state_;
        imu_state = imu_state_;
        gps_state = gps_state_;
        auto_workspace_imu_state = auto_workspace_imu_state_;
        auto_workspace_gps_state = auto_workspace_gps_state_;
        live_rgb_yolo_payload = latest_rgb_yolo_payload_;
        rgb_yolo_received_steady = latest_rgb_yolo_received_steady_;
        active_workspace_mode = active_workspace_mode_;
        manual_workspace_working = manual_workspace_working_;
        avoidance_start_ack_pending = avoidance_start_ack_pending_;
        vehicle_command_state = vehicle_command_state_;
        auto_avoid_runtime_state = auto_avoid_runtime_state_;
        auto_workspace_plan_state = auto_workspace_plan_state_;
        auto_workspace_runtime_state = auto_workspace_runtime_state_;
        auto_workspace_local_frame_state = auto_workspace_local_frame_state_;
    }

    const bool common_auto_workspace = active_workspace_mode == "AUTO";
    const auto& common_lidar_state =
        common_auto_workspace ? auto_workspace_lidar_state : lidar_state;
    const auto& common_imu_state =
        common_auto_workspace ? auto_workspace_imu_state : imu_state;
    const auto& common_gps_state =
        common_auto_workspace ? auto_workspace_gps_state : gps_state;
    const bool common_origin_lock_ready =
        common_auto_workspace ?
            auto_workspace_gps_state.origin_stable :
            gps_state.origin_stable;
    const double common_origin_lock_radius_m =
        common_auto_workspace ?
            auto_workspace_gps_state.origin_stability_radius_m :
            gps_state.origin_stability_radius_m;
    const std::size_t common_origin_lock_samples =
        common_auto_workspace ?
            auto_workspace_gps_state.origin_stability_samples :
            gps_state.origin_stability_samples;
    const bool common_base_link_valid =
        common_auto_workspace && common_gps_state.base_link_valid;
    const double common_gps_latitude =
        common_base_link_valid ?
            common_gps_state.base_link_latitude :
            common_gps_state.latitude;
    const double common_gps_longitude =
        common_base_link_valid ?
            common_gps_state.base_link_longitude :
            common_gps_state.longitude;

    const auto classifyTopicState =
        [&](const auto& topic_state,
            const std::string& started_text,
            const std::string& error_text) {
            if (!stack_process.running) {
                return deviceStateJson("not_started", "未启动");
            }
            if (topic_state.message_count > 0 &&
                (now_steady - topic_state.last_message_steady_) <= kSensorFreshWindow) {
                return deviceStateJson("started", started_text);
            }
            if (startup_grace_active) {
                return deviceStateJson("not_started", "未启动");
            }
            return deviceStateJson("error", error_text);
        };

    const auto classifyGpsState = [&]() {
        if (!stack_process.running) {
            return deviceStateJson("not_started", "未启动");
        }
        if (common_gps_state.message_count > 0 &&
            (now_steady - common_gps_state.last_message_steady_) <= kSensorFreshWindow) {
            if (common_gps_state.status >= 0 && common_gps_state.filtered_valid) {
                if (common_auto_workspace && auto_workspace_local_frame_state.valid) {
                    return deviceStateJson("started", "已启动 / FIX 稳定");
                }
                if (!common_auto_workspace && gps_state.origin_stable) {
                    return deviceStateJson("started", "已启动 / FIX 稳定");
                }
                return deviceStateJson("started", "已启动 / FIX 收敛中");
            }
            if (common_gps_state.status >= 0) {
                return deviceStateJson("started", "已启动 / FIX");
            }
            return deviceStateJson("started", "已启动 / 无 FIX");
        }
        if (startup_grace_active) {
            return deviceStateJson("not_started", "未启动");
        }
        return deviceStateJson("error", "启动出错 / 未收到定位");
    };

    const auto classifyRgbYoloState = [&]() {
        if (!stack_process.running) {
            return deviceStateJson("not_started", "未启动");
        }

        const bool fresh_payload =
            !live_rgb_yolo_payload.empty() &&
            rgb_yolo_received_steady != std::chrono::steady_clock::time_point{} &&
            (now_steady - rgb_yolo_received_steady) <= kRgbYoloFreshWindow;
        if (!fresh_payload) {
            if (startup_grace_active) {
                return deviceStateJson("not_started", "未启动");
            }
            return deviceStateJson("error", "启动出错 / 未收到检测状态");
        }

        const bool online = jsonRawFieldTrue(live_rgb_yolo_payload, "online");
        const bool camera_open = jsonRawFieldTrue(live_rgb_yolo_payload, "camera_open");
        const bool model_ready = jsonRawFieldTrue(live_rgb_yolo_payload, "model_ready");
        const std::string error_text = extractJsonStringField(live_rgb_yolo_payload, "error");
        if (online && camera_open && model_ready) {
            return deviceStateJson("started", "已启动 / YOLO Online");
        }
        if (!error_text.empty()) {
            return deviceStateJson("error", "启动出错 / " + error_text);
        }
        if (!model_ready) {
            return deviceStateJson("error", "启动出错 / 模型未就绪");
        }
        if (!camera_open) {
            return deviceStateJson("error", "启动出错 / 相机未打开");
        }
        return deviceStateJson("error", "启动出错 / YOLO 未在线");
    };

    const auto classifyStm32State = [&]() {
        if (!stack_process.running) {
            return deviceStateJson("not_started", "未启动");
        }

        if (stm32_emergency_active_.load()) {
            return deviceStateJson("error", "急停中");
        }

        if (active_workspace_mode == "AVOIDANCE") {
            if (avoidance_start_ack_pending) {
                return deviceStateJson("started", "已启动 / 等待避障 start ACK");
            }
            if (auto_avoid_control_running_.load() || auto_avoid_stm32_drive_enabled_.load()) {
                return deviceStateJson("started", "已启动 / 避障工作区控制中");
            }
            return deviceStateJson("started", "已启动 / 避障工作区待命");
        }
        if (active_workspace_mode == "AUTO") {
            if (auto_workspace_runtime_state.awaiting_start_ack) {
                return deviceStateJson("started", "已启动 / 等待自动 start ACK");
            }
            if (auto_workspace_runtime_state.task_running || auto_workspace_control_running_.load()) {
                return deviceStateJson("started", "已启动 / 自动工作区控制中");
            }
            if (auto_workspace_plan_state.valid) {
                return deviceStateJson("started", "已启动 / 自动道路路径已规划");
            }
            return deviceStateJson("started", "已启动 / 自动工作区待命");
        }
        if (active_workspace_mode == "MANUAL") {
            if (manual_workspace_working) {
                return deviceStateJson("started", "已启动 / 手动工作区控制中");
            }
            return deviceStateJson("started", "已启动 / 手动工作区待命");
        }
        return deviceStateJson("started", "已启动 / 不做在线检测");
    };

    const bool gps_key_valid =
        stack_process.running &&
        common_gps_state.filtered_valid &&
        common_gps_state.status >= 0 &&
        common_gps_state.message_count > 0 &&
        (now_steady - common_gps_state.last_message_steady_) <= kSensorFreshWindow &&
        std::isfinite(common_gps_latitude) &&
        std::isfinite(common_gps_longitude);
    const bool imu_key_valid =
        stack_process.running &&
        common_imu_state.has_attitude &&
        common_imu_state.message_count > 0 &&
        (now_steady - common_imu_state.last_message_steady_) <= kSensorFreshWindow;
    const bool auto_workspace_gps_valid =
        stack_process.running &&
        auto_workspace_gps_state.filtered_valid &&
        auto_workspace_gps_state.status >= 0 &&
        auto_workspace_gps_state.message_count > 0 &&
        (now_steady - auto_workspace_gps_state.last_message_steady_) <= kSensorFreshWindow &&
        auto_workspace_gps_state.base_link_valid;
    const bool auto_workspace_imu_valid =
        stack_process.running &&
        auto_workspace_imu_state.has_attitude &&
        auto_workspace_imu_state.message_count > 0 &&
        (now_steady - auto_workspace_imu_state.last_message_steady_) <= kSensorFreshWindow;
    const bool auto_workspace_heading_valid =
        auto_workspace_imu_valid && auto_workspace_gps_state.heading_valid;
    const bool auto_workspace_origin_valid = auto_workspace_local_frame_state.valid;
    const bool auto_workspace_local_valid = auto_workspace_gps_valid && auto_workspace_origin_valid;
    double auto_workspace_current_x_m = auto_workspace_runtime_state.current_x_m;
    double auto_workspace_current_y_m = auto_workspace_runtime_state.current_y_m;
    if (auto_workspace_local_valid) {
        std::tie(auto_workspace_current_x_m, auto_workspace_current_y_m) =
            localPlanarOffsetMeters(
                auto_workspace_local_frame_state.origin_latitude,
                auto_workspace_local_frame_state.origin_longitude,
                auto_workspace_gps_state.base_link_latitude,
                auto_workspace_gps_state.base_link_longitude);
    }
    const auto auto_workspace_path_json = [&]() {
        std::ostringstream path_out;
        path_out << "[";
        for (std::size_t index = 0; index < auto_workspace_plan_state.preview_path.size(); ++index) {
            const auto& point = auto_workspace_plan_state.preview_path[index];
            if (index > 0) {
                path_out << ",";
            }
            path_out << "{"
                     << "\"latitude\":" << numberJson(point.latitude, 7) << ","
                     << "\"longitude\":" << numberJson(point.longitude, 7) << ","
                     << "\"x_m\":" << numberJson(point.x_m, 3) << ","
                     << "\"y_m\":" << numberJson(point.y_m, 3)
                     << "}";
        }
        path_out << "]";
        return path_out.str();
    };

    std::ostringstream out;
    out << "{"
        << "\"system\":{"
        << "\"running\":" << boolJson(stack_process.running) << ","
        << "\"mode\":\"" << jsonEscape(active_workspace_mode) << "\""
        << "},"
        << "\"devices\":{"
        << "\"gps\":" << classifyGpsState() << ","
        << "\"imu\":" << classifyTopicState(common_imu_state, "已启动 / IMU 数据正常", "启动出错 / 未收到 IMU 数据") << ","
        << "\"lidar\":" << classifyTopicState(common_lidar_state, "已启动 / 雷达扫描正常", "启动出错 / 未收到雷达数据") << ","
        << "\"depth_camera\":" << classifyTopicState(depth_state, "已启动 / 深度流正常", "启动出错 / 未收到深度图") << ","
        << "\"rgb_yolo\":" << classifyRgbYoloState() << ","
        << "\"stm32\":" << classifyStm32State()
        << "},"
        << "\"key_display\":{"
        << "\"gps\":{"
        << "\"valid\":" << boolJson(gps_key_valid) << ","
        << "\"fix\":" << boolJson(gps_key_valid && common_gps_state.status >= 0) << ","
        << "\"latitude\":" << (gps_key_valid ? numberJson(common_gps_latitude, 7) : "null") << ","
        << "\"longitude\":" << (gps_key_valid ? numberJson(common_gps_longitude, 7) : "null") << ","
        << "\"altitude_m\":" << (gps_key_valid ? numberJson(common_gps_state.altitude, 2) : "null") << ","
        << "\"raw_latitude\":" << (common_gps_state.has_position ? numberJson(common_gps_state.raw_latitude, 7) : "null") << ","
        << "\"raw_longitude\":" << (common_gps_state.has_position ? numberJson(common_gps_state.raw_longitude, 7) : "null") << ","
        << "\"raw_altitude_m\":" << (common_gps_state.has_position ? numberJson(common_gps_state.raw_altitude, 2) : "null") << ","
        << "\"horizontal_stddev_m\":" <<
            (gps_key_valid && std::isfinite(common_gps_state.horizontal_stddev_m) ?
                numberJson(common_gps_state.horizontal_stddev_m, 2) :
                "null") << ","
        << "\"origin_lock_ready\":" << boolJson(common_origin_lock_ready) << ","
        << "\"origin_lock_radius_m\":" << numberJson(common_origin_lock_radius_m, 3) << ","
        << "\"origin_lock_samples\":" << common_origin_lock_samples
        << "},"
        << "\"imu\":{"
        << "\"valid\":" << boolJson(imu_key_valid) << ","
        << "\"roll_deg\":" << (imu_key_valid ? numberJson(common_imu_state.roll_deg, 1) : "null") << ","
        << "\"pitch_deg\":" << (imu_key_valid ? numberJson(common_imu_state.pitch_deg, 1) : "null") << ","
        << "\"yaw_deg\":" << (imu_key_valid ? numberJson(common_imu_state.yaw_deg, 1) : "null")
        << "},"
        << "\"vehicle\":{"
        << "\"speed_valid\":" << boolJson(vehicle_command_state.has_speed) << ","
        << "\"speed\":" << (vehicle_command_state.has_speed ? std::to_string(vehicle_command_state.speed) : "null") << ","
        << "\"angle_valid\":" << boolJson(vehicle_command_state.has_angle) << ","
        << "\"angle\":" << (vehicle_command_state.has_angle ? std::to_string(vehicle_command_state.angle) : "null")
        << "},"
        << "\"auto_avoid\":{"
        << "\"active\":" << boolJson(auto_avoid_runtime_state.active) << ","
        << "\"awaiting_start_ack\":" << boolJson(avoidance_start_ack_pending) << ","
        << "\"has_decision\":" << boolJson(auto_avoid_runtime_state.has_decision) << ","
        << "\"control_cycle_id\":" << auto_avoid_runtime_state.last_control_cycle_id << ","
        << "\"snapshot_valid\":" << boolJson(auto_avoid_runtime_state.snapshot_valid) << ","
        << "\"lidar_valid\":" << boolJson(auto_avoid_runtime_state.lidar_valid) << ","
        << "\"imu_valid\":" << boolJson(auto_avoid_runtime_state.imu_valid) << ","
        << "\"mode\":\"" << jsonEscape(
                auto_avoid_runtime_state.has_decision ?
                    AutoAvoidController::motionModeName(auto_avoid_runtime_state.last_decision.mode) :
                    "") << "\","
        << "\"state\":\"" << jsonEscape(
                AutoAvoidController::avoidanceStageName(
                    auto_avoid_runtime_state.last_decision.debug.state)) << "\","
        << "\"direction\":\"" << jsonEscape(
                AutoAvoidController::turnDirectionName(
                    auto_avoid_runtime_state.last_decision.direction)) << "\","
        << "\"speed_cm_s\":" << (auto_avoid_runtime_state.has_decision ?
                std::to_string(auto_avoid_runtime_state.last_decision.speed_cm_s) : "null") << ","
        << "\"steering_encoder\":" << (auto_avoid_runtime_state.has_decision ?
                std::to_string(auto_avoid_runtime_state.last_decision.steering_encoder) : "null") << ","
        << "\"reason_code\":\"" << jsonEscape(
                AutoAvoidController::decisionReasonName(
                    auto_avoid_runtime_state.last_decision.reason_code)) << "\","
        << "\"fallback_reason\":\"" << jsonEscape(
                AutoAvoidController::fallbackReasonName(
                    auto_avoid_runtime_state.last_decision.fallback_reason)) << "\","
        << "\"apply_result\":\"" << jsonEscape(auto_avoid_runtime_state.last_apply_result) << "\","
        << "\"snapshot_fresh\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.snapshot_fresh) << ","
        << "\"control_snapshot_seq\":" <<
                auto_avoid_runtime_state.last_decision.debug.control_snapshot_seq << ","
        << "\"control_snapshot_stamp_ms\":" <<
                auto_avoid_runtime_state.last_decision.debug.control_snapshot_stamp_ms << ","
        << "\"lidar_snapshot_seq\":" <<
                auto_avoid_runtime_state.last_decision.debug.lidar_snapshot_seq << ","
        << "\"imu_snapshot_seq\":" <<
                auto_avoid_runtime_state.last_decision.debug.imu_snapshot_seq << ","
        << "\"lidar_snapshot_age_ms\":" << (
                std::isfinite(auto_avoid_runtime_state.last_decision.debug.lidar_snapshot_age_ms) ?
                    numberJson(
                        auto_avoid_runtime_state.last_decision.debug.lidar_snapshot_age_ms,
                        1) :
                    "null") << ","
        << "\"imu_snapshot_age_ms\":" << (
                std::isfinite(auto_avoid_runtime_state.last_decision.debug.imu_snapshot_age_ms) ?
                    numberJson(
                        auto_avoid_runtime_state.last_decision.debug.imu_snapshot_age_ms,
                        1) :
                    "null") << ","
        << "\"control_snapshot_consistent\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.control_snapshot_consistent) << ","
        << "\"control_snapshot_source\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.control_snapshot_source) << "\","
        << "\"control_snapshot_fresh\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.control_snapshot_fresh) << ","
        << "\"front_nearest_valid\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.front_nearest_valid) << ","
        << "\"front_nearest_m\":" << (
                auto_avoid_runtime_state.last_decision.debug.front_nearest_valid ?
                    numberJson(auto_avoid_runtime_state.last_decision.debug.front_nearest_m, 2) :
                    "null") << ","
        << "\"front_angle_deg\":" << (
                auto_avoid_runtime_state.last_decision.debug.front_nearest_valid ?
                    numberJson(auto_avoid_runtime_state.last_decision.debug.front_angle_deg, 1) :
                    "null") << ","
        << "\"front_support_points\":" <<
                auto_avoid_runtime_state.last_decision.debug.front_support_points << ","
        << "\"selected_front_cluster_id\":" << (
                auto_avoid_runtime_state.last_decision.debug.front_target_selection.valid ?
                    std::to_string(
                        auto_avoid_runtime_state.last_decision.debug.front_target_selection.selected_front_cluster_id) :
                    "null") << ","
        << "\"selected_front_cluster_score\":" <<
                (auto_avoid_runtime_state.last_decision.debug.front_target_selection.valid ?
                    numberJson(
                        auto_avoid_runtime_state.last_decision.debug.front_target_selection.selected_front_cluster_score,
                        2) :
                    "null") << ","
        << "\"selected_front_cluster_wall_like\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.front_target_selection.selected_front_cluster_wall_like) << ","
        << "\"selected_front_cluster_points\":" <<
                auto_avoid_runtime_state.last_decision.debug.front_target_selection.selected_front_cluster_points << ","
        << "\"selected_front_cluster_span_deg\":" <<
                (auto_avoid_runtime_state.last_decision.debug.front_target_selection.valid ?
                    numberJson(
                        auto_avoid_runtime_state.last_decision.debug.front_target_selection.selected_front_cluster_span_deg,
                        1) :
                    "null") << ","
        << "\"selected_front_cluster_median_range\":" <<
                (auto_avoid_runtime_state.last_decision.debug.front_target_selection.valid ?
                    numberJson(
                        auto_avoid_runtime_state.last_decision.debug.front_target_selection.selected_front_cluster_median_range,
                        2) :
                    "null") << ","
        << "\"selected_front_cluster_nearest_range\":" <<
                (auto_avoid_runtime_state.last_decision.debug.front_target_selection.valid ?
                    numberJson(
                        auto_avoid_runtime_state.last_decision.debug.front_target_selection.selected_front_cluster_nearest_range,
                        2) :
                    "null") << ","
        << "\"selected_front_cluster_is_discrete_primary\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.front_target_selection.selected_front_cluster_is_discrete_primary) << ","
        << "\"selected_front_cluster_is_wall_like\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.front_target_selection.selected_front_cluster_is_wall_like) << ","
        << "\"wall_like_cluster_suppressed\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.front_target_selection.wall_like_cluster_suppressed) << ","
        << "\"front_target_role\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.front_target_selection.front_target_role) << "\","
        << "\"raw_zone_from_discrete_target\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.front_target_selection.raw_zone_from_discrete_target) << ","
        << "\"wall_like_suppressed_from_zone\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.front_target_selection.wall_like_suppressed_from_zone) << ","
        << "\"front_target_selection_reason\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.front_target_selection.front_target_selection_reason) << "\","
        << "\"raw_zone_source\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.front_target_selection.raw_zone_source) << "\","
        << "\"raw_zone\":\"" << jsonEscape(
                Judgment::frontObstacleZoneName(
                    auto_avoid_runtime_state.last_decision.debug.raw_zone)) << "\","
        << "\"resolved_zone\":\"" << jsonEscape(
                Judgment::frontObstacleZoneName(
                    auto_avoid_runtime_state.last_decision.debug.resolved_zone)) << "\","
        << "\"spike_suppressed\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.spike_suppressed) << ","
        << "\"zone_stabilized\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.zone_stabilized) << ","
        << "\"zone_ambiguous\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.zone_ambiguous) << ","
        << "\"resolved_zone_override_active\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.resolved_zone_override_active) << ","
        << "\"resolved_zone_override_reason\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.resolved_zone_override_reason) << "\","
        << "\"committed_direction_override_active\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.committed_direction_override_active) << ","
        << "\"committed_direction_override_reason\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.committed_direction_override_reason) << "\","
        << "\"turning_to_clearance_candidate\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.turning_to_clearance_candidate) << ","
        << "\"turning_to_clearance_confirm_ticks\":" <<
                auto_avoid_runtime_state.last_decision.debug.turning_to_clearance_confirm_ticks << ","
        << "\"turning_to_clearance_reason\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.turning_to_clearance_reason) << "\","
        << "\"center_turn_decision_mode\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.center_turn_decision_mode) << "\","
        << "\"center_turn_bias_removed\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.center_turn_bias_removed) << ","
        << "\"center_turn_decision_reason\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.center_turn_decision_reason) << "\","
        << "\"active_avoidance_commit_present\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.active_avoidance_commit_present) << ","
        << "\"sector_buffer_active_continue\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.sector_buffer_active_continue) << ","
        << "\"sector_buffer_continue_active\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.sector_buffer_active_continue) << ","
        << "\"sector_buffer_observe_only\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.sector_buffer_observe_only) << ","
        << "\"sector_buffer_redirect_to_straight\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.sector_buffer_redirect_to_straight) << ","
        << "\"sector_buffer_redirect_reason\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.sector_buffer_redirect_reason) << "\","
        << "\"straight_drive_due_to_sector_buffer\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.straight_drive_due_to_sector_buffer) << ","
        << "\"sector_buffer_interrupt_reason\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.sector_buffer_interrupt_reason) << "\","
        << "\"boundary_stop\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.boundary_stop) << ","
        << "\"emergency_stop\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.emergency_stop) << ","
        << "\"replan_triggered\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.replan_triggered) << ","
        << "\"active_stage_priority_mode\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.active_stage_priority_mode) << "\","
        << "\"replan_override_active\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.replan_override_active) << ","
        << "\"replan_override_reason\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.replan_override_reason) << "\","
        << "\"active_stage_protection_active\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.active_stage_protection_active) << ","
        << "\"active_stage_protection_reason\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.active_stage_protection_reason) << "\","
        << "\"return_heading_protected\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.return_heading_protected) << ","
        << "\"return_heading_protect_ticks_remaining\":" <<
                auto_avoid_runtime_state.last_decision.debug.return_heading_protect_ticks_remaining << ","
        << "\"lateral_balance_active\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.lateral_balance_active) << ","
        << "\"lateral_balance_correction_deg\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.lateral_balance_correction_deg, 2) << ","
        << "\"wall_constraint_active\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.wall_constraint_active) << ","
        << "\"wall_constraint_side\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.wall_constraint_side) << "\","
        << "\"wall_constraint_correction_deg\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.wall_constraint_correction_deg, 2) << ","
        << "\"boundary_recovery_active\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.boundary_recovery_active) << ","
        << "\"boundary_recovery_side\":\"" << jsonEscape(
                AutoAvoidController::boundaryRiskSideName(
                    auto_avoid_runtime_state.last_decision.debug.boundary_recovery_side)) << "\","
        << "\"boundary_recovery_level\":\"" << jsonEscape(
                AutoAvoidController::boundaryRecoveryLevelName(
                    auto_avoid_runtime_state.last_decision.debug.boundary_recovery_level)) << "\","
        << "\"boundary_recovery_correction_deg\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.boundary_recovery_correction_deg, 2) << ","
        << "\"boundary_recovery_limited_by_tail\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.boundary_recovery_limited_by_tail) << ","
        << "\"boundary_override_active\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.boundary_override_active) << ","
        << "\"boundary_override_reason\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.boundary_override_reason) << "\","
        << "\"boundary_override_reduced_main_steering\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.boundary_override_reduced_main_steering) << ","
        << "\"boundary_override_reduced_by_deg\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.boundary_override_reduced_by_deg, 2) << ","
        << "\"boundary_risk_left\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.boundary_risk_left, 2) << ","
        << "\"boundary_risk_right\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.boundary_risk_right, 2) << ","
        << "\"boundary_risk_delta\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.boundary_risk_delta, 2) << ","
        << "\"boundary_recovery_and_path_aligned\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.boundary_recovery_and_path_aligned) << ","
        << "\"boundary_recovery_and_path_conflict\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.boundary_recovery_and_path_conflict) << ","
        << "\"main_steering_deg\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.main_steering_deg, 2) << ","
        << "\"main_steering_source\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.main_steering_source) << "\","
        << "\"boundary_override_applied\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.boundary_override_applied) << ","
        << "\"boundary_override_delta_deg\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.boundary_override_delta_deg, 2) << ","
        << "\"boundary_recovery_applied\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.boundary_recovery_applied) << ","
        << "\"boundary_recovery_delta_deg\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.boundary_recovery_delta_deg, 2) << ","
        << "\"smoothed_steering_deg\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.smoothed_steering_deg, 2) << ","
        << "\"guarded_steering_deg\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.guarded_steering_deg, 2) << ","
        << "\"final_encoder_command\":" <<
                auto_avoid_runtime_state.last_decision.debug.final_encoder_command << ","
        << "\"steering_direction_consistent\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.steering_direction_consistent) << ","
        << "\"steering_direction_conflict_reason\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.steering_direction_conflict_reason) << "\","
        << "\"path_reference_valid\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.path_reference_valid) << ","
        << "\"reference_yaw_deg\":" << (
                auto_avoid_runtime_state.last_decision.debug.path_reference_valid ?
                    numberJson(auto_avoid_runtime_state.last_decision.debug.reference_yaw_deg, 1) :
                    "null") << ","
        << "\"reference_side_balance\":" << (
                auto_avoid_runtime_state.last_decision.debug.path_reference_valid ?
                    numberJson(auto_avoid_runtime_state.last_decision.debug.reference_side_balance, 2) :
                    "null") << ","
        << "\"reference_left_distance_m\":" << (
                auto_avoid_runtime_state.last_decision.debug.path_reference_valid ?
                    numberJson(auto_avoid_runtime_state.last_decision.debug.reference_left_distance_m, 2) :
                    "null") << ","
        << "\"reference_right_distance_m\":" << (
                auto_avoid_runtime_state.last_decision.debug.path_reference_valid ?
                    numberJson(auto_avoid_runtime_state.last_decision.debug.reference_right_distance_m, 2) :
                    "null") << ","
        << "\"path_reference_captured_ms\":" <<
                auto_avoid_runtime_state.last_decision.debug.path_reference_captured_ms << ","
        << "\"path_reference_captured_stage\":\"" << jsonEscape(
                AutoAvoidController::avoidanceStageName(
                    auto_avoid_runtime_state.last_decision.debug.path_reference_captured_stage)) << "\","
        << "\"path_reference_captured_this_cycle\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.path_reference_captured_this_cycle) << ","
        << "\"path_reference_clear_reason\":\"" << jsonEscape(
                AutoAvoidController::pathReferenceClearReasonName(
                    auto_avoid_runtime_state.last_decision.debug.path_reference_clear_reason)) << "\","
        << "\"return_to_path_active\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.return_to_path_active) << ","
        << "\"return_to_path_phase\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.return_to_path_phase) << "\","
        << "\"return_to_path_fast_recenter_active\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.return_to_path_fast_recenter_active) << ","
        << "\"return_to_path_settling_active\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.return_to_path_settling_active) << ","
        << "\"return_to_path_can_settle\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.return_to_path_can_settle) << ","
        << "\"return_to_path_blocked_reason\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug.return_to_path_blocked_reason) << "\","
        << "\"yaw_recovery_correction_deg\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.yaw_recovery_correction_deg, 2) << ","
        << "\"yaw_recovery_dynamic_gain\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.yaw_recovery_dynamic_gain, 3) << ","
        << "\"yaw_recovery_retained_by_path\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.yaw_recovery_retained_by_path) << ","
        << "\"yaw_recovery_final_deg\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.yaw_recovery_final_deg, 2) << ","
        << "\"path_recovery_correction_deg\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.path_recovery_correction_deg, 2) << ","
        << "\"path_recovery_balance_error\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.path_recovery_balance_error, 3) << ","
        << "\"path_recovery_dynamic_gain\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.path_recovery_dynamic_gain, 3) << ","
        << "\"path_recovery_fast_recenter_boost\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.path_recovery_fast_recenter_boost, 3) << ","
        << "\"path_recovery_final_deg\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.path_recovery_final_deg, 2) << ","
        << "\"combined_return_correction_deg\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.combined_return_correction_deg, 2) << ","
        << "\"combined_return_correction_limited_by_tail\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.combined_return_correction_limited_by_tail) << ","
        << "\"return_to_path_progress_score\":" <<
                numberJson(auto_avoid_runtime_state.last_decision.debug.return_to_path_progress_score, 3) << ","
        << "\"return_to_path_near_reference\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.return_to_path_near_reference) << ","
        << "\"tail_clearance_complete\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.tail_clearance_complete) << ","
        << "\"tail_clearance_blocking\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.tail_clearance_blocking) << ","
        << "\"path_recovery_ready\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.path_recovery_ready) << ","
        << "\"path_recovery_settled\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.path_recovery_settled) << ","
        << "\"exit_to_idle_ready\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.exit_to_idle_ready) << ","
        << "\"used_imu_heading\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.used_imu_heading) << ","
        << "\"used_encoder_fallback\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.used_encoder_fallback) << ","
        << "\"encoder_fallback_kind\":\"" << jsonEscape(
                AutoAvoidController::encoderFallbackKindName(
                    auto_avoid_runtime_state.last_decision.debug.encoder_fallback_kind)) << "\","
        << "\"target_yaw_valid\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.target_yaw_valid) << ","
        << "\"target_yaw_deg\":" << (
                auto_avoid_runtime_state.last_decision.debug.target_yaw_valid ?
                    numberJson(auto_avoid_runtime_state.last_decision.debug.target_yaw_deg, 1) :
                    "null") << ","
        << "\"target_yaw_locked_ms\":" <<
                auto_avoid_runtime_state.last_decision.debug.target_yaw_locked_ms << ","
        << "\"target_yaw_locked_by_stage\":\"" << jsonEscape(
                AutoAvoidController::avoidanceStageName(
                    auto_avoid_runtime_state.last_decision.debug.target_yaw_locked_by_stage)) << "\","
        << "\"target_yaw_locked_this_cycle\":" << boolJson(
                auto_avoid_runtime_state.last_decision.debug.target_yaw_locked_this_cycle) << ","
        << "\"target_yaw_clear_reason\":\"" << jsonEscape(
                AutoAvoidController::targetYawClearReasonName(
                    auto_avoid_runtime_state.last_decision.debug.target_yaw_clear_reason)) << "\","
        << "\"debug_text\":\"" << jsonEscape(
                auto_avoid_runtime_state.last_decision.debug_text) << "\""
        << "},"
        << "\"auto_workspace\":{"
        << "\"plan_ready\":" << boolJson(auto_workspace_plan_state.valid) << ","
        << "\"awaiting_start_ack\":" << boolJson(auto_workspace_runtime_state.awaiting_start_ack) << ","
        << "\"task_running\":" << boolJson(auto_workspace_runtime_state.task_running) << ","
        << "\"gps_valid\":" << boolJson(auto_workspace_gps_valid) << ","
        << "\"imu_valid\":" << boolJson(auto_workspace_imu_valid) << ","
        << "\"heading_valid\":" << boolJson(auto_workspace_heading_valid) << ","
        << "\"local_frame_valid\":" << boolJson(auto_workspace_local_valid) << ","
        << "\"origin_valid\":" << boolJson(auto_workspace_origin_valid) << ","
        << "\"avoidance_active\":" << boolJson(auto_workspace_runtime_state.avoidance_active) << ","
        << "\"target_reached\":" << boolJson(auto_workspace_runtime_state.target_reached) << ","
        << "\"phase\":\"" << jsonEscape(auto_workspace_runtime_state.phase) << "\","
        << "\"message\":\"" << jsonEscape(auto_workspace_runtime_state.message) << "\","
        << "\"planner\":\"" << jsonEscape(auto_workspace_plan_state.planner) << "\","
        << "\"route_provider\":\"" << jsonEscape(auto_workspace_plan_state.route_provider) << "\","
        << "\"heading_source\":\"" << jsonEscape(auto_workspace_runtime_state.heading_source) << "\","
        << "\"control_cycle_id\":" << auto_workspace_runtime_state.last_control_cycle_id << ","
        << "\"active_path_index\":" << auto_workspace_runtime_state.active_path_index << ","
        << "\"current_latitude\":" << (
                auto_workspace_gps_valid ?
                    numberJson(auto_workspace_gps_state.base_link_latitude, 7) :
                    "null") << ","
        << "\"current_longitude\":" << (
                auto_workspace_gps_valid ?
                    numberJson(auto_workspace_gps_state.base_link_longitude, 7) :
                    "null") << ","
        << "\"current_x_m\":" << numberJson(auto_workspace_current_x_m, 3) << ","
        << "\"current_y_m\":" << numberJson(auto_workspace_current_y_m, 3) << ","
        << "\"current_yaw_deg\":" << (
                auto_workspace_heading_valid ?
                    numberJson(auto_workspace_gps_state.heading_deg, 2) :
                    "null") << ","
        << "\"origin_latitude\":" << (
                auto_workspace_origin_valid ?
                    numberJson(auto_workspace_local_frame_state.origin_latitude, 7) :
                    "null") << ","
        << "\"origin_longitude\":" << (
                auto_workspace_origin_valid ?
                    numberJson(auto_workspace_local_frame_state.origin_longitude, 7) :
                    "null") << ","
        << "\"origin_altitude_m\":" << (
                auto_workspace_origin_valid ?
                    numberJson(auto_workspace_local_frame_state.origin_altitude, 2) :
                    "null") << ","
        << "\"origin_lock_ready\":" << boolJson(auto_workspace_gps_state.origin_stable) << ","
        << "\"origin_lock_radius_m\":" << numberJson(auto_workspace_gps_state.origin_stability_radius_m, 3) << ","
        << "\"origin_lock_samples\":" << auto_workspace_gps_state.origin_stability_samples << ","
        << "\"destination_latitude\":" << (auto_workspace_plan_state.valid ? numberJson(auto_workspace_plan_state.destination_latitude, 7) : "null") << ","
        << "\"destination_longitude\":" << (auto_workspace_plan_state.valid ? numberJson(auto_workspace_plan_state.destination_longitude, 7) : "null") << ","
        << "\"destination_x_m\":" << numberJson(auto_workspace_runtime_state.destination_x_m, 3) << ","
        << "\"destination_y_m\":" << numberJson(auto_workspace_runtime_state.destination_y_m, 3) << ","
        << "\"remaining_distance_m\":" << numberJson(auto_workspace_runtime_state.remaining_distance_m, 3) << ","
        << "\"target_bearing_deg\":" << numberJson(auto_workspace_runtime_state.target_bearing_deg, 3) << ","
        << "\"heading_error_deg\":" << numberJson(auto_workspace_runtime_state.heading_error_deg, 3) << ","
        << "\"command_speed_cm_s\":" << auto_workspace_runtime_state.command_speed_cm_s << ","
        << "\"command_steering_encoder\":" << auto_workspace_runtime_state.command_steering_encoder << ","
        << "\"preview_distance_m\":" << (auto_workspace_plan_state.valid ? numberJson(auto_workspace_plan_state.preview_distance_m, 2) : "null") << ","
        << "\"preview_duration_s\":" << (auto_workspace_plan_state.valid ? numberJson(auto_workspace_plan_state.preview_duration_s, 1) : "null") << ","
        << "\"preview_bearing_deg\":" << (auto_workspace_plan_state.valid ? numberJson(auto_workspace_plan_state.preview_bearing_deg, 2) : "null") << ","
        << "\"path\":" << (auto_workspace_plan_state.valid ? auto_workspace_path_json() : "[]")
        << "}"
        << "},"
        << "\"rgb_yolo_payload\":\"" << jsonEscape(rgb_yolo_payload) << "\""
        << "}";
    return out.str();
}

std::string LogDashboardServer::latestRgbYoloPayloadJson() const {
    std::string payload;
    std::chrono::steady_clock::time_point received_steady;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        payload = latest_rgb_yolo_payload_;
        received_steady = latest_rgb_yolo_received_steady_;
    }

    const bool has_live_payload =
        !payload.empty() &&
        received_steady != std::chrono::steady_clock::time_point{} &&
        (std::chrono::steady_clock::now() - received_steady) <= kRgbYoloFreshWindow;
    if (has_live_payload) {
        const std::string live_photo_url = extractJsonStringField(payload, "last_photo_url");
        if (live_photo_url.empty()) {
            const std::string fallback = latestSavedRgbYoloPayloadJson();
            if (!fallback.empty()) {
                return mergeLivePayloadWithSavedPhoto(payload, fallback);
            }
        }
        return payload;
    }

    const std::string fallback = latestSavedRgbYoloPayloadJson();
    if (!fallback.empty()) {
        return fallback;
    }

    return payload;
}

std::string LogDashboardServer::latestSavedRgbYoloPayloadJson() const {
    const fs::path storage_root(storage_root_);
    const fs::path photos_dir = storage_root / "photos";
    const auto latest_photo_path = latestSavedPhotoPath(photos_dir);
    if (!latest_photo_path.has_value()) {
        return "";
    }

    const std::string metadata_json = readFileText(metadataPathForPhoto(*latest_photo_path));
    if (!metadata_json.empty()) {
        return metadata_json;
    }

    const std::string photo_url =
        mountedUrlFor(storage_root, *latest_photo_path, kAllfileMountPrefix);
    const std::string photo_time = inferPhotoTime(*latest_photo_path);
    const std::string class_name = inferPhotoClassName(*latest_photo_path);

    std::ostringstream out;
    out << "{"
        << "\"node\":\"rgb_yolo_detector\","
        << "\"online\":false,"
        << "\"camera_open\":false,"
        << "\"model_ready\":false,"
        << "\"history_photo\":true,"
        << "\"source\":\"\","
        << "\"detected\":" << boolJson(!class_name.empty()) << ","
        << "\"detections_count\":" << (!class_name.empty() ? 1 : 0) << ","
        << "\"class_name\":\"" << jsonEscape(class_name) << "\","
        << "\"confidence\":0.000,"
        << "\"bbox_x\":0,"
        << "\"bbox_y\":0,"
        << "\"bbox_w\":0,"
        << "\"bbox_h\":0,"
        << "\"frame_width\":0,"
        << "\"frame_height\":0,"
        << "\"photo_output_dir\":\"" << jsonEscape(photos_dir.string()) << "\","
        << "\"last_photo_path\":\"" << jsonEscape(latest_photo_path->string()) << "\","
        << "\"last_photo_url\":\"" << jsonEscape(photo_url) << "\","
        << "\"last_photo_time\":\"" << jsonEscape(photo_time) << "\","
        << "\"last_frame_time\":\"" << jsonEscape(photo_time) << "\","
        << "\"last_detection_time\":\"" << jsonEscape(photo_time) << "\","
        << "\"error\":\"\","
        << "\"report\":\"history photo fallback\""
        << "}";
    return out.str();
}
