#include "tools/gru_data_recorder.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <initializer_list>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace bishe::tools
{

namespace
{

std::string envValue(std::initializer_list<const char *> keys)
{
    for (const char * key : keys) {
        if (const char * value = std::getenv(key)) {
            if (*value != '\0') {
                return value;
            }
        }
    }
    return "";
}

bool parseBool(const std::string & value, bool fallback)
{
    std::string normalized;
    normalized.reserve(value.size());
    for (char ch : value) {
        normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    }
    if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on") {
        return true;
    }
    if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off") {
        return false;
    }
    return fallback;
}

bool envBool(std::initializer_list<const char *> keys, bool fallback)
{
    const std::string value = envValue(keys);
    return value.empty() ? fallback : parseBool(value, fallback);
}

double envDouble(std::initializer_list<const char *> keys, double fallback)
{
    const std::string value = envValue(keys);
    if (value.empty()) {
        return fallback;
    }
    try {
        return std::stod(value);
    } catch (...) {
        return fallback;
    }
}

int envInt(std::initializer_list<const char *> keys, int fallback)
{
    const std::string value = envValue(keys);
    if (value.empty()) {
        return fallback;
    }
    try {
        return std::stoi(value);
    } catch (...) {
        return fallback;
    }
}

std::string envString(std::initializer_list<const char *> keys, const std::string & fallback)
{
    const std::string value = envValue(keys);
    return value.empty() ? fallback : value;
}

std::string compactTimestamp()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm tm {};
    localtime_r(&time, &tm);
    std::ostringstream out;
    out << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return out.str();
}

std::filesystem::path uniqueCsvPath(const std::filesystem::path & output_dir)
{
    const std::string stamp = compactTimestamp();
    std::filesystem::path csv_path = output_dir / ("gru_dataset_" + stamp + ".csv");
    for (int index = 1; std::filesystem::exists(csv_path); ++index) {
        std::ostringstream suffix;
        suffix << "_";
        suffix << std::setw(3) << std::setfill('0') << index;
        csv_path = output_dir / ("gru_dataset_" + stamp + suffix.str() + ".csv");
    }
    return csv_path;
}

std::string csvEscape(const std::string & value)
{
    bool needs_quotes = false;
    for (char ch : value) {
        if (ch == ',' || ch == '"' || ch == '\n' || ch == '\r') {
            needs_quotes = true;
            break;
        }
    }
    if (!needs_quotes) {
        return value;
    }

    std::string escaped;
    escaped.reserve(value.size() + 2);
    escaped.push_back('"');
    for (char ch : value) {
        if (ch == '"') {
            escaped += "\"\"";
        } else {
            escaped.push_back(ch);
        }
    }
    escaped.push_back('"');
    return escaped;
}

std::string boolField(bool value)
{
    return value ? "1" : "0";
}

std::string intField(bool valid, int value)
{
    return valid ? std::to_string(value) : std::string();
}

std::string doubleField(bool valid, double value, int precision = 4)
{
    if (!valid) {
        return "";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

void writeHeader(std::ofstream & output)
{
    output
        << "sample_time_ns,"
        << "mode,"
        << "manual_enabled,"
        << "auto_enabled,"
        << "stop_enabled,"
        << "depth_valid,"
        << "depth_nearest_m,"
        << "depth_zone,"
        << "depth_obstacle_ratio,"
        << "depth_valid_pixels,"
        << "depth_obstacle_pixels,"
        << "lidar_valid,"
        << "lidar_nearest_m,"
        << "lidar_nearest_angle_deg,"
        << "lidar_valid_points,"
        << "fused_valid,"
        << "fused_source,"
        << "fused_nearest_distance_m,"
        << "fused_nearest_angle_deg,"
        << "fused_zone,"
        << "imu_roll_deg,"
        << "imu_pitch_deg,"
        << "imu_yaw_deg,"
        << "speed_cmd,"
        << "angle_cmd,"
        << "speed_cmd_valid,"
        << "angle_cmd_valid,"
        << "teacher_action\n";
}

void writeRow(std::ofstream & output, const GruDataSample & sample, const std::string & teacher_action)
{
    std::vector<std::string> fields;
    fields.reserve(28);
    fields.push_back(std::to_string(sample.sample_time_ns));
    fields.push_back(sample.mode);
    fields.push_back(boolField(sample.manual_enabled));
    fields.push_back(boolField(sample.auto_enabled));
    fields.push_back(boolField(sample.stop_enabled));
    fields.push_back(boolField(sample.depth_valid));
    fields.push_back(doubleField(sample.depth_valid, sample.depth_nearest_m));
    fields.push_back(sample.depth_valid ? sample.depth_zone : std::string());
    fields.push_back(doubleField(sample.depth_obstacle_ratio_valid, sample.depth_obstacle_ratio));
    fields.push_back(intField(sample.depth_valid, sample.depth_valid_pixels));
    fields.push_back(intField(sample.depth_valid, sample.depth_obstacle_pixels));
    fields.push_back(boolField(sample.lidar_valid));
    fields.push_back(doubleField(sample.lidar_valid, sample.lidar_nearest_m));
    fields.push_back(doubleField(sample.lidar_angle_valid, sample.lidar_nearest_angle_deg));
    fields.push_back(intField(sample.lidar_valid, sample.lidar_valid_points));
    fields.push_back(boolField(sample.fused_valid));
    fields.push_back(sample.fused_valid ? sample.fused_source : std::string());
    fields.push_back(doubleField(sample.fused_valid, sample.fused_nearest_distance_m));
    fields.push_back(doubleField(sample.fused_angle_valid, sample.fused_nearest_angle_deg));
    fields.push_back(sample.fused_valid ? sample.fused_zone : std::string());
    fields.push_back(doubleField(sample.imu_valid, sample.imu_roll_deg));
    fields.push_back(doubleField(sample.imu_valid, sample.imu_pitch_deg));
    fields.push_back(doubleField(sample.imu_valid, sample.imu_yaw_deg));
    fields.push_back(intField(sample.speed_cmd_valid, sample.speed_cmd));
    fields.push_back(intField(sample.angle_cmd_valid, sample.angle_cmd));
    fields.push_back(boolField(sample.speed_cmd_valid));
    fields.push_back(boolField(sample.angle_cmd_valid));
    fields.push_back(teacher_action);

    for (std::size_t i = 0; i < fields.size(); ++i) {
        if (i > 0) {
            output << ",";
        }
        output << csvEscape(fields[i]);
    }
    output << "\n";
}

}  // namespace

GruDataRecorderConfig defaultGruDataRecorderConfig()
{
    GruDataRecorderConfig config;
    config.enable_gru_data_record = envBool(
        {"PROJECT_ENABLE_GRU_DATA_RECORD", "enable_gru_data_record"},
        config.enable_gru_data_record);
    config.record_only_in_manual_mode = envBool(
        {"PROJECT_GRU_RECORD_ONLY_MANUAL", "record_only_in_manual_mode"},
        config.record_only_in_manual_mode);
    config.record_hz = envDouble(
        {"PROJECT_GRU_RECORD_HZ", "record_hz"},
        config.record_hz);
    config.csv_output_dir = envString(
        {"PROJECT_GRU_CSV_OUTPUT_DIR", "csv_output_dir"},
        config.csv_output_dir);
    config.steer_positive_threshold = envDouble(
        {"PROJECT_GRU_STEER_POSITIVE_THRESHOLD", "steer_positive_threshold"},
        config.steer_positive_threshold);
    config.steer_negative_threshold = envDouble(
        {"PROJECT_GRU_STEER_NEGATIVE_THRESHOLD", "steer_negative_threshold"},
        config.steer_negative_threshold);
    config.stop_speed_threshold = envInt(
        {"PROJECT_GRU_STOP_SPEED_THRESHOLD", "stop_speed_threshold"},
        config.stop_speed_threshold);
    config.slowdown_delta_threshold = envInt(
        {"PROJECT_GRU_SLOWDOWN_DELTA_THRESHOLD", "slowdown_delta_threshold"},
        config.slowdown_delta_threshold);

    config.record_hz = std::clamp(config.record_hz, 0.1, 50.0);
    if (config.steer_negative_threshold > 0.0) {
        config.steer_negative_threshold = -config.steer_negative_threshold;
    }
    config.stop_speed_threshold = std::max(config.stop_speed_threshold, 0);
    config.slowdown_delta_threshold = std::max(config.slowdown_delta_threshold, 0);
    return config;
}

GruDataRecorder::GruDataRecorder(GruDataRecorderConfig config)
    : config_(std::move(config))
{
    if (config_.enable_gru_data_record) {
        std::error_code error;
        std::filesystem::create_directories(config_.csv_output_dir, error);
    }
}

GruDataRecorder::~GruDataRecorder()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (output_.is_open()) {
        output_.flush();
        output_.close();
    }
}

bool GruDataRecorder::enabled() const
{
    return config_.enable_gru_data_record;
}

double GruDataRecorder::recordHz() const
{
    return config_.record_hz;
}

const GruDataRecorderConfig & GruDataRecorder::config() const
{
    return config_;
}

bool GruDataRecorder::startNewSession()
{
    if (!config_.enable_gru_data_record) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (output_.is_open()) {
        output_.flush();
        output_.close();
    }
    csv_path_.clear();
    last_record_steady_ = {};
    previous_speed_valid_ = false;
    previous_speed_cmd_ = 0;
    return openNewFileLocked();
}

void GruDataRecorder::record(const GruDataSample & sample)
{
    if (!config_.enable_gru_data_record) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!shouldRecordLocked(sample, now)) {
        return;
    }
    if (!ensureFileLocked()) {
        return;
    }

    const std::string teacher_action = deriveTeacherActionLocked(sample);
    writeRow(output_, sample, teacher_action);
    output_.flush();
    last_record_steady_ = now;
    updateHistoryLocked(sample);
}

bool GruDataRecorder::shouldRecordLocked(
    const GruDataSample & sample,
    std::chrono::steady_clock::time_point now) const
{
    if (config_.record_only_in_manual_mode && !sample.manual_enabled) {
        return false;
    }

    if (last_record_steady_ == std::chrono::steady_clock::time_point {}) {
        return true;
    }

    const auto min_interval = std::chrono::duration<double>(1.0 / config_.record_hz);
    return (now - last_record_steady_) >= min_interval;
}

bool GruDataRecorder::ensureFileLocked()
{
    if (output_.is_open()) {
        return true;
    }

    return openNewFileLocked();
}

bool GruDataRecorder::openNewFileLocked()
{
    std::error_code error;
    std::filesystem::create_directories(config_.csv_output_dir, error);
    if (error) {
        return false;
    }

    const std::filesystem::path output_dir(config_.csv_output_dir);
    const std::filesystem::path csv_path = uniqueCsvPath(output_dir);
    output_.open(csv_path, std::ios::out | std::ios::app);
    if (!output_.is_open()) {
        return false;
    }

    csv_path_ = csv_path.string();
    writeHeader(output_);
    output_.flush();
    return true;
}

std::string GruDataRecorder::deriveTeacherActionLocked(const GruDataSample & sample) const
{
    if (sample.stop_enabled || (sample.mode == "STOP" && !sample.speed_cmd_valid)) {
        return "STOP";
    }
    if (!sample.manual_enabled) {
        return "UNKNOWN";
    }
    if (sample.speed_cmd_valid && sample.speed_cmd <= config_.stop_speed_threshold) {
        return "STOP";
    }
    if (sample.speed_cmd_valid &&
        previous_speed_valid_ &&
        (previous_speed_cmd_ - sample.speed_cmd) >= config_.slowdown_delta_threshold) {
        return "SLOW_DOWN";
    }
    if (sample.angle_cmd_valid && sample.angle_cmd >= config_.steer_positive_threshold) {
        return "POSITIVE_AVOID";
    }
    if (sample.angle_cmd_valid && sample.angle_cmd <= config_.steer_negative_threshold) {
        return "NEGATIVE_AVOID";
    }
    return "KEEP";
}

void GruDataRecorder::updateHistoryLocked(const GruDataSample & sample)
{
    if (sample.speed_cmd_valid) {
        previous_speed_valid_ = true;
        previous_speed_cmd_ = sample.speed_cmd;
    }
}

}  // namespace bishe::tools
