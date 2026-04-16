#pragma once

#include <chrono>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>

namespace bishe::tools
{

struct GruDataRecorderConfig
{
    bool enable_gru_data_record = true;
    bool record_only_in_manual_mode = true;
    double record_hz = 5.0;
    std::string csv_output_dir = "/home/mao/use/project/Log/gru_dataset";
    double steer_positive_threshold = 30.0;
    double steer_negative_threshold = -30.0;
    int stop_speed_threshold = 5;
    int slowdown_delta_threshold = 10;
};

GruDataRecorderConfig defaultGruDataRecorderConfig();

struct GruDataSample
{
    std::int64_t sample_time_ns = 0;
    std::string mode;
    bool manual_enabled = false;
    bool auto_enabled = false;
    bool stop_enabled = false;

    bool depth_valid = false;
    double depth_nearest_m = 0.0;
    std::string depth_zone;
    bool depth_obstacle_ratio_valid = false;
    double depth_obstacle_ratio = 0.0;
    int depth_valid_pixels = 0;
    int depth_obstacle_pixels = 0;

    bool lidar_valid = false;
    double lidar_nearest_m = 0.0;
    bool lidar_angle_valid = false;
    double lidar_nearest_angle_deg = 0.0;
    int lidar_valid_points = 0;

    bool fused_valid = false;
    std::string fused_source;
    double fused_nearest_distance_m = 0.0;
    bool fused_angle_valid = false;
    double fused_nearest_angle_deg = 0.0;
    std::string fused_zone;

    bool imu_valid = false;
    double imu_roll_deg = 0.0;
    double imu_pitch_deg = 0.0;
    double imu_yaw_deg = 0.0;

    bool speed_cmd_valid = false;
    int speed_cmd = 0;
    bool angle_cmd_valid = false;
    int angle_cmd = 0;
};

class GruDataRecorder
{
public:
    explicit GruDataRecorder(
        GruDataRecorderConfig config = defaultGruDataRecorderConfig());
    ~GruDataRecorder();

    bool enabled() const;
    double recordHz() const;
    const GruDataRecorderConfig & config() const;

    bool startNewSession();
    void record(const GruDataSample & sample);

private:
    bool shouldRecordLocked(const GruDataSample & sample, std::chrono::steady_clock::time_point now) const;
    bool ensureFileLocked();
    bool openNewFileLocked();
    std::string deriveTeacherActionLocked(const GruDataSample & sample) const;
    void updateHistoryLocked(const GruDataSample & sample);

private:
    GruDataRecorderConfig config_;
    mutable std::mutex mutex_;
    std::ofstream output_;
    std::string csv_path_;
    std::chrono::steady_clock::time_point last_record_steady_ {};
    bool previous_speed_valid_ = false;
    int previous_speed_cmd_ = 0;
};

}  // namespace bishe::tools
