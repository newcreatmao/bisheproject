#ifndef BISHE_VEHICLE_STATE_HPP
#define BISHE_VEHICLE_STATE_HPP

#include <cstddef>
#include <cstdint>
#include "common/sensor_types.hpp"

namespace bishe
{
namespace vehicle
{

struct ObstacleState
{
    // LiDAR observation only: geometry and occupancy information, no steering
    // or motion recommendation.
    bool detected = false;
    float min_distance = 999.0f;
    float angle_deg = 0.0f;
    int points = 0;
    float front_min_distance = 999.0f;
    float left_min_distance = 999.0f;
    float right_min_distance = 999.0f;
    int front_points = 0;
    int left_points = 0;
    int right_points = 0;
};

struct VisionObstacleState
{
    // Depth-camera observation only.
    bool detected = false;
    float min_distance = 999.0f;
    float lateral_offset = 0.0f;
    int points = 0;
};

struct LocalizationState
{
    bool gps_valid = false;
    bool imu_valid = false;

    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;

    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
    double roll_deg = 0.0;
};

struct SensorChannelDiagnostics
{
    bool available = false;
    bool fresh = false;
    bool aligned = false;
    std::uint64_t sequence = 0;
    std::size_t cached = 0;
    std::size_t capacity = 0;
    std::uint64_t total_pushed = 0;
    std::uint64_t overwritten = 0;
    double age_ms = -1.0;
    double skew_ms = -1.0;
};

struct FusionDiagnostics
{
    bool reference_valid = false;
    std::uint64_t generation = 0;
    double reference_age_ms = -1.0;
};

struct VehicleState
{
    bool gps_online = false;
    bool imu_online = false;
    bool lidar_online = false;
    bool camera_online = false;
    bool stm32_online = false;

    ObstacleState obstacle;
    VisionObstacleState vision_obstacle;
    bishe::common::LidarObstacleSummary lidar_summary;
    bishe::common::DepthObstacleSummary depth_summary;
    bishe::common::ImuStateSummary imu_summary;
    LocalizationState localization;
    SensorChannelDiagnostics gps_diag;
    SensorChannelDiagnostics imu_diag;
    SensorChannelDiagnostics lidar_diag;
    SensorChannelDiagnostics camera_diag;
    FusionDiagnostics fusion_diag;
};

}  // namespace vehicle
}  // namespace bishe

#endif
