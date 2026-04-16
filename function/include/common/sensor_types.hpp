#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "common/fixed_ring_buffer.hpp"

namespace bishe::common
{

constexpr float kNoObstacleDistanceMeters = 999.0f;
constexpr std::size_t kLidarSummaryHistoryCapacity = 8;
constexpr std::size_t kDepthSummaryHistoryCapacity = 8;
constexpr std::size_t kImuSummaryHistoryCapacity = 16;

struct LidarObstacleSummary
{
    rclcpp::Time stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    bool valid = false;
    float front_min_distance_m = kNoObstacleDistanceMeters;
    float left_min_distance_m = kNoObstacleDistanceMeters;
    float right_min_distance_m = kNoObstacleDistanceMeters;
    int front_valid_points = 0;
    int left_valid_points = 0;
    int right_valid_points = 0;
    bool obstacle_detected = false;
    std::uint64_t sequence = 0;
};

struct DepthObstacleSummary
{
    rclcpp::Time stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    bool valid = false;
    bool obstacle_detected = false;
    float front_min_distance_m = kNoObstacleDistanceMeters;
    float obstacle_ratio = 0.0f;
    std::uint64_t sequence = 0;
};

struct ImuStateSummary
{
    rclcpp::Time stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    bool valid = false;
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    std::uint64_t sequence = 0;
};

// Observation-only snapshot for the local sensing chain. It carries geometry
// summaries and sensor history, but no control intent or recognition payload.
struct LocalObservationSnapshot
{
    rclcpp::Time snapshot_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    rclcpp::Time reference_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    std::uint64_t generation = 0;

    std::optional<LidarObstacleSummary> latest_lidar;
    std::optional<DepthObstacleSummary> latest_depth;
    std::optional<ImuStateSummary> latest_imu;

    FixedRingBufferWindow<LidarObstacleSummary, kLidarSummaryHistoryCapacity> lidar_history;
    FixedRingBufferWindow<DepthObstacleSummary, kDepthSummaryHistoryCapacity> depth_history;
    FixedRingBufferWindow<ImuStateSummary, kImuSummaryHistoryCapacity> imu_history;

    FixedRingBufferStats lidar_stats;
    FixedRingBufferStats depth_stats;
    FixedRingBufferStats imu_stats;
};

}  // namespace bishe::common
