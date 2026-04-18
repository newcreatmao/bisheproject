#pragma once

#include "control/auto_avoid.hpp"

#include <cstdint>
#include <string>

#include "sensor_msgs/msg/laser_scan.hpp"

class AutoAvoidInputBuilder {
public:
    using SectorState = AutoAvoidController::SectorSample;

    struct LidarInputFrame {
        bool valid = false;
        bool obstacle_detected = false;
        double nearest_m = 0.0;
        double nearest_angle_deg = 0.0;
        int valid_points = 0;
        SectorState negative_front_sector;
        SectorState front_sector;
        SectorState auto_avoid_front_sector;
        SectorState positive_front_sector;
        SectorState avoidance_buffer_sector;
        std::string front_nearest_zone;
    };

    LidarInputFrame buildLidarInputFrame(
        const sensor_msgs::msg::LaserScan::SharedPtr& msg,
        const std::string& previous_front_nearest_zone) const;

    AutoAvoidController::SensorSnapshot buildSnapshot(
        const LidarInputFrame& lidar_frame,
        bool lidar_fresh,
        bool imu_fresh,
        double yaw_deg,
        std::int64_t timestamp_steady_ms) const;

    static std::string frontNearestZoneFromAngle(
        double angle_deg,
        const std::string& fallback = "");

private:
    static bool updateSector(SectorState& sector, double range, double angle_deg);
};
