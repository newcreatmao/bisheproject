#pragma once

#include "control/Judgment.hpp"

#include <string>

class AutoAvoidController {
public:
    struct SectorSample {
        bool valid = false;
        double nearest_m = 0.0;
        double nearest_angle_deg = 0.0;
        int valid_points = 0;
    };

    struct SensorSnapshot {
        bool lidar_valid = false;
        SectorSample negative_front;  // Lidar -90 to -60 degrees.
        SectorSample front;           // Lidar -60 to 60 degrees.
        SectorSample positive_front;  // Lidar 60 to 90 degrees.
        bool imu_valid = false;
        double yaw_deg = 0.0;
        bool target_yaw_valid = false;
        double target_yaw_deg = 0.0;
    };

    enum class MotionMode {
        Stop,
        Drive
    };

    enum class TurnDirection {
        Stop,
        Straight,
        Left,
        Right
    };

    struct Config {
        int cruise_speed_cm_s = 45;
        int caution_speed_cm_s = 36;
        int avoidance_speed_cm_s = 27;
        double caution_turn_angle_deg = 8.0;
        double avoidance_turn_angle_deg = 13.0;
        double center_turn_angle_deg = 15.0;
        double heading_kp = 0.25;
        double straight_heading_max_correction_deg = 5.0;
        double caution_heading_max_correction_deg = 3.0;
        double too_close_heading_max_correction_deg = 1.5;
        double max_usable_yaw_error_deg = 45.0;
        int obstacle_zone_switch_confirm_ticks = 4;
    };

    struct Command {
        bool valid = false;
        MotionMode mode = MotionMode::Stop;
        TurnDirection direction = TurnDirection::Stop;
        int speed_cm_s = 0;
        int steering_encoder = 0;
        double steering_angle_deg = 0.0;
        bool boundary_clear = false;
        bool front_clear = false;
        bool front_too_close = false;
        Judgment::FrontObstacleZone obstacle_zone = Judgment::FrontObstacleZone::Unknown;
        double obstacle_angle_deg = 0.0;
        double front_nearest_m = 0.0;
        bool imu_heading_used = false;
        double yaw_deg = 0.0;
        double target_yaw_deg = 0.0;
        double yaw_error_deg = 0.0;
        double heading_correction_deg = 0.0;
        std::string reason;
    };

    AutoAvoidController();
    explicit AutoAvoidController(const Config& config);

    Command decide(const SensorSnapshot& snapshot);
    void reset();

    const Config& config() const;

    static const char* motionModeName(MotionMode mode);
    static const char* turnDirectionName(TurnDirection direction);

private:
    double headingCorrectionDeg(
        const SensorSnapshot& snapshot,
        double max_correction_deg) const;
    void resetObstacleZoneHysteresis();
    Judgment::FrontObstacleResult stabilizeFrontObstacleZone(
        const Judgment::FrontObstacleResult& front_obstacle,
        bool front_clear);
    TurnDirection chooseCenterTurnDirection(const SensorSnapshot& snapshot) const;
    Command driveCommand(
        TurnDirection direction,
        int speed_cm_s,
        double steering_angle_deg,
        const Judgment::VehicleBoundaryResult& boundary,
        const Judgment::FrontObstacleResult& front_obstacle,
        bool front_clear,
        const SensorSnapshot& snapshot,
        double heading_correction_deg,
        std::string reason) const;
    Command stopCommand(
        const Judgment::VehicleBoundaryResult& boundary,
        const Judgment::FrontObstacleResult& front_obstacle,
        std::string reason) const;

    Config config_;
    Judgment judgment_;
    Judgment::FrontObstacleZone stable_obstacle_zone_ =
        Judgment::FrontObstacleZone::Unknown;
    Judgment::FrontObstacleZone pending_obstacle_zone_ =
        Judgment::FrontObstacleZone::Unknown;
    int pending_obstacle_zone_ticks_ = 0;
};
