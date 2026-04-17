#include "control/auto_avoid.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace {

int clampSpeed(int speed_cm_s) {
    return std::clamp(speed_cm_s, 0, 100);
}

double clampSteeringAngle(double steering_angle_deg) {
    if (!std::isfinite(steering_angle_deg)) {
        return 0.0;
    }
    return std::clamp(
        steering_angle_deg,
        -Judgment::kMaxSteeringAngleDeg,
        Judgment::kMaxSteeringAngleDeg);
}

double normalizeAngleErrorDeg(double angle_deg) {
    if (!std::isfinite(angle_deg)) {
        return 0.0;
    }
    double normalized = std::fmod(angle_deg + 180.0, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }
    return normalized - 180.0;
}

double steeringAngleForDirection(
    AutoAvoidController::TurnDirection direction,
    double magnitude_deg) {
    switch (direction) {
        case AutoAvoidController::TurnDirection::Left:
            return -std::abs(magnitude_deg);
        case AutoAvoidController::TurnDirection::Right:
            return std::abs(magnitude_deg);
        case AutoAvoidController::TurnDirection::Straight:
        case AutoAvoidController::TurnDirection::Stop:
        default:
            return 0.0;
    }
}

bool isStableObstacleZone(Judgment::FrontObstacleZone zone) {
    return zone != Judgment::FrontObstacleZone::Unknown;
}

}  // namespace

AutoAvoidController::AutoAvoidController()
    : AutoAvoidController(Config{}) {}

AutoAvoidController::AutoAvoidController(const Config& config)
    : config_(config) {}

const AutoAvoidController::Config& AutoAvoidController::config() const {
    return config_;
}

void AutoAvoidController::reset() {
    resetObstacleZoneHysteresis();
}

const char* AutoAvoidController::motionModeName(MotionMode mode) {
    switch (mode) {
        case MotionMode::Drive:
            return "drive";
        case MotionMode::Stop:
        default:
            return "stop";
    }
}

const char* AutoAvoidController::turnDirectionName(TurnDirection direction) {
    switch (direction) {
        case TurnDirection::Straight:
            return "straight";
        case TurnDirection::Left:
            return "left";
        case TurnDirection::Right:
            return "right";
        case TurnDirection::Stop:
        default:
            return "stop";
    }
}

AutoAvoidController::Command AutoAvoidController::decide(
    const SensorSnapshot& snapshot) {
    Judgment::VehicleBoundaryInput boundary_input;
    boundary_input.left_valid = snapshot.negative_front.valid;
    boundary_input.left_nearest_m = snapshot.negative_front.nearest_m;
    boundary_input.right_valid = snapshot.positive_front.valid;
    boundary_input.right_nearest_m = snapshot.positive_front.nearest_m;
    const auto boundary = judgment_.checkVehicleBoundary(boundary_input);

    Judgment::FrontObstacleInput front_input;
    front_input.front_valid = snapshot.front.valid;
    front_input.front_nearest_m = snapshot.front.nearest_m;
    front_input.front_nearest_angle_deg = snapshot.front.nearest_angle_deg;
    auto front_obstacle = judgment_.checkFrontObstacle(front_input);
    const bool front_clear = Judgment::isFrontPathClear(
        snapshot.front.valid,
        snapshot.front.nearest_m);

    if (!snapshot.lidar_valid) {
        resetObstacleZoneHysteresis();
        return stopCommand(boundary, front_obstacle, "lidar invalid or stale");
    }

    if (!boundary.valid) {
        resetObstacleZoneHysteresis();
        return stopCommand(boundary, front_obstacle, "vehicle boundary data invalid");
    }

    if (!boundary.clear) {
        resetObstacleZoneHysteresis();
        return stopCommand(boundary, front_obstacle, "vehicle boundary too close");
    }

    if (!front_obstacle.valid) {
        resetObstacleZoneHysteresis();
        return stopCommand(boundary, front_obstacle, "front lidar data invalid");
    }

    if (front_clear) {
        resetObstacleZoneHysteresis();
        const double heading_correction_deg = headingCorrectionDeg(
            snapshot,
            config_.straight_heading_max_correction_deg);
        return driveCommand(
            TurnDirection::Straight,
            config_.cruise_speed_cm_s,
            heading_correction_deg,
            boundary,
            front_obstacle,
            true,
            snapshot,
            heading_correction_deg,
            "front path clear");
    }

    front_obstacle = stabilizeFrontObstacleZone(front_obstacle, false);

    TurnDirection direction = TurnDirection::Straight;
    double steering_angle_deg = 0.0;
    int speed_cm_s = config_.caution_speed_cm_s;
    std::string reason = "front obstacle caution";

    if (front_obstacle.zone == Judgment::FrontObstacleZone::Left) {
        direction = TurnDirection::Right;
    } else if (front_obstacle.zone == Judgment::FrontObstacleZone::Right) {
        direction = TurnDirection::Left;
    } else {
        direction = chooseCenterTurnDirection(snapshot);
    }

    if (front_obstacle.too_close) {
        speed_cm_s = config_.avoidance_speed_cm_s;
        steering_angle_deg = steeringAngleForDirection(
            direction,
            direction == TurnDirection::Straight ?
                0.0 :
                (front_obstacle.zone == Judgment::FrontObstacleZone::Center ?
                    config_.center_turn_angle_deg :
                    config_.avoidance_turn_angle_deg));
        reason = "front obstacle too close";
    } else {
        steering_angle_deg = steeringAngleForDirection(
            direction,
            direction == TurnDirection::Straight ? 0.0 : config_.caution_turn_angle_deg);
    }

    const double heading_correction_deg = headingCorrectionDeg(
        snapshot,
        front_obstacle.too_close ?
            config_.too_close_heading_max_correction_deg :
            config_.caution_heading_max_correction_deg);
    steering_angle_deg += heading_correction_deg;

    return driveCommand(
        direction,
        speed_cm_s,
        steering_angle_deg,
        boundary,
        front_obstacle,
        false,
        snapshot,
        heading_correction_deg,
        reason);
}

void AutoAvoidController::resetObstacleZoneHysteresis() {
    stable_obstacle_zone_ = Judgment::FrontObstacleZone::Unknown;
    pending_obstacle_zone_ = Judgment::FrontObstacleZone::Unknown;
    pending_obstacle_zone_ticks_ = 0;
}

Judgment::FrontObstacleResult AutoAvoidController::stabilizeFrontObstacleZone(
    const Judgment::FrontObstacleResult& front_obstacle,
    bool front_clear) {
    if (front_clear || !front_obstacle.valid || !isStableObstacleZone(front_obstacle.zone)) {
        resetObstacleZoneHysteresis();
        return front_obstacle;
    }

    if (!isStableObstacleZone(stable_obstacle_zone_)) {
        stable_obstacle_zone_ = front_obstacle.zone;
        pending_obstacle_zone_ = Judgment::FrontObstacleZone::Unknown;
        pending_obstacle_zone_ticks_ = 0;
        return front_obstacle;
    }

    if (front_obstacle.zone == stable_obstacle_zone_) {
        pending_obstacle_zone_ = Judgment::FrontObstacleZone::Unknown;
        pending_obstacle_zone_ticks_ = 0;
        return front_obstacle;
    }

    if (front_obstacle.zone != pending_obstacle_zone_) {
        pending_obstacle_zone_ = front_obstacle.zone;
        pending_obstacle_zone_ticks_ = 1;
    } else {
        ++pending_obstacle_zone_ticks_;
    }

    const int confirm_ticks = std::max(1, config_.obstacle_zone_switch_confirm_ticks);
    if (pending_obstacle_zone_ticks_ >= confirm_ticks) {
        stable_obstacle_zone_ = pending_obstacle_zone_;
        pending_obstacle_zone_ = Judgment::FrontObstacleZone::Unknown;
        pending_obstacle_zone_ticks_ = 0;
        return front_obstacle;
    }

    auto stabilized = front_obstacle;
    stabilized.zone = stable_obstacle_zone_;
    return stabilized;
}

double AutoAvoidController::headingCorrectionDeg(
    const SensorSnapshot& snapshot,
    double max_correction_deg) const {
    if (!snapshot.imu_valid || !snapshot.target_yaw_valid) {
        return 0.0;
    }
    if (!std::isfinite(snapshot.yaw_deg) || !std::isfinite(snapshot.target_yaw_deg)) {
        return 0.0;
    }

    const double yaw_error_deg = normalizeAngleErrorDeg(snapshot.target_yaw_deg - snapshot.yaw_deg);
    if (std::abs(yaw_error_deg) > config_.max_usable_yaw_error_deg) {
        return 0.0;
    }

    const double correction_limit = std::max(0.0, max_correction_deg);
    // ROS yaw normally increases counter-clockwise. In this vehicle command map,
    // negative steering means left and positive steering means right.
    return std::clamp(
        -config_.heading_kp * yaw_error_deg,
        -correction_limit,
        correction_limit);
}

AutoAvoidController::TurnDirection AutoAvoidController::chooseCenterTurnDirection(
    const SensorSnapshot& snapshot) const {
    if (!snapshot.negative_front.valid || !snapshot.positive_front.valid) {
        return TurnDirection::Right;
    }

    if (snapshot.negative_front.nearest_m > snapshot.positive_front.nearest_m) {
        return TurnDirection::Left;
    }
    if (snapshot.positive_front.nearest_m > snapshot.negative_front.nearest_m) {
        return TurnDirection::Right;
    }
    return TurnDirection::Right;
}

AutoAvoidController::Command AutoAvoidController::driveCommand(
    TurnDirection direction,
    int speed_cm_s,
    double steering_angle_deg,
    const Judgment::VehicleBoundaryResult& boundary,
    const Judgment::FrontObstacleResult& front_obstacle,
    bool front_clear,
    const SensorSnapshot& snapshot,
    double heading_correction_deg,
    std::string reason) const {
    Command command;
    const double clamped_steering_angle_deg = clampSteeringAngle(steering_angle_deg);
    command.valid = true;
    command.mode = MotionMode::Drive;
    command.direction = direction;
    command.speed_cm_s = clampSpeed(speed_cm_s);
    command.steering_angle_deg = clamped_steering_angle_deg;
    command.steering_encoder = Judgment::steeringAngleDegToEncoder(clamped_steering_angle_deg);
    command.boundary_clear = boundary.clear;
    command.front_clear = front_clear;
    command.front_too_close = front_obstacle.too_close;
    command.obstacle_zone = front_obstacle.zone;
    command.obstacle_angle_deg = front_obstacle.angle_deg;
    command.front_nearest_m = front_obstacle.nearest_m;
    command.imu_heading_used =
        snapshot.imu_valid &&
        snapshot.target_yaw_valid &&
        std::abs(heading_correction_deg) > 0.001;
    command.yaw_deg = snapshot.yaw_deg;
    command.target_yaw_deg = snapshot.target_yaw_deg;
    command.yaw_error_deg =
        (snapshot.imu_valid && snapshot.target_yaw_valid) ?
            normalizeAngleErrorDeg(snapshot.target_yaw_deg - snapshot.yaw_deg) :
            0.0;
    command.heading_correction_deg = heading_correction_deg;
    if (command.imu_heading_used) {
        reason += " + imu heading hold";
    }
    command.reason = std::move(reason);
    return command;
}

AutoAvoidController::Command AutoAvoidController::stopCommand(
    const Judgment::VehicleBoundaryResult& boundary,
    const Judgment::FrontObstacleResult& front_obstacle,
    std::string reason) const {
    Command command;
    command.valid = true;
    command.mode = MotionMode::Stop;
    command.direction = TurnDirection::Stop;
    command.speed_cm_s = 0;
    command.steering_encoder = 0;
    command.steering_angle_deg = 0.0;
    command.boundary_clear = boundary.clear;
    command.front_clear = false;
    command.front_too_close = front_obstacle.too_close;
    command.obstacle_zone = front_obstacle.zone;
    command.obstacle_angle_deg = front_obstacle.angle_deg;
    command.front_nearest_m = front_obstacle.nearest_m;
    command.reason = std::move(reason);
    return command;
}
