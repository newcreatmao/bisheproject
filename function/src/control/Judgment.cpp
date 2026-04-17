#include "control/Judgment.hpp"

#include <algorithm>
#include <cmath>

namespace {

double normalizedBoundaryThreshold(double threshold_m) {
    if (!std::isfinite(threshold_m) || threshold_m <= 0.0) {
        return Judgment::kVehicleBoundaryThresholdMeters;
    }
    return threshold_m;
}

double normalizedFrontPathThreshold(double threshold_m) {
    if (!std::isfinite(threshold_m) || threshold_m <= 0.0) {
        return Judgment::kFrontPathThresholdMeters;
    }
    return threshold_m;
}

double normalizedFrontObstacleThreshold(double threshold_m) {
    if (!std::isfinite(threshold_m) || threshold_m <= 0.0) {
        return Judgment::kFrontObstacleThresholdMeters;
    }
    return threshold_m;
}

}  // namespace

bool Judgment::isBoundaryDistanceClear(
    bool valid,
    double nearest_m,
    double threshold_m) {
    if (!isBoundaryReadingValid(valid, nearest_m)) {
        return false;
    }
    return nearest_m > normalizedBoundaryThreshold(threshold_m);
}

bool Judgment::isBoundaryReadingValid(bool valid, double nearest_m) {
    return valid && std::isfinite(nearest_m);
}

bool Judgment::isLeftBoundaryClear(
    bool left_valid,
    double left_nearest_m,
    double threshold_m) {
    return isBoundaryDistanceClear(left_valid, left_nearest_m, threshold_m);
}

bool Judgment::isRightBoundaryClear(
    bool right_valid,
    double right_nearest_m,
    double threshold_m) {
    return isBoundaryDistanceClear(right_valid, right_nearest_m, threshold_m);
}

bool Judgment::areVehicleBoundariesClear(
    const VehicleBoundaryInput& input,
    double threshold_m) {
    return isLeftBoundaryClear(input.left_valid, input.left_nearest_m, threshold_m) &&
        isRightBoundaryClear(input.right_valid, input.right_nearest_m, threshold_m);
}

bool Judgment::isFrontPathClear(
    bool front_valid,
    double front_nearest_m,
    double threshold_m) {
    if (!isBoundaryReadingValid(front_valid, front_nearest_m)) {
        return false;
    }
    return front_nearest_m > normalizedFrontPathThreshold(threshold_m);
}

bool Judgment::isFrontObstacleTooClose(
    bool front_valid,
    double front_nearest_m,
    double threshold_m) {
    if (!isBoundaryReadingValid(front_valid, front_nearest_m)) {
        return false;
    }
    return front_nearest_m < normalizedFrontObstacleThreshold(threshold_m);
}

Judgment::FrontObstacleZone Judgment::frontObstacleZoneFromAngle(double angle_deg) {
    if (!std::isfinite(angle_deg) || angle_deg < -60.0 || angle_deg > 60.0) {
        return FrontObstacleZone::Unknown;
    }
    if (angle_deg < -20.0) {
        return FrontObstacleZone::Left;
    }
    if (angle_deg <= 20.0) {
        return FrontObstacleZone::Center;
    }
    return FrontObstacleZone::Right;
}

const char* Judgment::frontObstacleZoneName(FrontObstacleZone zone) {
    switch (zone) {
        case FrontObstacleZone::Left:
            return "left";
        case FrontObstacleZone::Center:
            return "center";
        case FrontObstacleZone::Right:
            return "right";
        case FrontObstacleZone::Unknown:
        default:
            return "unknown";
    }
}

int Judgment::clampSteeringEncoder(int encoder_value) {
    return std::clamp(encoder_value, kMinSteeringEncoder, kMaxSteeringEncoder);
}

int Judgment::steeringAngleDegToEncoder(double steering_angle_deg) {
    if (!std::isfinite(steering_angle_deg)) {
        return 0;
    }

    const double clamped_angle_deg = std::clamp(
        steering_angle_deg,
        -kMaxSteeringAngleDeg,
        kMaxSteeringAngleDeg);
    const double encoder_per_degree =
        static_cast<double>(kMaxSteeringEncoder) / kMaxSteeringAngleDeg;
    return clampSteeringEncoder(static_cast<int>(std::lround(clamped_angle_deg * encoder_per_degree)));
}

double Judgment::steeringEncoderToAngleDeg(int encoder_value) {
    const int clamped_encoder = clampSteeringEncoder(encoder_value);
    const double degree_per_encoder =
        kMaxSteeringAngleDeg / static_cast<double>(kMaxSteeringEncoder);
    return static_cast<double>(clamped_encoder) * degree_per_encoder;
}

Judgment::FrontObstacleResult Judgment::checkFrontObstacle(
    const FrontObstacleInput& input,
    double threshold_m) const {
    FrontObstacleResult result;
    result.threshold_m = normalizedFrontObstacleThreshold(threshold_m);
    result.valid =
        isBoundaryReadingValid(input.front_valid, input.front_nearest_m) &&
        std::isfinite(input.front_nearest_angle_deg);
    result.too_close = isFrontObstacleTooClose(
        input.front_valid,
        input.front_nearest_m,
        result.threshold_m);
    result.zone = frontObstacleZoneFromAngle(input.front_nearest_angle_deg);
    result.angle_deg = input.front_nearest_angle_deg;
    result.nearest_m = input.front_nearest_m;
    if (result.zone == FrontObstacleZone::Unknown) {
        result.valid = false;
        result.too_close = false;
    }
    return result;
}

Judgment::VehicleBoundaryResult Judgment::checkVehicleBoundary(
    const VehicleBoundaryInput& input,
    double threshold_m) const {
    VehicleBoundaryResult result;
    result.threshold_m = normalizedBoundaryThreshold(threshold_m);
    result.left_clear = isLeftBoundaryClear(
        input.left_valid,
        input.left_nearest_m,
        result.threshold_m);
    result.right_clear = isRightBoundaryClear(
        input.right_valid,
        input.right_nearest_m,
        result.threshold_m);
    result.valid =
        isBoundaryReadingValid(input.left_valid, input.left_nearest_m) &&
        isBoundaryReadingValid(input.right_valid, input.right_nearest_m);
    result.clear = result.valid && areVehicleBoundariesClear(input, result.threshold_m);
    return result;
}
