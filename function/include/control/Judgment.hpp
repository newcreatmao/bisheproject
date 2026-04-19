#pragma once

class Judgment {
public:
    static constexpr double kVehicleBoundaryThresholdMeters = 0.60;
    static constexpr double kFrontObstacleThresholdMeters = 0.80;
    static constexpr double kMaxSteeringAngleDeg = 15.0;
    static constexpr int kMinSteeringEncoder = -230;
    static constexpr int kMaxSteeringEncoder = 230;

    struct VehicleBoundaryInput {
        bool left_valid = false;
        double left_nearest_m = 0.0;   // Lidar -90 to -62 degrees.
        bool right_valid = false;
        double right_nearest_m = 0.0;  // Lidar 62 to 90 degrees.
    };

    struct VehicleBoundaryResult {
        // valid means the boundary judgment itself completed successfully.
        // Missing side-sector readings are allowed and no longer force an
        // invalid result in wide-open areas.
        bool valid = false;
        bool left_clear = false;
        bool right_clear = false;
        bool clear = false;
        double threshold_m = 0.60;
    };

    enum class FrontObstacleZone {
        Unknown,
        Left,
        Center,
        Right
    };

    struct FrontObstacleInput {
        bool front_valid = false;
        double front_nearest_m = 0.0;       // Lidar -58 to 58 degrees.
        double front_nearest_angle_deg = 0.0;
    };

    struct FrontObstacleResult {
        bool valid = false;
        bool too_close = false;
        FrontObstacleZone zone = FrontObstacleZone::Unknown;
        double angle_deg = 0.0;
        double nearest_m = 0.0;
        double threshold_m = 0.80;
    };

    VehicleBoundaryResult checkVehicleBoundary(
        const VehicleBoundaryInput& input,
        double threshold_m = kVehicleBoundaryThresholdMeters) const;

    FrontObstacleResult checkFrontObstacle(
        const FrontObstacleInput& input,
        double threshold_m = kFrontObstacleThresholdMeters) const;

    static bool isBoundaryReadingValid(bool valid, double nearest_m);

    static bool isLeftBoundaryClear(
        bool left_valid,
        double left_nearest_m,
        double threshold_m = kVehicleBoundaryThresholdMeters);

    static bool isRightBoundaryClear(
        bool right_valid,
        double right_nearest_m,
        double threshold_m = kVehicleBoundaryThresholdMeters);

    static bool areVehicleBoundariesClear(
        const VehicleBoundaryInput& input,
        double threshold_m = kVehicleBoundaryThresholdMeters);

    static bool isFrontObstacleTooClose(
        bool front_valid,
        double front_nearest_m,
        double threshold_m = kFrontObstacleThresholdMeters);

    static FrontObstacleZone frontObstacleZoneFromAngle(double angle_deg);

    static bool isFrontObstacleZoneBufferAngle(double angle_deg);

    static const char* frontObstacleZoneName(FrontObstacleZone zone);

    static int steeringAngleDegToEncoder(double steering_angle_deg);

    static double steeringEncoderToAngleDeg(int encoder_value);

    static int clampSteeringEncoder(int encoder_value);

    static bool isBoundaryDistanceClear(
        bool valid,
        double nearest_m,
        double threshold_m = kVehicleBoundaryThresholdMeters);
};
