#include "control/auto_avoid.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

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

bool isTurningDirection(AutoAvoidController::TurnDirection direction) {
    return direction == AutoAvoidController::TurnDirection::Left ||
        direction == AutoAvoidController::TurnDirection::Right;
}

bool isFrontZoneBuffer(
    const Judgment::FrontObstacleResult& front_obstacle) {
    return front_obstacle.valid &&
        front_obstacle.zone == Judgment::FrontObstacleZone::Unknown &&
        Judgment::isFrontObstacleZoneBufferAngle(front_obstacle.angle_deg);
}

bool isBoundaryRiskSide(AutoAvoidController::BoundaryRiskSide side) {
    return side == AutoAvoidController::BoundaryRiskSide::Left ||
        side == AutoAvoidController::BoundaryRiskSide::Right;
}

double steeringAngleForBoundarySide(
    AutoAvoidController::BoundaryRiskSide side,
    double magnitude_deg) {
    switch (side) {
        case AutoAvoidController::BoundaryRiskSide::Left:
            return std::abs(magnitude_deg);
        case AutoAvoidController::BoundaryRiskSide::Right:
            return -std::abs(magnitude_deg);
        case AutoAvoidController::BoundaryRiskSide::None:
        default:
            return 0.0;
    }
}

}  // namespace

AutoAvoidController::AutoAvoidController()
    : AutoAvoidController(Config{}) {}

AutoAvoidController::AutoAvoidController(const Config& config)
    : config_(config) {}

const AutoAvoidController::Config& AutoAvoidController::config() const {
    return config_;
}

AutoAvoidController::AvoidanceStage AutoAvoidController::currentAvoidanceStage() const {
    return active_avoidance_state_.stage;
}

void AutoAvoidController::reset() {
    resetFilteredLidarHistory();
    resetFrontSpikeFilter();
    resetMotionHysteresis();
    resetObstacleZoneStabilizer();
    resetDriveSpeed();
    resetAvoidanceState();
    resetSteeringSmoothing();
    target_yaw_state_ = TargetYawState{};
    target_yaw_state_.last_clear_reason = TargetYawClearReason::ControllerReset;
    path_reference_state_ = PathReferenceState{};
    path_reference_state_.clear_reason = PathReferenceClearReason::ControllerReset;
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

const char* AutoAvoidController::avoidanceStageName(AvoidanceStage stage) {
    switch (stage) {
        case AvoidanceStage::Turning:
            return "Turning";
        case AvoidanceStage::ClearanceHold:
            return "ClearanceHold";
        case AvoidanceStage::ReturnHeading:
            return "ReturnToPath";
        case AvoidanceStage::Idle:
        default:
            return "Idle";
    }
}

const char* AutoAvoidController::decisionReasonName(DecisionReason reason) {
    switch (reason) {
        case DecisionReason::InvalidDecision:
            return "invalid_decision";
        case DecisionReason::LidarInvalid:
            return "lidar_invalid";
        case DecisionReason::BoundaryInvalid:
            return "boundary_invalid";
        case DecisionReason::BoundaryStop:
            return "boundary_stop";
        case DecisionReason::FrontInvalid:
            return "front_invalid";
        case DecisionReason::SectorBuffer:
            return "sector_buffer";
        case DecisionReason::SectorBufferContinue:
            return "sector_buffer_continue";
        case DecisionReason::EmergencyStop:
            return "emergency_stop";
        case DecisionReason::FrontZoneBuffer:
            return "front_zone_buffer";
        case DecisionReason::FrontAvoidanceImu:
            return "front_avoidance_imu";
        case DecisionReason::FrontAvoidanceEncoderFallback:
            return "front_avoidance_encoder_fallback";
        case DecisionReason::ClearanceHoldImu:
            return "clearance_hold_imu";
        case DecisionReason::ClearanceHoldEncoderFallback:
            return "clearance_hold_encoder_fallback";
        case DecisionReason::ReturnHeading:
            return "return_to_path";
        case DecisionReason::StraightDrive:
            return "straight_drive";
        case DecisionReason::Unknown:
        default:
            return "unknown";
    }
}

const char* AutoAvoidController::fallbackReasonName(FallbackReason reason) {
    switch (reason) {
        case FallbackReason::InvalidDecision:
            return "invalid_decision";
        case FallbackReason::LidarInvalid:
            return "lidar_invalid";
        case FallbackReason::BoundaryInvalid:
            return "boundary_invalid";
        case FallbackReason::FrontInvalid:
            return "front_invalid";
        case FallbackReason::SectorBuffer:
            return "sector_buffer";
        case FallbackReason::FrontZoneBuffer:
            return "front_zone_buffer";
        case FallbackReason::None:
        default:
            return "none";
    }
}

const char* AutoAvoidController::encoderFallbackKindName(EncoderFallbackKind kind) {
    switch (kind) {
        case EncoderFallbackKind::FrontObstacleLinear:
            return "front_obstacle_linear";
        case EncoderFallbackKind::ClearanceHoldFixed:
            return "clearance_hold_fixed";
        case EncoderFallbackKind::None:
        default:
            return "none";
    }
}

const char* AutoAvoidController::pathReferenceClearReasonName(
    PathReferenceClearReason reason) {
    switch (reason) {
        case PathReferenceClearReason::ControllerReset:
            return "controller_reset";
        case PathReferenceClearReason::InvalidInput:
            return "invalid_input";
        case PathReferenceClearReason::SafetyStop:
            return "safety_stop";
        case PathReferenceClearReason::AvoidanceCompleted:
            return "avoidance_completed";
        case PathReferenceClearReason::None:
        default:
            return "none";
    }
}

const char* AutoAvoidController::targetYawClearReasonName(TargetYawClearReason reason) {
    switch (reason) {
        case TargetYawClearReason::ControllerReset:
            return "controller_reset";
        case TargetYawClearReason::None:
        default:
            return "none";
    }
}

const char* AutoAvoidController::boundaryRiskSideName(BoundaryRiskSide side) {
    switch (side) {
        case BoundaryRiskSide::Left:
            return "left";
        case BoundaryRiskSide::Right:
            return "right";
        case BoundaryRiskSide::None:
        default:
            return "none";
    }
}

const char* AutoAvoidController::boundaryRecoveryLevelName(BoundaryRecoveryLevel level) {
    switch (level) {
        case BoundaryRecoveryLevel::Soft:
            return "soft";
        case BoundaryRecoveryLevel::Strong:
            return "strong";
        case BoundaryRecoveryLevel::Critical:
            return "critical";
        case BoundaryRecoveryLevel::None:
        default:
            return "none";
    }
}

AutoAvoidController::Command AutoAvoidController::decide(
    const SensorSnapshot& snapshot) {
    path_reference_state_.captured_this_cycle = false;
    auto assessment = assessSnapshot(bindManagedTargetYaw(snapshot));
    auto front_obstacle = assessment.front_obstacle;
    auto debug = baseDebugInfo(assessment);

    if (!assessment.snapshot.lidar_valid) {
        resetMotionHysteresis();
        resetObstacleZoneStabilizer();
        resetDriveSpeed();
        resetAvoidanceState();
        resetSteeringSmoothing();
        clearPathReference(PathReferenceClearReason::InvalidInput);
        debug.path_reference_valid = false;
        debug.path_reference_clear_reason = path_reference_state_.clear_reason;
        debug.reason_code = DecisionReason::LidarInvalid;
        debug.fallback_reason = FallbackReason::LidarInvalid;
        return safetyFallbackCommand(debug);
    }

    if (!assessment.boundary.valid) {
        resetMotionHysteresis();
        resetObstacleZoneStabilizer();
        resetDriveSpeed();
        resetAvoidanceState();
        resetSteeringSmoothing();
        clearPathReference(PathReferenceClearReason::InvalidInput);
        debug.path_reference_valid = false;
        debug.path_reference_clear_reason = path_reference_state_.clear_reason;
        debug.reason_code = DecisionReason::BoundaryInvalid;
        debug.fallback_reason = FallbackReason::BoundaryInvalid;
        return safetyFallbackCommand(debug);
    }

    if (!assessment.boundary.clear) {
        resetMotionHysteresis();
        resetObstacleZoneStabilizer();
        resetDriveSpeed();
        resetAvoidanceState();
        resetSteeringSmoothing();
        clearPathReference(PathReferenceClearReason::SafetyStop);
        debug.path_reference_valid = false;
        debug.path_reference_clear_reason = path_reference_state_.clear_reason;
        debug.reason_code = DecisionReason::BoundaryStop;
        debug.boundary_stop = true;
        return stopCommand(debug);
    }

    if (!front_obstacle.valid) {
        resetMotionHysteresis();
        resetObstacleZoneStabilizer();
        resetDriveSpeed();
        resetAvoidanceState();
        resetSteeringSmoothing();
        clearPathReference(PathReferenceClearReason::InvalidInput);
        debug.path_reference_valid = false;
        debug.path_reference_clear_reason = path_reference_state_.clear_reason;
        debug.reason_code = DecisionReason::FrontInvalid;
        debug.fallback_reason = FallbackReason::FrontInvalid;
        return safetyFallbackCommand(debug);
    }

    if (isSectorBufferNearest(assessment.snapshot)) {
        if ((active_avoidance_state_.stage == AvoidanceStage::Turning ||
                active_avoidance_state_.stage == AvoidanceStage::ClearanceHold) &&
            hasCommittedAvoidanceTurn() &&
            assessment.front_distance_valid &&
            assessment.safety_front_m > config_.emergency_stop_distance_m) {
            return decideSectorBufferDuringActiveStage(assessment, debug);
        }
        resetSteeringSmoothing();
        debug.reason_code = DecisionReason::SectorBuffer;
        debug.fallback_reason = FallbackReason::SectorBuffer;
        return neutralSafeCommand(debug);
    }

    if (assessment.front_distance_valid &&
        emergencyStopActive(assessment.safety_front_m)) {
        resetDriveSpeed();
        resetSteeringSmoothing();
        clearPathReference(PathReferenceClearReason::SafetyStop);
        debug.path_reference_valid = false;
        debug.path_reference_clear_reason = path_reference_state_.clear_reason;
        debug.reason_code = DecisionReason::EmergencyStop;
        debug.emergency_stop = true;
        return stopCommand(debug);
    }

    front_obstacle.too_close =
        assessment.front_distance_valid &&
        avoidanceTurnActive(assessment.safety_front_m);

    if (shouldReplanFrontDuringActiveAvoidance(assessment.snapshot, front_obstacle)) {
        resetAvoidanceState();
        front_obstacle.too_close = true;
        debug.replan_triggered = true;
    }

    const auto zone_resolution =
        resolveFrontObstacleZone(assessment, front_obstacle);
    front_obstacle = zone_resolution.obstacle;
    debug.resolved_zone = front_obstacle.zone;
    debug.zone_stabilized = zone_resolution.stabilized;
    debug.zone_ambiguous = zone_resolution.ambiguous;
    debug.resolved_zone_override_active = zone_resolution.override_active;
    debug.resolved_zone_override_reason = zone_resolution.override_reason;
    if (zone_resolution.ambiguous) {
        resetSteeringSmoothing();
        debug.reason_code = DecisionReason::FrontZoneBuffer;
        debug.fallback_reason = FallbackReason::FrontZoneBuffer;
        return neutralSafeCommand(debug);
    }

    if (front_obstacle.too_close) {
        return decideTooCloseFront(assessment, front_obstacle, debug);
    }

    if (active_avoidance_state_.stage != AvoidanceStage::Idle) {
        return decideActiveAvoidanceStage(assessment, front_obstacle, debug);
    }

    return decideStraightDrive(assessment, front_obstacle, debug);
}

AutoAvoidController::SensorSnapshot AutoAvoidController::normalizedSnapshot(
    const SensorSnapshot& snapshot) const {
    SensorSnapshot normalized = snapshot;
    normalized.front_nearest_valid =
        normalized.front.valid && std::isfinite(normalized.front.nearest_m);
    normalized.front_nearest_m =
        normalized.front_nearest_valid ? normalized.front.nearest_m : 0.0;
    normalized.front_angle_deg =
        normalized.front.valid && std::isfinite(normalized.front.nearest_angle_deg) ?
            normalized.front.nearest_angle_deg :
            0.0;
    normalized.front_support_points =
        normalized.front.valid ? normalized.front.support_points : 0;
    return normalized;
}

AutoAvoidController::SensorSnapshot AutoAvoidController::bindManagedTargetYaw(
    const SensorSnapshot& snapshot) {
    target_yaw_state_.locked_this_cycle = false;

    SensorSnapshot bound = snapshot;
    if (!target_yaw_state_.valid &&
        bound.imu_valid &&
        std::isfinite(bound.yaw_deg)) {
        target_yaw_state_.valid = true;
        target_yaw_state_.yaw_deg = bound.yaw_deg;
        target_yaw_state_.locked_timestamp_steady_ms = bound.timestamp_steady_ms;
        target_yaw_state_.locked_by_stage = active_avoidance_state_.stage;
        target_yaw_state_.locked_this_cycle = true;
        target_yaw_state_.last_clear_reason = TargetYawClearReason::None;
    }

    bound.target_yaw_valid = target_yaw_state_.valid;
    bound.target_yaw_deg = target_yaw_state_.valid ? target_yaw_state_.yaw_deg : 0.0;
    return bound;
}

AutoAvoidController::SnapshotAssessment AutoAvoidController::assessSnapshot(
    const SensorSnapshot& snapshot) {
    const auto filtered_snapshot = filteredSnapshot(snapshot);
    const auto safer_distance =
        [](const SectorSample& raw_sample, const SectorSample& filtered_sample) {
            const bool raw_valid = raw_sample.valid && std::isfinite(raw_sample.nearest_m);
            const bool filtered_valid =
                filtered_sample.valid && std::isfinite(filtered_sample.nearest_m);
            if (raw_valid && filtered_valid) {
                return std::min(raw_sample.nearest_m, filtered_sample.nearest_m);
            }
            if (raw_valid) {
                return raw_sample.nearest_m;
            }
            if (filtered_valid) {
                return filtered_sample.nearest_m;
            }
            return std::numeric_limits<double>::quiet_NaN();
        };

    SnapshotAssessment assessment;
    assessment.snapshot = normalizedSnapshot(filtered_snapshot);
    assessment.safety_negative_front_m =
        safer_distance(snapshot.negative_front, filtered_snapshot.negative_front);
    assessment.safety_front_m =
        safer_distance(snapshot.front, filtered_snapshot.front);
    assessment.safety_positive_front_m =
        safer_distance(snapshot.positive_front, filtered_snapshot.positive_front);
    assessment.front_distance_valid = std::isfinite(assessment.safety_front_m);

    Judgment::VehicleBoundaryInput boundary_input;
    boundary_input.left_valid = std::isfinite(assessment.safety_negative_front_m);
    boundary_input.left_nearest_m =
        boundary_input.left_valid ? assessment.safety_negative_front_m : 0.0;
    boundary_input.right_valid = std::isfinite(assessment.safety_positive_front_m);
    boundary_input.right_nearest_m =
        boundary_input.right_valid ? assessment.safety_positive_front_m : 0.0;
    assessment.boundary = judgment_.checkVehicleBoundary(boundary_input);

    Judgment::FrontObstacleInput front_input;
    front_input.front_valid = assessment.snapshot.front.valid;
    front_input.front_nearest_m = assessment.snapshot.front_nearest_m;
    front_input.front_nearest_angle_deg = assessment.snapshot.front_angle_deg;
    assessment.front_obstacle = judgment_.checkFrontObstacle(front_input);
    return assessment;
}

AutoAvoidController::DebugInfo AutoAvoidController::baseDebugInfo(
    const SnapshotAssessment& assessment) const {
    DebugInfo debug;
    debug.snapshot_fresh = assessment.snapshot.snapshot_fresh;
    debug.lidar_valid = assessment.snapshot.lidar_valid;
    debug.imu_valid = assessment.snapshot.imu_valid;
    debug.front_nearest_valid = assessment.snapshot.front_nearest_valid;
    debug.front_nearest_m = assessment.snapshot.front_nearest_m;
    debug.front_angle_deg = assessment.snapshot.front_angle_deg;
    debug.front_support_points = assessment.snapshot.front_support_points;
    debug.front_target_selection = assessment.snapshot.front_target_selection;
    debug.state = active_avoidance_state_.stage;
    debug.direction = active_avoidance_state_.committed_direction;
    debug.raw_zone = assessment.front_obstacle.zone;
    debug.resolved_zone = assessment.front_obstacle.zone;
    debug.spike_suppressed = front_spike_filter_state_.suppressed_this_cycle;
    debug.target_yaw_valid = target_yaw_state_.valid;
    debug.target_yaw_deg = target_yaw_state_.valid ? target_yaw_state_.yaw_deg : 0.0;
    debug.target_yaw_locked_ms = target_yaw_state_.locked_timestamp_steady_ms;
    debug.target_yaw_locked_by_stage = target_yaw_state_.locked_by_stage;
    debug.target_yaw_locked_this_cycle = target_yaw_state_.locked_this_cycle;
    debug.target_yaw_clear_reason = target_yaw_state_.last_clear_reason;
    debug.path_reference_valid = path_reference_state_.valid;
    debug.reference_yaw_deg =
        path_reference_state_.valid ? path_reference_state_.reference_yaw_deg : 0.0;
    debug.reference_side_balance =
        path_reference_state_.valid ? path_reference_state_.reference_side_balance : 0.0;
    debug.reference_left_distance_m =
        path_reference_state_.valid ? path_reference_state_.reference_left_distance_m : 0.0;
    debug.reference_right_distance_m =
        path_reference_state_.valid ? path_reference_state_.reference_right_distance_m : 0.0;
    debug.path_reference_captured_ms =
        path_reference_state_.valid ? path_reference_state_.captured_steady_ms : 0;
    debug.path_reference_captured_stage =
        path_reference_state_.valid ? path_reference_state_.captured_stage : AvoidanceStage::Idle;
    debug.path_reference_captured_this_cycle = path_reference_state_.captured_this_cycle;
    debug.path_reference_clear_reason = path_reference_state_.clear_reason;
    return debug;
}

void AutoAvoidController::capturePathReferenceIfNeeded(
    const SnapshotAssessment& assessment) {
    if (path_reference_state_.valid) {
        return;
    }

    if (!std::isfinite(assessment.safety_negative_front_m) ||
        !std::isfinite(assessment.safety_positive_front_m)) {
        return;
    }

    double reference_yaw_deg = std::numeric_limits<double>::quiet_NaN();
    if (assessment.snapshot.imu_valid && std::isfinite(assessment.snapshot.yaw_deg)) {
        reference_yaw_deg = assessment.snapshot.yaw_deg;
    } else if (assessment.snapshot.target_yaw_valid &&
        std::isfinite(assessment.snapshot.target_yaw_deg)) {
        reference_yaw_deg = assessment.snapshot.target_yaw_deg;
    }

    if (!std::isfinite(reference_yaw_deg)) {
        return;
    }

    path_reference_state_.valid = true;
    path_reference_state_.reference_yaw_deg = reference_yaw_deg;
    path_reference_state_.reference_side_balance =
        sideBalanceFromAssessment(assessment);
    path_reference_state_.reference_left_distance_m =
        assessment.safety_negative_front_m;
    path_reference_state_.reference_right_distance_m =
        assessment.safety_positive_front_m;
    path_reference_state_.captured_steady_ms =
        assessment.snapshot.timestamp_steady_ms;
    path_reference_state_.captured_this_cycle = true;
    path_reference_state_.captured_stage = active_avoidance_state_.stage;
    path_reference_state_.clear_reason = PathReferenceClearReason::None;
}

void AutoAvoidController::clearPathReference(PathReferenceClearReason reason) {
    path_reference_state_ = PathReferenceState{};
    path_reference_state_.clear_reason = reason;
}

AutoAvoidController::ObstacleZoneResolution AutoAvoidController::resolveFrontObstacleZone(
    const SnapshotAssessment& assessment,
    const Judgment::FrontObstacleResult& front_obstacle) {
    ObstacleZoneResolution resolution;
    resolution.obstacle = front_obstacle;
    const auto preferred_direction = turnDirectionForObstacleZone(front_obstacle.zone);
    const bool conflict_with_committed_direction =
        hasCommittedAvoidanceTurn() &&
        isTurningDirection(preferred_direction) &&
        preferred_direction != active_avoidance_state_.committed_direction &&
        (!assessment.snapshot.front_target_selection.valid ||
            !assessment.snapshot.front_target_selection.selected_front_cluster_wall_like);
    if (!isFrontZoneBuffer(front_obstacle)) {
        if (front_obstacle.too_close) {
            bool stabilized = false;
            const auto previous_stable_zone = obstacle_zone_state_.stable_zone;
            const int confirm_ticks_override =
                conflict_with_committed_direction ?
                    std::max(1, config_.obstacle_zone_override_confirm_ticks) :
                    0;
            resolution.obstacle = stabilizeFrontObstacleZone(
                front_obstacle,
                stabilized,
                confirm_ticks_override);
            resolution.stabilized = stabilized;
            if (conflict_with_committed_direction &&
                isStableObstacleZone(previous_stable_zone) &&
                previous_stable_zone != resolution.obstacle.zone &&
                resolution.obstacle.zone == front_obstacle.zone) {
                resolution.override_active = true;
                resolution.override_reason = "committed_direction_conflict";
            }
        }
        return resolution;
    }

    if (hasCommittedAvoidanceTurn()) {
        resolution.obstacle.zone = active_avoidance_state_.committed_obstacle_side_zone;
        resolution.stabilized = true;
        return resolution;
    }

    if (isStableObstacleZone(obstacle_zone_state_.stable_zone)) {
        resolution.obstacle.zone = obstacle_zone_state_.stable_zone;
        resolution.stabilized = true;
        return resolution;
    }

    resolution.ambiguous = true;
    resolution.obstacle.nearest_m = assessment.snapshot.front_nearest_m;
    resolution.obstacle.angle_deg = assessment.snapshot.front_angle_deg;
    return resolution;
}

AutoAvoidController::Command AutoAvoidController::decideTooCloseFront(
    const SnapshotAssessment& assessment,
    const Judgment::FrontObstacleResult& front_obstacle,
    DebugInfo debug) {
    capturePathReferenceIfNeeded(assessment);
    TurnDirection direction = active_avoidance_state_.committed_direction;
    const TurnDirection raw_observed_direction =
        turnDirectionForObstacleZone(debug.raw_zone);
    const TurnDirection resolved_observed_direction =
        turnDirectionForObstacleZone(front_obstacle.zone);
    if (!isTurningDirection(direction)) {
        if (isTurningDirection(resolved_observed_direction)) {
            direction = resolved_observed_direction;
        } else {
            direction = chooseCenterTurnDirection(assessment.snapshot);
        }
        active_avoidance_state_.pending_override_direction = TurnDirection::Straight;
        active_avoidance_state_.pending_override_ticks = 0;
    } else {
        const bool preferred_target_is_not_wall_like =
            !assessment.snapshot.front_target_selection.valid ||
            !assessment.snapshot.front_target_selection.selected_front_cluster_wall_like;
        if (isTurningDirection(raw_observed_direction) &&
            raw_observed_direction != direction &&
            preferred_target_is_not_wall_like) {
            if (active_avoidance_state_.pending_override_direction != raw_observed_direction) {
                active_avoidance_state_.pending_override_direction = raw_observed_direction;
                active_avoidance_state_.pending_override_ticks = 1;
            } else {
                ++active_avoidance_state_.pending_override_ticks;
            }

            int confirm_ticks =
                std::max(1, config_.committed_direction_override_confirm_ticks);
            if (debug.resolved_zone_override_active) {
                confirm_ticks = std::max(1, confirm_ticks - 1);
            }
            if (active_avoidance_state_.pending_override_ticks >= confirm_ticks) {
                direction = raw_observed_direction;
                debug.committed_direction_override_active = true;
                debug.committed_direction_override_reason =
                    debug.resolved_zone_override_active ?
                        "resolved_zone_fast_switch" :
                        "raw_zone_conflict_confirmed";
            }
        } else {
            active_avoidance_state_.pending_override_direction = TurnDirection::Straight;
            active_avoidance_state_.pending_override_ticks = 0;
        }
    }

    lockAvoidanceTurn(direction, front_obstacle, assessment.snapshot);
    if (debug.committed_direction_override_active) {
        active_avoidance_state_.pending_override_direction = TurnDirection::Straight;
        active_avoidance_state_.pending_override_ticks = 0;
    }
    debug.state = active_avoidance_state_.stage;
    debug.direction = direction;
    debug.resolved_zone = front_obstacle.zone;

    double steering_angle_deg = 0.0;
    bool command_target_yaw_valid = false;
    double heading_correction_deg = 0.0;

    if (assessment.snapshot.imu_valid &&
        active_avoidance_state_.committed_target_yaw_valid &&
        std::isfinite(assessment.snapshot.yaw_deg)) {
        command_target_yaw_valid = true;
        const double command_target_yaw_deg =
            active_avoidance_state_.committed_target_yaw_deg;
        heading_correction_deg = avoidanceHeadingCorrectionDeg(
            assessment.snapshot.yaw_deg,
            command_target_yaw_deg,
            assessment.safety_front_m);
        steering_angle_deg = heading_correction_deg;
        debug.reason_code = DecisionReason::FrontAvoidanceImu;
        debug.used_imu_heading = true;
    } else {
        steering_angle_deg = fallbackSteeringAngleDeg(
            EncoderFallbackKind::FrontObstacleLinear,
            direction,
            &front_obstacle,
            debug);
        debug.reason_code = DecisionReason::FrontAvoidanceEncoderFallback;
    }

    const bool boundary_tail_blocking =
        hasCommittedAvoidanceTurn() && !tailClearanceComplete(assessment.snapshot);
    const auto boundary_recovery = boundaryRecoveryDecision(
        assessment,
        active_avoidance_state_.stage,
        direction,
        steering_angle_deg,
        0.0,
        boundary_tail_blocking);
    applyBoundaryRecoveryDebug(boundary_recovery, debug);
    steering_angle_deg =
        boundary_recovery.adjusted_main_steering_deg +
        boundary_recovery.correction_deg;
    steering_angle_deg = smoothSteeringAngleDeg(
        steering_angle_deg,
        avoidanceSteeringSlewDegPerTick(assessment.safety_front_m));
    steering_angle_deg =
        applyBoundarySteeringGuardDeg(steering_angle_deg, assessment.snapshot, &debug);
    if (command_target_yaw_valid) {
        heading_correction_deg = steering_angle_deg;
    }

    const int speed_cm_s = frontSpeedCmS(assessment.safety_front_m);
    rememberDriveSpeed(speed_cm_s);
    return driveCommand(
        direction,
        speed_cm_s,
        steering_angle_deg,
        assessment.snapshot,
        command_target_yaw_valid,
        heading_correction_deg,
        debug);
}

AutoAvoidController::Command AutoAvoidController::decideActiveAvoidanceStage(
    const SnapshotAssessment& assessment,
    const Judgment::FrontObstacleResult& front_obstacle,
    DebugInfo debug) {
    if (active_avoidance_state_.stage == AvoidanceStage::Turning) {
        enterClearanceHold();
    }

    Command stage_command;
    if (active_avoidance_state_.stage == AvoidanceStage::ClearanceHold) {
        if (tryContinueClearanceHoldStage(assessment, debug, stage_command)) {
            return stage_command;
        }
        active_avoidance_state_.stage = AvoidanceStage::ReturnHeading;
        active_avoidance_state_.return_heading_ticks = 0;
        active_avoidance_state_.return_to_path_settle_ticks = 0;
        active_avoidance_state_.return_to_path_exit_ticks = 0;
        active_avoidance_state_.return_to_path_settling = false;
    }

    if (active_avoidance_state_.stage == AvoidanceStage::ReturnHeading) {
        if (tryContinueReturnHeadingStage(assessment, debug, stage_command)) {
            return stage_command;
        }
        resetAvoidanceState();
        clearPathReference(PathReferenceClearReason::AvoidanceCompleted);
    }

    return decideStraightDrive(assessment, front_obstacle, debug);
}

AutoAvoidController::Command AutoAvoidController::decideSectorBufferDuringActiveStage(
    const SnapshotAssessment& assessment,
    DebugInfo debug) {
    if (!hasCommittedAvoidanceTurn()) {
        resetSteeringSmoothing();
        debug.reason_code = DecisionReason::SectorBuffer;
        debug.fallback_reason = FallbackReason::SectorBuffer;
        return neutralSafeCommand(debug);
    }

    const bool clearance_hold_stage =
        active_avoidance_state_.stage == AvoidanceStage::ClearanceHold;
    const int speed_cm_s = std::min(
        frontSpeedCmS(assessment.safety_front_m),
        clearance_hold_stage ?
            std::min(
                config_.clearance_hold_speed_cm_s,
                config_.sector_buffer_continue_speed_cm_s) :
            config_.sector_buffer_continue_speed_cm_s);

    double steering_angle_deg =
        steeringAngleForDirection(
            active_avoidance_state_.committed_direction,
            config_.sector_buffer_continue_turn_angle_deg);
    bool command_target_yaw_valid = false;
    double heading_correction_deg = 0.0;

    debug.state = active_avoidance_state_.stage;
    debug.direction = active_avoidance_state_.committed_direction;
    debug.reason_code = DecisionReason::SectorBufferContinue;
    debug.sector_buffer_active_continue = true;

    if (assessment.snapshot.imu_valid &&
        active_avoidance_state_.committed_target_yaw_valid &&
        std::isfinite(assessment.snapshot.yaw_deg)) {
        command_target_yaw_valid = true;
        const double command_target_yaw_deg =
            active_avoidance_state_.committed_target_yaw_deg;
        heading_correction_deg =
            clearance_hold_stage ?
                clearanceHoldHeadingCorrectionDeg(
                    assessment.snapshot.yaw_deg,
                    command_target_yaw_deg) :
                avoidanceHeadingCorrectionDeg(
                    assessment.snapshot.yaw_deg,
                    command_target_yaw_deg,
                    assessment.safety_front_m);
        steering_angle_deg = heading_correction_deg;
        debug.used_imu_heading = true;
    }

    const bool boundary_tail_blocking =
        hasCommittedAvoidanceTurn() && !tailClearanceComplete(assessment.snapshot);
    const auto boundary_recovery = boundaryRecoveryDecision(
        assessment,
        active_avoidance_state_.stage,
        active_avoidance_state_.committed_direction,
        steering_angle_deg,
        0.0,
        boundary_tail_blocking);
    applyBoundaryRecoveryDebug(boundary_recovery, debug);
    steering_angle_deg =
        boundary_recovery.adjusted_main_steering_deg +
        boundary_recovery.correction_deg;
    steering_angle_deg = smoothSteeringAngleDeg(
        steering_angle_deg,
        clearance_hold_stage ?
            config_.clearance_hold_steering_slew_deg_per_tick :
            avoidanceSteeringSlewDegPerTick(assessment.safety_front_m));
    steering_angle_deg =
        applyBoundarySteeringGuardDeg(steering_angle_deg, assessment.snapshot, &debug);
    if (command_target_yaw_valid) {
        heading_correction_deg = steering_angle_deg;
    }

    rememberDriveSpeed(speed_cm_s);
    return driveCommand(
        active_avoidance_state_.committed_direction,
        speed_cm_s,
        steering_angle_deg,
        assessment.snapshot,
        command_target_yaw_valid,
        heading_correction_deg,
        debug);
}

bool AutoAvoidController::tryContinueClearanceHoldStage(
    const SnapshotAssessment& assessment,
    DebugInfo debug,
    Command& command) {
    const int speed_cm_s = std::min(
        frontSpeedCmS(assessment.safety_front_m),
        config_.clearance_hold_speed_cm_s);
    if (updateClearanceHoldAndCheckDone(assessment.snapshot, speed_cm_s)) {
        return false;
    }

    double steering_angle_deg = 0.0;
    bool command_target_yaw_valid = false;
    double heading_correction_deg = 0.0;

    debug.state = active_avoidance_state_.stage;
    debug.direction = active_avoidance_state_.committed_direction;
    if (assessment.snapshot.imu_valid &&
        active_avoidance_state_.committed_target_yaw_valid &&
        std::isfinite(assessment.snapshot.yaw_deg)) {
        command_target_yaw_valid = true;
        const double command_target_yaw_deg =
            active_avoidance_state_.committed_target_yaw_deg;
        heading_correction_deg = clearanceHoldHeadingCorrectionDeg(
            assessment.snapshot.yaw_deg,
            command_target_yaw_deg);
        steering_angle_deg = heading_correction_deg;
        debug.reason_code = DecisionReason::ClearanceHoldImu;
        debug.used_imu_heading = true;
    } else {
        steering_angle_deg = fallbackSteeringAngleDeg(
            EncoderFallbackKind::ClearanceHoldFixed,
            active_avoidance_state_.committed_direction,
            nullptr,
            debug);
        debug.reason_code = DecisionReason::ClearanceHoldEncoderFallback;
    }

    const bool boundary_tail_blocking =
        hasCommittedAvoidanceTurn() && !tailClearanceComplete(assessment.snapshot);
    const auto boundary_recovery = boundaryRecoveryDecision(
        assessment,
        active_avoidance_state_.stage,
        active_avoidance_state_.committed_direction,
        steering_angle_deg,
        0.0,
        boundary_tail_blocking);
    applyBoundaryRecoveryDebug(boundary_recovery, debug);
    steering_angle_deg =
        boundary_recovery.adjusted_main_steering_deg +
        boundary_recovery.correction_deg;
    steering_angle_deg = smoothSteeringAngleDeg(
        steering_angle_deg,
        config_.clearance_hold_steering_slew_deg_per_tick);
    steering_angle_deg =
        applyBoundarySteeringGuardDeg(steering_angle_deg, assessment.snapshot, &debug);
    if (command_target_yaw_valid) {
        heading_correction_deg = steering_angle_deg;
    }

    rememberDriveSpeed(speed_cm_s);
    command = driveCommand(
        active_avoidance_state_.committed_direction,
        speed_cm_s,
        steering_angle_deg,
        assessment.snapshot,
        command_target_yaw_valid,
        heading_correction_deg,
        debug);
    return true;
}

bool AutoAvoidController::tryContinueReturnHeadingStage(
    const SnapshotAssessment& assessment,
    DebugInfo debug,
    Command& command) {
    auto normalizedErrorRatio = [](double value, double scale) {
        if (!std::isfinite(value)) {
            return 0.0;
        }
        return std::clamp(
            std::abs(value) / std::max(1e-6, scale),
            0.0,
            1.0);
    };

    const int protect_ticks = std::max(0, config_.return_heading_min_hold_ticks);
    const bool protected_phase =
        active_avoidance_state_.return_heading_ticks < protect_ticks;
    const int protect_ticks_remaining =
        std::max(0, protect_ticks - active_avoidance_state_.return_heading_ticks);
    const int speed_cm_s = std::min(
        frontSpeedCmS(assessment.safety_front_m),
        config_.return_heading_speed_cm_s);
    const double reference_yaw_deg =
        path_reference_state_.valid ?
            path_reference_state_.reference_yaw_deg :
            assessment.snapshot.target_yaw_deg;
    const bool front_clear_enough = frontClearEnoughForPathRecovery(assessment.snapshot);
    const bool tail_clearance_complete = tailClearanceComplete(assessment.snapshot);
    const bool tail_clearance_blocking = !tail_clearance_complete;
    const double current_side_balance = sideBalanceFromAssessment(assessment);
    const double path_recovery_balance_error =
        path_reference_state_.valid ?
            current_side_balance - path_reference_state_.reference_side_balance :
            0.0;
    const double yaw_error_deg =
        normalizeAngleErrorDeg(reference_yaw_deg - assessment.snapshot.yaw_deg);
    const double fast_balance_error_m = std::max(
        config_.return_to_path_near_reference_balance_error_m,
        config_.return_to_path_fast_recenter_balance_error_m);
    const double fast_yaw_error_deg = std::max(
        config_.return_to_path_near_reference_yaw_error_deg,
        config_.return_to_path_fast_recenter_yaw_error_deg);
    const double near_reference_balance_error_m =
        std::max(0.0, config_.return_to_path_near_reference_balance_error_m);
    const double near_reference_yaw_error_deg =
        std::max(0.0, config_.return_to_path_near_reference_yaw_error_deg);
    const double settle_balance_error_m = std::max(
        std::max(0.0, config_.return_to_path_balance_tolerance_m),
        std::max(0.0, config_.return_to_path_settle_balance_error_m));
    const double settle_yaw_error_deg = std::max(
        std::max(0.0, config_.return_heading_tolerance_deg),
        std::max(0.0, config_.return_to_path_settle_yaw_error_deg));
    const double settling_hold_balance_error_m = std::max(
        settle_balance_error_m,
        std::max(0.0, config_.return_to_path_settling_hold_balance_error_m));
    const double settling_hold_yaw_error_deg = std::max(
        settle_yaw_error_deg,
        std::max(0.0, config_.return_to_path_settling_hold_yaw_error_deg));
    const bool path_near_reference =
        !path_reference_state_.valid ||
        std::abs(path_recovery_balance_error) <= near_reference_balance_error_m;
    const bool yaw_near_reference =
        std::abs(yaw_error_deg) <= near_reference_yaw_error_deg;
    const bool return_to_path_near_reference =
        path_near_reference && yaw_near_reference;
    const bool path_recovery_settled =
        !path_reference_state_.valid ||
        std::abs(path_recovery_balance_error) <= settle_balance_error_m;
    const bool return_to_path_can_settle =
        front_clear_enough &&
        path_recovery_settled &&
        std::abs(yaw_error_deg) <= settle_yaw_error_deg;
    const bool can_hold_settling =
        front_clear_enough &&
        (!path_reference_state_.valid ||
            std::abs(path_recovery_balance_error) <= settling_hold_balance_error_m) &&
        std::abs(yaw_error_deg) <= settling_hold_yaw_error_deg;
    const int settle_entry_ticks =
        std::max(1, config_.return_to_path_settle_entry_ticks);
    const bool was_settling = active_avoidance_state_.return_to_path_settling;
    if (return_to_path_can_settle) {
        ++active_avoidance_state_.return_to_path_settle_ticks;
    } else if (!was_settling || !can_hold_settling) {
        active_avoidance_state_.return_to_path_settle_ticks = 0;
    } else {
        active_avoidance_state_.return_to_path_settle_ticks = std::max(
            active_avoidance_state_.return_to_path_settle_ticks,
            settle_entry_ticks);
    }
    bool return_to_path_settling_active = was_settling;
    if (!return_to_path_settling_active &&
        active_avoidance_state_.return_to_path_settle_ticks >= settle_entry_ticks) {
        return_to_path_settling_active = true;
    }
    if (return_to_path_settling_active && !can_hold_settling) {
        return_to_path_settling_active = false;
        active_avoidance_state_.return_to_path_settle_ticks = 0;
    }
    active_avoidance_state_.return_to_path_settling = return_to_path_settling_active;
    const bool return_to_path_fast_recenter_active =
        !return_to_path_settling_active;
    const double balance_gap =
        path_reference_state_.valid ?
            normalizedErrorRatio(path_recovery_balance_error, fast_balance_error_m) :
            0.0;
    const double yaw_gap =
        normalizedErrorRatio(yaw_error_deg, fast_yaw_error_deg);
    const double return_to_path_progress_score = std::clamp(
        1.0 - (path_reference_state_.valid ? (balance_gap * 0.65 + yaw_gap * 0.35) : yaw_gap),
        0.0,
        1.0);
    const bool yaw_recovery_retained_by_path =
        path_reference_state_.valid &&
        std::abs(path_recovery_balance_error) >
            std::max(
                settle_balance_error_m,
                near_reference_balance_error_m) &&
        std::abs(yaw_error_deg) <= fast_yaw_error_deg;
    double yaw_recovery_dynamic_gain =
        std::max(0.0, config_.return_heading_kp) *
        (return_to_path_fast_recenter_active ?
            std::max(0.0, config_.return_to_path_yaw_fast_gain_scale) :
            std::max(0.0, config_.return_to_path_yaw_settling_gain_scale));
    if (protected_phase) {
        yaw_recovery_dynamic_gain *= 1.10;
    }
    if (yaw_recovery_retained_by_path) {
        yaw_recovery_dynamic_gain *=
            std::max(0.0, config_.return_to_path_yaw_retained_gain_scale);
    }
    const double yaw_gain_shape =
        return_to_path_settling_active ?
            (0.28 + 0.72 * yaw_gap) :
            (0.85 + 0.35 * std::max(yaw_gap, balance_gap));
    yaw_recovery_dynamic_gain *= yaw_gain_shape;
    if (return_to_path_settling_active && path_recovery_settled) {
        yaw_recovery_dynamic_gain *= 0.75;
    }
    const double return_heading_max_correction_deg =
        std::min(
            Judgment::kMaxSteeringAngleDeg,
            config_.straight_heading_max_correction_deg +
                (protected_phase ? 0.8 : 0.0) +
                (return_to_path_fast_recenter_active ? 1.0 : 0.0));
    double yaw_recovery_correction_deg = headingCorrectionDegForTarget(
        assessment.snapshot.yaw_deg,
        reference_yaw_deg,
        yaw_recovery_dynamic_gain,
        return_heading_max_correction_deg,
        return_to_path_settling_active ?
            near_reference_yaw_error_deg :
            0.0);
    const bool path_recovery_ready =
        path_reference_state_.valid && front_clear_enough;
    double path_recovery_fast_recenter_boost = 0.0;
    double path_recovery_dynamic_gain = 0.0;
    double path_recovery_correction_deg = 0.0;
    if (path_recovery_ready) {
        const double fast_boost_ratio =
            return_to_path_fast_recenter_active ?
                (1.0 - return_to_path_progress_score) *
                    std::max(0.0, config_.return_to_path_balance_fast_boost_ratio) :
                0.0;
        const double gain_scale =
            return_to_path_fast_recenter_active ?
                std::max(0.0, config_.return_to_path_balance_fast_gain_scale) :
                std::max(0.0, config_.return_to_path_balance_settling_gain_scale);
        const double path_error_gain_scale =
            return_to_path_settling_active ?
                (0.25 + 0.75 * balance_gap) :
                (0.55 + 0.85 * balance_gap);
        path_recovery_fast_recenter_boost = fast_boost_ratio;
        path_recovery_dynamic_gain =
            std::max(0.0, config_.return_to_path_balance_gain_deg_per_m) *
            gain_scale *
            path_error_gain_scale *
            (1.0 + fast_boost_ratio);
        const double path_recovery_max_correction_deg =
            return_to_path_fast_recenter_active ?
                std::max(
                    std::max(0.0, config_.return_to_path_balance_max_correction_deg),
                    std::max(0.0, config_.return_to_path_balance_fast_max_correction_deg)) :
                std::max(0.0, config_.return_to_path_balance_max_correction_deg);
        path_recovery_correction_deg = pathRecoveryCorrectionDeg(
            assessment,
            path_recovery_dynamic_gain,
            path_recovery_max_correction_deg,
            debug);
    }
    double combined_return_correction_deg =
        yaw_recovery_correction_deg + path_recovery_correction_deg;
    bool combined_return_correction_limited_by_tail = false;

    if (tail_clearance_blocking && hasCommittedAvoidanceTurn()) {
        const bool opposite_pull =
            (active_avoidance_state_.committed_direction == TurnDirection::Left &&
                combined_return_correction_deg > 0.0) ||
            (active_avoidance_state_.committed_direction == TurnDirection::Right &&
                combined_return_correction_deg < 0.0);
        if (opposite_pull && std::abs(combined_return_correction_deg) > 0.001) {
            const double tail_limit_base_scale = std::clamp(
                config_.return_to_path_tail_blocking_limit_scale,
                0.0,
                1.0);
            const double tail_release_progress_score = std::clamp(
                config_.return_to_path_tail_release_progress_score,
                0.0,
                0.98);
            const double tail_release_limit_scale = std::clamp(
                std::max(
                    tail_limit_base_scale,
                    std::max(0.0, config_.return_to_path_tail_release_limit_scale)),
                0.0,
                1.0);
            double tail_limit_scale = tail_limit_base_scale;
            if (return_to_path_progress_score > tail_release_progress_score) {
                const double release_ratio =
                    (return_to_path_progress_score - tail_release_progress_score) /
                    std::max(1e-6, 1.0 - tail_release_progress_score);
                tail_limit_scale = std::max(
                    tail_limit_scale,
                    tail_limit_base_scale +
                        release_ratio * (tail_release_limit_scale - tail_limit_base_scale));
            }
            if (return_to_path_settling_active) {
                tail_limit_scale = std::max(tail_limit_scale, tail_release_limit_scale);
            }
            double limited_magnitude =
                std::abs(combined_return_correction_deg) *
                tail_limit_scale;
            const double tail_min_correction_deg =
                return_to_path_fast_recenter_active ?
                    std::max(0.0, config_.return_to_path_tail_blocking_min_correction_deg) :
                    std::min(
                        std::max(0.0, config_.return_to_path_exit_correction_deg),
                        std::max(
                            0.0,
                            config_.return_to_path_tail_blocking_min_correction_deg) *
                            0.5);
            if (tail_min_correction_deg > 0.0) {
                limited_magnitude = std::max(
                    limited_magnitude,
                    std::min(
                        std::abs(combined_return_correction_deg),
                        tail_min_correction_deg));
            }
            if (limited_magnitude + 1e-6 < std::abs(combined_return_correction_deg)) {
                combined_return_correction_deg =
                    std::copysign(limited_magnitude, combined_return_correction_deg);
                combined_return_correction_limited_by_tail = true;
            }
        }
    }

    const bool combined_return_correction_small =
        std::abs(combined_return_correction_deg) <=
        std::max(0.0, config_.return_to_path_exit_correction_deg);
    const bool exit_candidate =
        front_clear_enough &&
        return_to_path_settling_active &&
        return_to_path_near_reference &&
        path_recovery_settled &&
        tail_clearance_complete &&
        combined_return_correction_small;
    if (exit_candidate) {
        ++active_avoidance_state_.return_to_path_exit_ticks;
    } else {
        active_avoidance_state_.return_to_path_exit_ticks = 0;
    }
    const int exit_confirm_ticks =
        std::max(1, config_.return_to_path_settle_confirm_ticks);
    const bool exit_to_idle_ready =
        active_avoidance_state_.return_to_path_exit_ticks >= exit_confirm_ticks;
    std::string return_to_path_blocked_reason = "none";
    if (!return_to_path_settling_active) {
        if (!front_clear_enough) {
            return_to_path_blocked_reason = "front_not_clear";
        } else if (!return_to_path_can_settle) {
            if (std::abs(yaw_error_deg) > settle_yaw_error_deg) {
                return_to_path_blocked_reason = "yaw_not_ready_for_settling";
            } else if (path_reference_state_.valid && !path_recovery_settled) {
                return_to_path_blocked_reason = "path_not_ready_for_settling";
            } else {
                return_to_path_blocked_reason = "waiting_for_settling_window";
            }
        } else if (active_avoidance_state_.return_to_path_settle_ticks < settle_entry_ticks) {
            return_to_path_blocked_reason = "settle_confirming";
        }
    } else {
        if (!front_clear_enough) {
            return_to_path_blocked_reason = "front_not_clear";
        } else if (!return_to_path_near_reference) {
            if (!yaw_near_reference) {
                return_to_path_blocked_reason = "yaw_not_near_reference";
            } else if (path_reference_state_.valid && !path_near_reference) {
                return_to_path_blocked_reason = "path_not_near_reference";
            }
        } else if (!combined_return_correction_small) {
            return_to_path_blocked_reason = "return_correction_still_large";
        } else if (!tail_clearance_complete) {
            return_to_path_blocked_reason = "tail_clearance_blocking";
        } else if (!exit_to_idle_ready) {
            return_to_path_blocked_reason = "exit_confirming";
        }
    }
    if (return_to_path_blocked_reason == "none" &&
        combined_return_correction_limited_by_tail &&
        tail_clearance_blocking) {
        return_to_path_blocked_reason = "tail_limiting_return";
    }

    debug.state = active_avoidance_state_.stage;
    debug.direction = TurnDirection::Straight;
    debug.reason_code = DecisionReason::ReturnHeading;
    debug.return_to_path_active = true;
    debug.return_heading_protected = protected_phase;
    debug.return_heading_protect_ticks_remaining = protect_ticks_remaining;
    debug.path_reference_valid = path_reference_state_.valid;
    debug.reference_yaw_deg =
        path_reference_state_.valid ? path_reference_state_.reference_yaw_deg : 0.0;
    debug.reference_side_balance =
        path_reference_state_.valid ? path_reference_state_.reference_side_balance : 0.0;
    debug.reference_left_distance_m =
        path_reference_state_.valid ? path_reference_state_.reference_left_distance_m : 0.0;
    debug.reference_right_distance_m =
        path_reference_state_.valid ? path_reference_state_.reference_right_distance_m : 0.0;
    debug.return_to_path_phase =
        return_to_path_fast_recenter_active ? "fast_recenter" : "settling";
    debug.return_to_path_fast_recenter_active = return_to_path_fast_recenter_active;
    debug.return_to_path_settling_active = return_to_path_settling_active;
    debug.return_to_path_can_settle = return_to_path_can_settle;
    debug.return_to_path_blocked_reason = return_to_path_blocked_reason;
    debug.yaw_recovery_correction_deg = yaw_recovery_correction_deg;
    debug.yaw_recovery_dynamic_gain = yaw_recovery_dynamic_gain;
    debug.yaw_recovery_retained_by_path = yaw_recovery_retained_by_path;
    debug.yaw_recovery_final_deg = yaw_recovery_correction_deg;
    debug.path_recovery_correction_deg = path_recovery_correction_deg;
    debug.path_recovery_balance_error = path_recovery_balance_error;
    debug.path_recovery_dynamic_gain = path_recovery_dynamic_gain;
    debug.path_recovery_fast_recenter_boost = path_recovery_fast_recenter_boost;
    debug.path_recovery_final_deg = path_recovery_correction_deg;
    debug.combined_return_correction_deg = combined_return_correction_deg;
    debug.combined_return_correction_limited_by_tail =
        combined_return_correction_limited_by_tail;
    debug.return_to_path_progress_score = return_to_path_progress_score;
    debug.return_to_path_near_reference = return_to_path_near_reference;
    debug.tail_clearance_complete = tail_clearance_complete;
    debug.tail_clearance_blocking = tail_clearance_blocking;
    debug.path_recovery_ready = path_recovery_ready;
    debug.path_recovery_settled = path_recovery_settled;
    debug.exit_to_idle_ready = exit_to_idle_ready;

    if (debug.exit_to_idle_ready) {
        return false;
    }

    const auto boundary_recovery = boundaryRecoveryDecision(
        assessment,
        active_avoidance_state_.stage,
        TurnDirection::Straight,
        combined_return_correction_deg,
        path_recovery_correction_deg,
        tail_clearance_blocking);
    applyBoundaryRecoveryDebug(boundary_recovery, debug);
    double steering_angle_deg = smoothSteeringAngleDeg(
        boundary_recovery.adjusted_main_steering_deg +
            boundary_recovery.correction_deg,
        config_.return_heading_steering_slew_deg_per_tick);
    steering_angle_deg =
        applyBoundarySteeringGuardDeg(steering_angle_deg, assessment.snapshot, &debug);
    debug.used_imu_heading =
        assessment.snapshot.imu_valid && std::abs(steering_angle_deg) > 0.001;
    rememberDriveSpeed(speed_cm_s);
    command = driveCommand(
        TurnDirection::Straight,
        speed_cm_s,
        steering_angle_deg,
        assessment.snapshot,
        std::isfinite(reference_yaw_deg),
        steering_angle_deg,
        debug);
    ++active_avoidance_state_.return_heading_ticks;
    return true;
}

AutoAvoidController::Command AutoAvoidController::decideStraightDrive(
    const SnapshotAssessment& assessment,
    const Judgment::FrontObstacleResult& front_obstacle_input,
    DebugInfo debug) {
    auto front_obstacle = front_obstacle_input;
    if (isFrontZoneBuffer(front_obstacle)) {
        if (isStableObstacleZone(obstacle_zone_state_.stable_zone)) {
            front_obstacle.zone = obstacle_zone_state_.stable_zone;
            debug.zone_stabilized = true;
        } else {
            resetSteeringSmoothing();
            debug.reason_code = DecisionReason::FrontZoneBuffer;
            debug.fallback_reason = FallbackReason::FrontZoneBuffer;
            debug.zone_ambiguous = true;
            return neutralSafeCommand(debug);
        }
    } else {
        bool stabilized = false;
        front_obstacle = stabilizeFrontObstacleZone(front_obstacle, stabilized);
        debug.zone_stabilized = stabilized;
    }
    debug.resolved_zone = front_obstacle.zone;

    const int speed_cm_s = frontSpeedCmS(assessment.safety_front_m);
    double heading_correction_deg = headingCorrectionDeg(
        assessment.snapshot,
        config_.caution_heading_max_correction_deg);
    heading_correction_deg += lateralBalanceCorrectionDeg(
        assessment,
        config_.lateral_balance_max_correction_deg,
        debug);
    const auto boundary_recovery = boundaryRecoveryDecision(
        assessment,
        active_avoidance_state_.stage,
        TurnDirection::Straight,
        heading_correction_deg,
        0.0,
        false);
    applyBoundaryRecoveryDebug(boundary_recovery, debug);
    heading_correction_deg =
        boundary_recovery.adjusted_main_steering_deg +
        boundary_recovery.correction_deg;
    double steering_angle_deg = smoothSteeringAngleDeg(
        heading_correction_deg,
        config_.straight_steering_slew_deg_per_tick);
    steering_angle_deg =
        applyBoundarySteeringGuardDeg(steering_angle_deg, assessment.snapshot, &debug);
    heading_correction_deg = steering_angle_deg;
    debug.state = active_avoidance_state_.stage;
    debug.direction = TurnDirection::Straight;
    debug.reason_code = DecisionReason::StraightDrive;
    debug.used_imu_heading =
        assessment.snapshot.imu_valid &&
        assessment.snapshot.target_yaw_valid &&
        std::abs(heading_correction_deg) > 0.001;
    rememberDriveSpeed(speed_cm_s);
    return driveCommand(
        TurnDirection::Straight,
        speed_cm_s,
        steering_angle_deg,
        assessment.snapshot,
        assessment.snapshot.target_yaw_valid,
        heading_correction_deg,
        debug);
}

AutoAvoidController::SensorSnapshot AutoAvoidController::filteredSnapshot(
    const SensorSnapshot& snapshot) {
    if (!snapshot.lidar_valid) {
        resetFilteredLidarHistory();
        resetFrontSpikeFilter();
        return snapshot;
    }

    SensorSnapshot filtered = snapshot;
    filtered = normalizedSnapshot(filtered);
    filtered.negative_front =
        filteredSectorSample(snapshot.negative_front, negative_front_history_);
    filtered.front = filteredSectorSample(snapshot.front, front_history_);
    filtered.positive_front =
        filteredSectorSample(snapshot.positive_front, positive_front_history_);
    filtered.sector_buffer =
        filteredSectorSample(snapshot.sector_buffer, sector_buffer_history_);
    const auto spike_filter_result = applyFrontSpikeFilter(
        filtered.front,
        filtered.negative_front,
        filtered.positive_front);
    filtered.front = spike_filter_result.sample;
    return normalizedSnapshot(filtered);
}

AutoAvoidController::SectorSample AutoAvoidController::filteredSectorSample(
    const SectorSample& sample,
    std::deque<SectorSample>& history) const {
    const int filter_window = std::max(1, config_.lidar_filter_window);
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

    std::vector<SectorSample> ordered(history.begin(), history.end());
    std::sort(
        ordered.begin(),
        ordered.end(),
        [](const SectorSample& lhs, const SectorSample& rhs) {
            return lhs.nearest_m < rhs.nearest_m;
        });
    return ordered[ordered.size() / 2];
}

void AutoAvoidController::resetFilteredLidarHistory() {
    negative_front_history_.clear();
    front_history_.clear();
    positive_front_history_.clear();
    sector_buffer_history_.clear();
}

void AutoAvoidController::resetFrontSpikeFilter() {
    front_spike_filter_state_ = FrontSpikeFilterState{};
}

AutoAvoidController::FrontSpikeFilterResult AutoAvoidController::applyFrontSpikeFilter(
    const SectorSample& front,
    const SectorSample& negative_front,
    const SectorSample& positive_front) {
    front_spike_filter_state_.suppressed_this_cycle = false;
    FrontSpikeFilterResult result;
    result.sample = front;
    if (!front.valid || !std::isfinite(front.nearest_m) || !std::isfinite(front.nearest_angle_deg)) {
        resetFrontSpikeFilter();
        result.sample = front;
        return result;
    }

    const bool side_clear =
        (!negative_front.valid || !std::isfinite(negative_front.nearest_m) ||
            negative_front.nearest_m >= config_.front_spike_side_clear_distance_m) &&
        (!positive_front.valid || !std::isfinite(positive_front.nearest_m) ||
            positive_front.nearest_m >= config_.front_spike_side_clear_distance_m);
    const bool suspicious_center_spike =
        front_spike_filter_state_.has_last_stable_sample &&
        std::isfinite(front_spike_filter_state_.last_stable_sample.nearest_m) &&
        front_spike_filter_state_.last_stable_sample.nearest_m >
            config_.avoidance_turn_max_distance_m &&
        std::abs(front.nearest_angle_deg) <= config_.front_spike_center_half_width_deg &&
        front.nearest_m + config_.front_spike_jump_distance_m <
            front_spike_filter_state_.last_stable_sample.nearest_m &&
        front.support_points > 0 &&
        front.support_points <= config_.front_spike_max_support_points &&
        side_clear;

    if (suspicious_center_spike) {
        const int confirm_ticks =
            front.nearest_m <= config_.emergency_stop_distance_m ?
                1 :
                std::max(1, config_.front_spike_confirm_ticks);
        const bool similar_pending_spike =
            front_spike_filter_state_.has_pending_sample &&
            std::isfinite(front_spike_filter_state_.pending_sample.nearest_m) &&
            std::isfinite(front_spike_filter_state_.pending_sample.nearest_angle_deg) &&
            std::abs(front.nearest_m - front_spike_filter_state_.pending_sample.nearest_m) <=
                0.30 &&
            std::abs(
                front.nearest_angle_deg -
                front_spike_filter_state_.pending_sample.nearest_angle_deg) <= 8.0;
        if (!similar_pending_spike) {
            front_spike_filter_state_.pending_sample = front;
            front_spike_filter_state_.has_pending_sample = true;
            front_spike_filter_state_.pending_ticks = 1;
        } else {
            ++front_spike_filter_state_.pending_ticks;
        }

        if (front_spike_filter_state_.pending_ticks < confirm_ticks) {
            front_spike_filter_state_.suppressed_this_cycle = true;
            result.suppressed = true;
            result.sample = front_spike_filter_state_.last_stable_sample;
            return result;
        }
    } else {
        front_spike_filter_state_.has_pending_sample = false;
        front_spike_filter_state_.pending_sample = SectorSample{};
        front_spike_filter_state_.pending_ticks = 0;
    }

    front_spike_filter_state_.has_last_stable_sample = true;
    front_spike_filter_state_.last_stable_sample = front;
    front_spike_filter_state_.has_pending_sample = false;
    front_spike_filter_state_.pending_sample = SectorSample{};
    front_spike_filter_state_.pending_ticks = 0;
    result.sample = front;
    return result;
}

void AutoAvoidController::resetMotionHysteresis() {
    motion_hysteresis_state_ = MotionHysteresisState{};
}

bool AutoAvoidController::emergencyStopActive(double front_nearest_m) {
    if (!std::isfinite(front_nearest_m)) {
        motion_hysteresis_state_.emergency_stop_latched = false;
        return false;
    }

    const double release_distance = std::max(
        config_.emergency_stop_release_distance_m,
        config_.emergency_stop_distance_m);
    if (motion_hysteresis_state_.emergency_stop_latched) {
        if (front_nearest_m > release_distance) {
            motion_hysteresis_state_.emergency_stop_latched = false;
        }
    } else if (front_nearest_m <= config_.emergency_stop_distance_m) {
        motion_hysteresis_state_.emergency_stop_latched = true;
    }
    return motion_hysteresis_state_.emergency_stop_latched;
}

bool AutoAvoidController::avoidanceTurnActive(double front_nearest_m) {
    if (!std::isfinite(front_nearest_m)) {
        motion_hysteresis_state_.avoidance_turn_latched = false;
        return false;
    }

    const double release_distance = std::max(
        config_.avoidance_release_distance_m,
        config_.avoidance_turn_max_distance_m);
    if (motion_hysteresis_state_.avoidance_turn_latched) {
        if (front_nearest_m >= release_distance) {
            motion_hysteresis_state_.avoidance_turn_latched = false;
        }
    } else if (front_nearest_m < config_.avoidance_turn_max_distance_m) {
        motion_hysteresis_state_.avoidance_turn_latched = true;
    }
    return motion_hysteresis_state_.avoidance_turn_latched;
}

bool AutoAvoidController::shouldReplanFrontDuringActiveAvoidance(
    const SensorSnapshot& snapshot,
    const Judgment::FrontObstacleResult& front_obstacle) const {
    if (active_avoidance_state_.stage != AvoidanceStage::ClearanceHold &&
        active_avoidance_state_.stage != AvoidanceStage::ReturnHeading) {
        return false;
    }

    if (!snapshot.front.valid ||
        !std::isfinite(snapshot.front.nearest_m) ||
        !front_obstacle.valid ||
        !std::isfinite(front_obstacle.nearest_m) ||
        !std::isfinite(front_obstacle.angle_deg)) {
        return false;
    }

    double replan_distance_m = std::max(
        config_.avoidance_turn_max_distance_m,
        config_.active_stage_front_replan_distance_m);
    if (active_avoidance_state_.stage == AvoidanceStage::ReturnHeading &&
        active_avoidance_state_.return_heading_ticks <
            std::max(0, config_.return_heading_min_hold_ticks)) {
        replan_distance_m = config_.avoidance_turn_max_distance_m;
    }

    if (snapshot.front.nearest_m > replan_distance_m) {
        return false;
    }

    if (std::abs(front_obstacle.angle_deg) <=
        std::max(0.0, config_.active_stage_front_replan_center_half_width_deg)) {
        return true;
    }

    return front_obstacle.zone == Judgment::FrontObstacleZone::Center;
}

void AutoAvoidController::resetAvoidanceState() {
    active_avoidance_state_ = ActiveAvoidanceState{};
}

bool AutoAvoidController::hasCommittedAvoidanceTurn() const {
    return isTurningDirection(active_avoidance_state_.committed_direction);
}

AutoAvoidController::TurnDirection AutoAvoidController::turnDirectionForObstacleZone(
    Judgment::FrontObstacleZone zone) const {
    switch (zone) {
        case Judgment::FrontObstacleZone::Left:
            return TurnDirection::Right;
        case Judgment::FrontObstacleZone::Right:
            return TurnDirection::Left;
        case Judgment::FrontObstacleZone::Center:
            return TurnDirection::Straight;
        case Judgment::FrontObstacleZone::Unknown:
        default:
            return TurnDirection::Stop;
    }
}

Judgment::FrontObstacleZone AutoAvoidController::obstacleSideZoneForTurnDirection(
    TurnDirection direction) const {
    switch (direction) {
        case TurnDirection::Left:
            return Judgment::FrontObstacleZone::Right;
        case TurnDirection::Right:
            return Judgment::FrontObstacleZone::Left;
        case TurnDirection::Straight:
        case TurnDirection::Stop:
        default:
            return Judgment::FrontObstacleZone::Unknown;
    }
}

const AutoAvoidController::SectorSample* AutoAvoidController::sectorForZone(
    const SensorSnapshot& snapshot,
    Judgment::FrontObstacleZone zone) const {
    switch (zone) {
        case Judgment::FrontObstacleZone::Left:
            return &snapshot.negative_front;
        case Judgment::FrontObstacleZone::Right:
            return &snapshot.positive_front;
        case Judgment::FrontObstacleZone::Center:
        case Judgment::FrontObstacleZone::Unknown:
        default:
            return nullptr;
    }
}

void AutoAvoidController::lockAvoidanceTurn(
    TurnDirection direction,
    const Judgment::FrontObstacleResult& front_obstacle,
    const SensorSnapshot& snapshot) {
    if (!isTurningDirection(direction)) {
        return;
    }

    const bool direction_changed = active_avoidance_state_.committed_direction != direction;
    if (direction_changed || active_avoidance_state_.stage == AvoidanceStage::Idle) {
        active_avoidance_state_.committed_direction = direction;
        active_avoidance_state_.committed_obstacle_side_zone =
            obstacleSideZoneForTurnDirection(direction);
        active_avoidance_state_.committed_target_yaw_valid = false;
        active_avoidance_state_.committed_target_yaw_deg = 0.0;
        active_avoidance_state_.tail_side_obstacle_seen = false;
        active_avoidance_state_.tail_side_clear_phase_started = false;
        active_avoidance_state_.clearance_hold_progress_m = 0.0;
        active_avoidance_state_.tail_post_clear_progress_m = 0.0;
        active_avoidance_state_.return_heading_ticks = 0;
        active_avoidance_state_.return_to_path_settle_ticks = 0;
        active_avoidance_state_.return_to_path_exit_ticks = 0;
        active_avoidance_state_.return_to_path_settling = false;
        active_avoidance_state_.pending_override_direction = TurnDirection::Straight;
        active_avoidance_state_.pending_override_ticks = 0;
    }

    active_avoidance_state_.stage = AvoidanceStage::Turning;

    const SectorSample* obstacle_side_sector =
        sectorForZone(snapshot, active_avoidance_state_.committed_obstacle_side_zone);
    if (obstacle_side_sector &&
        obstacle_side_sector->valid &&
        std::isfinite(obstacle_side_sector->nearest_m) &&
        obstacle_side_sector->nearest_m <= tailClearanceSideCaptureDistanceM()) {
        active_avoidance_state_.tail_side_obstacle_seen = true;
    }

    if (!snapshot.imu_valid ||
        !snapshot.target_yaw_valid ||
        !std::isfinite(snapshot.target_yaw_deg)) {
        active_avoidance_state_.committed_target_yaw_valid = false;
        active_avoidance_state_.committed_target_yaw_deg = 0.0;
        return;
    }

    const double target_yaw_delta_deg =
        linearAvoidanceTargetYawDeltaDeg(front_obstacle);
    const double candidate_signed_delta_deg =
        direction == TurnDirection::Left ? target_yaw_delta_deg : -target_yaw_delta_deg;

    double committed_signed_delta_deg = candidate_signed_delta_deg;
    if (active_avoidance_state_.committed_target_yaw_valid) {
        committed_signed_delta_deg = normalizeAngleErrorDeg(
            active_avoidance_state_.committed_target_yaw_deg - snapshot.target_yaw_deg);
        if (direction == TurnDirection::Left) {
            committed_signed_delta_deg = std::max(
                committed_signed_delta_deg,
                candidate_signed_delta_deg);
        } else {
            committed_signed_delta_deg = std::min(
                committed_signed_delta_deg,
                candidate_signed_delta_deg);
        }
    }

    active_avoidance_state_.committed_target_yaw_valid = true;
    active_avoidance_state_.committed_target_yaw_deg = normalizeAngleErrorDeg(
        snapshot.target_yaw_deg + committed_signed_delta_deg);
}

void AutoAvoidController::enterClearanceHold() {
    if (!hasCommittedAvoidanceTurn()) {
        resetAvoidanceState();
        return;
    }

    active_avoidance_state_.stage = AvoidanceStage::ClearanceHold;
    active_avoidance_state_.clearance_hold_progress_m = 0.0;
    active_avoidance_state_.tail_post_clear_progress_m = 0.0;
    active_avoidance_state_.tail_side_clear_phase_started = false;
}

double AutoAvoidController::rearReachFromLidarM() const {
    return std::max(
        0.0,
        std::max(0.0, config_.lidar_x_from_base_link_m) +
            std::max(0.0, config_.vehicle_length_m * 0.5));
}

double AutoAvoidController::tailClearanceSideReleaseDistanceM() const {
    return std::max(
        Judgment::kVehicleBoundaryThresholdMeters,
        std::max(0.0, config_.vehicle_width_m) + std::max(0.0, config_.tail_clearance_margin_m));
}

double AutoAvoidController::tailClearanceSideCaptureDistanceM() const {
    return tailClearanceSideReleaseDistanceM() + 0.25;
}

double AutoAvoidController::tailClearanceMinHoldDistanceM() const {
    return std::max(0.35, rearReachFromLidarM() * 0.35);
}

double AutoAvoidController::tailClearancePostSideClearTravelM() const {
    return std::max(
        0.55,
        rearReachFromLidarM() -
            std::max(0.0, config_.vehicle_width_m) * 0.35 +
            std::max(0.0, config_.tail_clearance_margin_m));
}

double AutoAvoidController::estimatedTravelPerTickM(int speed_cm_s) const {
    constexpr double kAutoAvoidControlPeriodSeconds = 0.10;
    return std::max(0, speed_cm_s) * 0.01 * kAutoAvoidControlPeriodSeconds;
}

bool AutoAvoidController::updateClearanceHoldAndCheckDone(
    const SensorSnapshot& snapshot,
    int speed_cm_s) {
    const double traveled_m = estimatedTravelPerTickM(speed_cm_s);
    active_avoidance_state_.clearance_hold_progress_m += traveled_m;

    const SectorSample* obstacle_side_sector =
        sectorForZone(snapshot, active_avoidance_state_.committed_obstacle_side_zone);
    bool side_obstacle_present = false;
    if (obstacle_side_sector &&
        obstacle_side_sector->valid &&
        std::isfinite(obstacle_side_sector->nearest_m)) {
        if (obstacle_side_sector->nearest_m <= tailClearanceSideCaptureDistanceM()) {
            active_avoidance_state_.tail_side_obstacle_seen = true;
        }
        side_obstacle_present =
            obstacle_side_sector->nearest_m <= tailClearanceSideReleaseDistanceM();
    }

    if (side_obstacle_present) {
        active_avoidance_state_.tail_side_clear_phase_started = false;
        active_avoidance_state_.tail_post_clear_progress_m = 0.0;
    } else if (active_avoidance_state_.tail_side_obstacle_seen) {
        active_avoidance_state_.tail_side_clear_phase_started = true;
        active_avoidance_state_.tail_post_clear_progress_m += traveled_m;
    }

    const bool front_clear_enough =
        snapshot.front.valid &&
        std::isfinite(snapshot.front.nearest_m) &&
        snapshot.front.nearest_m >=
            std::max(config_.avoidance_release_distance_m, config_.medium_speed_min_distance_m);
    const bool min_hold_done =
        active_avoidance_state_.clearance_hold_progress_m >=
            tailClearanceMinHoldDistanceM();
    const bool side_clear_done =
        !active_avoidance_state_.tail_side_obstacle_seen ||
        (active_avoidance_state_.tail_side_clear_phase_started &&
            active_avoidance_state_.tail_post_clear_progress_m >=
                tailClearancePostSideClearTravelM());
    return front_clear_enough && min_hold_done && side_clear_done;
}

double AutoAvoidController::sideBalanceFromAssessment(
    const SnapshotAssessment& assessment) const {
    if (!std::isfinite(assessment.safety_negative_front_m) ||
        !std::isfinite(assessment.safety_positive_front_m)) {
        return 0.0;
    }
    return assessment.safety_positive_front_m - assessment.safety_negative_front_m;
}

bool AutoAvoidController::frontClearEnoughForPathRecovery(
    const SensorSnapshot& snapshot) const {
    return snapshot.front.valid &&
        std::isfinite(snapshot.front.nearest_m) &&
        snapshot.front.nearest_m >=
            std::max(config_.avoidance_release_distance_m, config_.medium_speed_min_distance_m);
}

bool AutoAvoidController::tailClearanceComplete(
    const SensorSnapshot& snapshot) const {
    const SectorSample* obstacle_side_sector =
        sectorForZone(snapshot, active_avoidance_state_.committed_obstacle_side_zone);
    bool side_obstacle_present = false;
    if (obstacle_side_sector &&
        obstacle_side_sector->valid &&
        std::isfinite(obstacle_side_sector->nearest_m)) {
        side_obstacle_present =
            obstacle_side_sector->nearest_m <= tailClearanceSideReleaseDistanceM();
    }

    const bool min_hold_done =
        active_avoidance_state_.clearance_hold_progress_m >=
            tailClearanceMinHoldDistanceM();
    const bool side_clear_done =
        !active_avoidance_state_.tail_side_obstacle_seen ||
        (active_avoidance_state_.tail_side_clear_phase_started &&
            active_avoidance_state_.tail_post_clear_progress_m >=
                tailClearancePostSideClearTravelM() &&
            !side_obstacle_present);
    return frontClearEnoughForPathRecovery(snapshot) && min_hold_done && side_clear_done;
}

double AutoAvoidController::pathRecoveryCorrectionDeg(
    const SnapshotAssessment& assessment,
    double gain_deg_per_m,
    double max_correction_deg,
    DebugInfo& debug) const {
    if (!path_reference_state_.valid) {
        return 0.0;
    }

    const double gain = std::max(0.0, gain_deg_per_m);
    const double max_correction = std::max(0.0, max_correction_deg);
    if (gain <= 0.0 || max_correction <= 0.0) {
        return 0.0;
    }

    const double current_side_balance = sideBalanceFromAssessment(assessment);
    const double balance_error_m =
        current_side_balance - path_reference_state_.reference_side_balance;
    const double correction_deg = std::clamp(
        gain * balance_error_m,
        -max_correction,
        max_correction);
    if (std::abs(correction_deg) < 0.05) {
        return 0.0;
    }

    debug.lateral_balance_active = true;
    debug.lateral_balance_correction_deg = correction_deg;
    return correction_deg;
}

double AutoAvoidController::normalizedAvoidanceDistanceRatio(
    double front_nearest_m) const {
    if (!std::isfinite(front_nearest_m)) {
        return 0.0;
    }

    const double turn_distance_m = std::max(
        config_.avoidance_turn_max_distance_m,
        config_.emergency_stop_distance_m + 1e-6);
    const double span_m = std::max(
        1e-6,
        turn_distance_m - config_.emergency_stop_distance_m);
    return std::clamp(
        (turn_distance_m - front_nearest_m) / span_m,
        0.0,
        1.0);
}

double AutoAvoidController::avoidanceHeadingCorrectionDeg(
    double yaw_deg,
    double target_yaw_deg,
    double front_nearest_m) const {
    const double ratio = normalizedAvoidanceDistanceRatio(front_nearest_m);
    const double kp =
        config_.avoidance_heading_kp_far +
        ratio * (config_.avoidance_heading_kp_near - config_.avoidance_heading_kp_far);
    const double max_correction_deg =
        config_.avoidance_heading_max_correction_far_deg +
        ratio * (config_.avoidance_heading_max_correction_near_deg -
            config_.avoidance_heading_max_correction_far_deg);
    return headingCorrectionDegForTarget(
        yaw_deg,
        target_yaw_deg,
        kp,
        max_correction_deg,
        config_.avoidance_heading_target_tolerance_deg);
}

double AutoAvoidController::clearanceHoldHeadingCorrectionDeg(
    double yaw_deg,
    double target_yaw_deg) const {
    return headingCorrectionDegForTarget(
        yaw_deg,
        target_yaw_deg,
        config_.clearance_hold_heading_kp,
        config_.clearance_hold_max_correction_deg,
        config_.avoidance_heading_target_tolerance_deg);
}

int AutoAvoidController::frontSpeedCmS(double front_nearest_m) const {
    const int current_speed_cm_s =
        has_last_drive_speed_ ? last_drive_speed_cm_s_ : config_.caution_speed_cm_s;
    if (!std::isfinite(front_nearest_m)) {
        return current_speed_cm_s;
    }
    if (front_nearest_m > config_.fast_speed_min_distance_m) {
        return config_.cruise_speed_cm_s;
    }
    if (front_nearest_m >= config_.medium_speed_max_distance_m) {
        return has_last_drive_speed_ ? last_drive_speed_cm_s_ : config_.caution_speed_cm_s;
    }
    if (front_nearest_m > config_.medium_speed_min_distance_m) {
        return config_.caution_speed_cm_s;
    }
    if (front_nearest_m >= config_.avoidance_turn_max_distance_m) {
        return has_last_drive_speed_ ? last_drive_speed_cm_s_ : config_.avoidance_speed_cm_s;
    }
    return config_.avoidance_speed_cm_s;
}

double AutoAvoidController::linearAvoidanceTargetYawDeltaDeg(
    const Judgment::FrontObstacleResult& front_obstacle) const {
    if (!std::isfinite(front_obstacle.angle_deg) ||
        !std::isfinite(front_obstacle.nearest_m)) {
        return 0.0;
    }

    const double front_half_width_deg =
        std::max(0.0, config_.linear_avoidance_front_half_width_deg);
    const double max_target_delta_deg = std::max(
        std::max(0.0, config_.center_turn_angle_deg),
        config_.avoidance_target_max_delta_deg);
    const double min_target_delta_deg = std::clamp(
        config_.avoidance_target_min_delta_deg,
        0.0,
        max_target_delta_deg);
    const double distance_weight = std::clamp(
        config_.avoidance_target_distance_weight,
        0.0,
        1.0);

    if (front_half_width_deg <= 0.0) {
        return max_target_delta_deg;
    }

    const double angle_from_center_deg =
        std::min(std::abs(front_obstacle.angle_deg), front_half_width_deg);
    const double center_intrusion_deg =
        std::max(0.0, front_half_width_deg - angle_from_center_deg);
    const double angle_intrusion_ratio = std::clamp(
        center_intrusion_deg / front_half_width_deg,
        0.0,
        1.0);
    const double distance_span_m =
        std::max(1e-6, config_.avoidance_turn_max_distance_m - config_.emergency_stop_distance_m);
    const double distance_intrusion_ratio = std::clamp(
        (config_.avoidance_turn_max_distance_m - front_obstacle.nearest_m) / distance_span_m,
        0.0,
        1.0);
    const double blended_intrusion_ratio =
        (1.0 - distance_weight) * angle_intrusion_ratio +
        distance_weight * distance_intrusion_ratio;
    const double combined_intrusion_ratio = std::clamp(
        std::max(distance_intrusion_ratio, blended_intrusion_ratio),
        0.0,
        1.0);

    // Closer obstacles should keep a larger yaw target, even when the nearest angle
    // sits near the side of the forward sector.
    return min_target_delta_deg +
        combined_intrusion_ratio * (max_target_delta_deg - min_target_delta_deg);
}

double AutoAvoidController::fallbackSteeringAngleDeg(
    EncoderFallbackKind kind,
    TurnDirection direction,
    const Judgment::FrontObstacleResult* front_obstacle,
    DebugInfo& debug) const {
    debug.used_encoder_fallback = true;
    debug.encoder_fallback_kind = kind;

    switch (kind) {
        case EncoderFallbackKind::FrontObstacleLinear: {
            if (!front_obstacle ||
                direction == TurnDirection::Straight ||
                direction == TurnDirection::Stop ||
                !std::isfinite(front_obstacle->angle_deg)) {
                return 0.0;
            }

            const double front_half_width_deg =
                std::max(0.0, config_.linear_avoidance_front_half_width_deg);
            const double angle_from_center_deg =
                std::min(std::abs(front_obstacle->angle_deg), front_half_width_deg);
            const double center_intrusion_deg =
                std::max(0.0, front_half_width_deg - angle_from_center_deg);
            const int encoder_per_10deg =
                std::max(0, config_.linear_avoidance_encoder_per_10deg);
            const int magnitude = Judgment::clampSteeringEncoder(static_cast<int>(std::lround(
                center_intrusion_deg * static_cast<double>(encoder_per_10deg) / 10.0)));
            const int steering_encoder =
                direction == TurnDirection::Left ? -magnitude : magnitude;
            return Judgment::steeringEncoderToAngleDeg(steering_encoder);
        }
        case EncoderFallbackKind::ClearanceHoldFixed:
            return steeringAngleForDirection(direction, config_.caution_turn_angle_deg);
        case EncoderFallbackKind::None:
        default:
            return 0.0;
    }
}

void AutoAvoidController::rememberDriveSpeed(int speed_cm_s) {
    has_last_drive_speed_ = true;
    last_drive_speed_cm_s_ = clampSpeed(speed_cm_s);
}

void AutoAvoidController::resetDriveSpeed() {
    has_last_drive_speed_ = false;
    last_drive_speed_cm_s_ = 0;
}

void AutoAvoidController::resetObstacleZoneStabilizer() {
    obstacle_zone_state_ = ObstacleZoneStabilizerState{};
}

Judgment::FrontObstacleResult AutoAvoidController::stabilizeFrontObstacleZone(
    const Judgment::FrontObstacleResult& front_obstacle,
    bool& stabilized,
    int confirm_ticks_override) {
    stabilized = false;
    if (!front_obstacle.valid || !isStableObstacleZone(front_obstacle.zone)) {
        resetObstacleZoneStabilizer();
        return front_obstacle;
    }

    if (!isStableObstacleZone(obstacle_zone_state_.stable_zone)) {
        obstacle_zone_state_.stable_zone = front_obstacle.zone;
        obstacle_zone_state_.pending_zone = Judgment::FrontObstacleZone::Unknown;
        obstacle_zone_state_.pending_ticks = 0;
        return front_obstacle;
    }

    if (front_obstacle.zone == obstacle_zone_state_.stable_zone) {
        obstacle_zone_state_.pending_zone = Judgment::FrontObstacleZone::Unknown;
        obstacle_zone_state_.pending_ticks = 0;
        return front_obstacle;
    }

    if (front_obstacle.zone != obstacle_zone_state_.pending_zone) {
        obstacle_zone_state_.pending_zone = front_obstacle.zone;
        obstacle_zone_state_.pending_ticks = 1;
    } else {
        ++obstacle_zone_state_.pending_ticks;
    }

    const int confirm_ticks = std::max(
        1,
        confirm_ticks_override > 0 ?
            confirm_ticks_override :
            config_.obstacle_zone_switch_confirm_ticks);
    if (obstacle_zone_state_.pending_ticks >= confirm_ticks) {
        obstacle_zone_state_.stable_zone = obstacle_zone_state_.pending_zone;
        obstacle_zone_state_.pending_zone = Judgment::FrontObstacleZone::Unknown;
        obstacle_zone_state_.pending_ticks = 0;
        return front_obstacle;
    }

    auto stabilized_obstacle = front_obstacle;
    stabilized_obstacle.zone = obstacle_zone_state_.stable_zone;
    stabilized = true;
    return stabilized_obstacle;
}

bool AutoAvoidController::isSectorBufferNearest(const SensorSnapshot& snapshot) const {
    if (!snapshot.sector_buffer.valid || !std::isfinite(snapshot.sector_buffer.nearest_m)) {
        return false;
    }

    const double buffer_nearest_m = snapshot.sector_buffer.nearest_m;
    const auto buffer_is_closer_than =
        [buffer_nearest_m](const SectorSample& sector) {
            return !sector.valid ||
                !std::isfinite(sector.nearest_m) ||
                buffer_nearest_m <= sector.nearest_m;
        };

    return buffer_is_closer_than(snapshot.negative_front) &&
        buffer_is_closer_than(snapshot.front) &&
        buffer_is_closer_than(snapshot.positive_front);
}

double AutoAvoidController::headingCorrectionDegForTarget(
    double yaw_deg,
    double target_yaw_deg,
    double kp,
    double max_correction_deg,
    double settle_tolerance_deg) const {
    if (!std::isfinite(yaw_deg) || !std::isfinite(target_yaw_deg)) {
        return 0.0;
    }

    const double yaw_error_deg = normalizeAngleErrorDeg(target_yaw_deg - yaw_deg);
    if (std::abs(yaw_error_deg) > config_.max_usable_yaw_error_deg) {
        return 0.0;
    }

    if (std::abs(yaw_error_deg) <= std::max(0.0, settle_tolerance_deg)) {
        return 0.0;
    }

    const double normalized_kp = std::max(0.0, kp);
    const double correction_limit = std::max(0.0, max_correction_deg);
    // ROS yaw normally increases counter-clockwise. In this vehicle command map,
    // negative steering means left and positive steering means right.
    return std::clamp(
        -normalized_kp * yaw_error_deg,
        -correction_limit,
        correction_limit);
}

double AutoAvoidController::headingCorrectionDeg(
    const SensorSnapshot& snapshot,
    double max_correction_deg) const {
    if (!snapshot.imu_valid || !snapshot.target_yaw_valid) {
        return 0.0;
    }
    return headingCorrectionDegForTarget(
        snapshot.yaw_deg,
        snapshot.target_yaw_deg,
        config_.heading_kp,
        max_correction_deg,
        config_.straight_heading_tolerance_deg);
}

double AutoAvoidController::lateralBalanceCorrectionDeg(
    const SnapshotAssessment& assessment,
    double max_correction_deg,
    DebugInfo& debug) const {
    if (!std::isfinite(assessment.safety_negative_front_m) ||
        !std::isfinite(assessment.safety_positive_front_m)) {
        return 0.0;
    }

    const double minimum_side_distance_m =
        std::max(
            Judgment::kVehicleBoundaryThresholdMeters,
            config_.lateral_balance_min_side_distance_m);
    if (assessment.safety_negative_front_m < minimum_side_distance_m ||
        assessment.safety_positive_front_m < minimum_side_distance_m) {
        return 0.0;
    }

    const double gain = std::max(0.0, config_.lateral_balance_gain_deg_per_m);
    const double correction_limit = std::max(0.0, max_correction_deg);
    if (gain <= 0.0 || correction_limit <= 0.0) {
        return 0.0;
    }

    const double balance_error_m =
        assessment.safety_positive_front_m - assessment.safety_negative_front_m;
    const double correction_deg = std::clamp(
        gain * balance_error_m,
        -correction_limit,
        correction_limit);
    if (std::abs(correction_deg) < 0.05) {
        return 0.0;
    }

    debug.lateral_balance_active = true;
    debug.lateral_balance_correction_deg = correction_deg;
    return correction_deg;
}

double AutoAvoidController::boundaryRiskFromDistanceM(double distance_m) const {
    if (!std::isfinite(distance_m)) {
        return 0.0;
    }

    const double hard_boundary_m = Judgment::kVehicleBoundaryThresholdMeters;
    const double soft_boundary_m = hard_boundary_m +
        std::max(0.05, config_.boundary_recovery_soft_margin_m);
    if (distance_m <= hard_boundary_m) {
        return 1.0;
    }
    if (distance_m >= soft_boundary_m) {
        return 0.0;
    }

    return std::clamp(
        (soft_boundary_m - distance_m) /
            std::max(1e-6, soft_boundary_m - hard_boundary_m),
        0.0,
        1.0);
}

AutoAvoidController::BoundaryRecoveryLevel
AutoAvoidController::boundaryRecoveryLevelForDistanceM(double distance_m) const {
    if (!std::isfinite(distance_m)) {
        return BoundaryRecoveryLevel::None;
    }

    const double hard_boundary_m = Judgment::kVehicleBoundaryThresholdMeters;
    const double critical_boundary_m = hard_boundary_m +
        std::max(0.0, config_.boundary_recovery_critical_margin_m);
    const double strong_boundary_m = hard_boundary_m +
        std::max(
            std::max(config_.boundary_recovery_critical_margin_m, 0.0),
            config_.boundary_recovery_strong_margin_m);
    const double soft_boundary_m = hard_boundary_m +
        std::max(
            std::max(config_.boundary_recovery_strong_margin_m, 0.0),
            config_.boundary_recovery_soft_margin_m);

    if (distance_m <= critical_boundary_m) {
        return BoundaryRecoveryLevel::Critical;
    }
    if (distance_m <= strong_boundary_m) {
        return BoundaryRecoveryLevel::Strong;
    }
    if (distance_m <= soft_boundary_m) {
        return BoundaryRecoveryLevel::Soft;
    }
    return BoundaryRecoveryLevel::None;
}

double AutoAvoidController::boundaryRecoveryStageWeight(AvoidanceStage stage) const {
    switch (stage) {
        case AvoidanceStage::Turning:
            return std::max(0.0, config_.boundary_recovery_turning_weight);
        case AvoidanceStage::ClearanceHold:
            return std::max(0.0, config_.boundary_recovery_clearance_hold_weight);
        case AvoidanceStage::ReturnHeading:
            return std::max(0.0, config_.boundary_recovery_return_to_path_weight);
        case AvoidanceStage::Idle:
        default:
            return std::max(0.0, config_.boundary_recovery_straight_weight);
    }
}

bool AutoAvoidController::directionTowardBoundary(
    TurnDirection direction,
    BoundaryRiskSide boundary_side) const {
    switch (boundary_side) {
        case BoundaryRiskSide::Left:
            return direction == TurnDirection::Left;
        case BoundaryRiskSide::Right:
            return direction == TurnDirection::Right;
        case BoundaryRiskSide::None:
        default:
            return false;
    }
}

bool AutoAvoidController::steeringTowardBoundary(
    double steering_angle_deg,
    BoundaryRiskSide boundary_side) const {
    switch (boundary_side) {
        case BoundaryRiskSide::Left:
            return steering_angle_deg < -0.05;
        case BoundaryRiskSide::Right:
            return steering_angle_deg > 0.05;
        case BoundaryRiskSide::None:
        default:
            return false;
    }
}

AutoAvoidController::BoundaryRecoveryDecision
AutoAvoidController::boundaryRecoveryDecision(
    const SnapshotAssessment& assessment,
    AvoidanceStage stage,
    TurnDirection direction,
    double main_steering_angle_deg,
    double path_recovery_correction_deg,
    bool tail_clearance_blocking) const {
    BoundaryRecoveryDecision decision;
    decision.adjusted_main_steering_deg = main_steering_angle_deg;

    decision.left_risk = boundaryRiskFromDistanceM(assessment.safety_negative_front_m);
    decision.right_risk = boundaryRiskFromDistanceM(assessment.safety_positive_front_m);
    decision.risk_delta = decision.right_risk - decision.left_risk;

    const double dominant_risk = std::max(decision.left_risk, decision.right_risk);
    if (dominant_risk <= 0.0) {
        return decision;
    }

    const double left_distance_m = assessment.safety_negative_front_m;
    const double right_distance_m = assessment.safety_positive_front_m;
    const double distance_delta_m = right_distance_m - left_distance_m;
    if (decision.risk_delta > 0.06 || distance_delta_m < -0.04) {
        decision.side = BoundaryRiskSide::Right;
    } else if (decision.risk_delta < -0.06 || distance_delta_m > 0.04) {
        decision.side = BoundaryRiskSide::Left;
    }

    if (!isBoundaryRiskSide(decision.side)) {
        return decision;
    }

    const double danger_distance_m =
        decision.side == BoundaryRiskSide::Left ? left_distance_m : right_distance_m;
    decision.level = boundaryRecoveryLevelForDistanceM(danger_distance_m);

    const double stage_weight = boundaryRecoveryStageWeight(stage);
    if (stage_weight <= 0.0) {
        return decision;
    }

    const bool steering_into_danger =
        steeringTowardBoundary(main_steering_angle_deg, decision.side);
    const bool direction_into_danger =
        directionTowardBoundary(direction, decision.side);
    const double asymmetry = dominant_risk > 1e-6 ?
        std::clamp(std::abs(decision.risk_delta) / dominant_risk, 0.0, 1.0) :
        0.0;
    const double danger_bias =
        steering_into_danger || direction_into_danger ?
            std::max(0.35, asymmetry) :
            asymmetry;
    double correction_deg =
        steeringAngleForBoundarySide(
            decision.side,
            dominant_risk *
                std::max(0.0, config_.boundary_recovery_max_correction_deg) *
                stage_weight *
                danger_bias);

    const bool opposes_committed_direction =
        hasCommittedAvoidanceTurn() &&
        directionTowardBoundary(active_avoidance_state_.committed_direction, decision.side);
    if (opposes_committed_direction && tail_clearance_blocking) {
        const double tail_scale = std::clamp(
            std::max(std::max(0.0, config_.boundary_recovery_tail_limit_weight), dominant_risk),
            0.0,
            1.0);
        correction_deg *= tail_scale;
        decision.limited_by_tail = std::abs(correction_deg) > 0.05;
    }

    const double activation_risk = std::clamp(
        config_.boundary_override_activation_risk,
        0.0,
        0.95);
    if ((steering_into_danger || direction_into_danger) &&
        dominant_risk >= activation_risk) {
        const double reduction_ratio = std::clamp(
            (dominant_risk - activation_risk) /
                std::max(1e-6, 1.0 - activation_risk) *
                stage_weight *
                std::max(0.0, config_.boundary_override_max_reduction_ratio),
            0.0,
            std::max(0.0, config_.boundary_override_max_reduction_ratio));
        if (std::abs(main_steering_angle_deg) > 0.05 && reduction_ratio > 0.0) {
            decision.adjusted_main_steering_deg =
                main_steering_angle_deg * (1.0 - reduction_ratio);
            decision.reduced_by_deg =
                std::abs(main_steering_angle_deg - decision.adjusted_main_steering_deg);
            decision.reduced_main_steering =
                std::abs(decision.reduced_by_deg) > 0.05;
        }

        decision.override_active =
            decision.reduced_main_steering || std::abs(correction_deg) > 0.12;
        if (decision.override_active) {
            decision.override_reason =
                std::string(steering_into_danger ? "steering_into_" : "direction_into_") +
                boundaryRiskSideName(decision.side) +
                "_boundary";
        }
    }

    decision.correction_deg = correction_deg;
    decision.active =
        decision.override_active || std::abs(decision.correction_deg) > 0.05;
    if (std::abs(path_recovery_correction_deg) > 0.05 &&
        std::abs(decision.correction_deg) > 0.05) {
        decision.aligned_with_path =
            (path_recovery_correction_deg > 0.0 && decision.correction_deg > 0.0) ||
            (path_recovery_correction_deg < 0.0 && decision.correction_deg < 0.0);
        decision.conflict_with_path =
            !decision.aligned_with_path;
    }
    return decision;
}

void AutoAvoidController::applyBoundaryRecoveryDebug(
    const BoundaryRecoveryDecision& decision,
    DebugInfo& debug) const {
    debug.boundary_risk_left = decision.left_risk;
    debug.boundary_risk_right = decision.right_risk;
    debug.boundary_risk_delta = decision.risk_delta;
    debug.boundary_recovery_active = decision.active;
    debug.boundary_recovery_side = decision.side;
    debug.boundary_recovery_level = decision.level;
    debug.boundary_recovery_correction_deg = decision.correction_deg;
    debug.boundary_recovery_limited_by_tail = decision.limited_by_tail;
    debug.boundary_override_active = decision.override_active;
    debug.boundary_override_reason = decision.override_reason;
    debug.boundary_override_reduced_main_steering = decision.reduced_main_steering;
    debug.boundary_override_reduced_by_deg = decision.reduced_by_deg;
    debug.boundary_recovery_and_path_aligned = decision.aligned_with_path;
    debug.boundary_recovery_and_path_conflict = decision.conflict_with_path;
}

double AutoAvoidController::avoidanceSteeringSlewDegPerTick(
    double front_nearest_m) const {
    const double ratio = normalizedAvoidanceDistanceRatio(front_nearest_m);
    return std::max(
        0.1,
        config_.avoidance_steering_slew_far_deg_per_tick +
            ratio * (config_.avoidance_steering_slew_near_deg_per_tick -
                config_.avoidance_steering_slew_far_deg_per_tick));
}

void AutoAvoidController::resetSteeringSmoothing() {
    steering_smoothing_state_ = SteeringSmoothingState{};
}

double AutoAvoidController::smoothSteeringAngleDeg(
    double desired_steering_angle_deg,
    double max_step_deg) {
    const double desired = clampSteeringAngle(desired_steering_angle_deg);
    const double step = std::max(0.1, max_step_deg);
    if (!steering_smoothing_state_.has_last_smoothed_angle) {
        steering_smoothing_state_.has_last_smoothed_angle = true;
        steering_smoothing_state_.last_smoothed_angle_deg = desired;
        return desired;
    }

    const double last = steering_smoothing_state_.last_smoothed_angle_deg;
    double output = desired;
    if ((last > 0.0 && desired < 0.0) || (last < 0.0 && desired > 0.0)) {
        const double unwind_step = std::max(step, 1.5);
        const double unwind = std::min(std::abs(last), unwind_step);
        output = last > 0.0 ? last - unwind : last + unwind;
    } else {
        const double delta = desired - last;
        output = last + std::clamp(delta, -step, step);
    }

    output = clampSteeringAngle(output);
    steering_smoothing_state_.has_last_smoothed_angle = true;
    steering_smoothing_state_.last_smoothed_angle_deg = output;
    return output;
}

int AutoAvoidController::compensatedImuHeadingEncoder(double steering_angle_deg) const {
    const int raw_encoder = Judgment::steeringAngleDegToEncoder(steering_angle_deg);
    if (raw_encoder == 0) {
        return 0;
    }

    const int sign = raw_encoder < 0 ? -1 : 1;
    int magnitude = std::abs(raw_encoder);
    const int deadband_start = std::max(0, config_.imu_heading_deadband_start_encoder);
    if (magnitude < deadband_start) {
        return 0;
    }

    magnitude = static_cast<int>(std::lround(
        static_cast<double>(magnitude) * std::max(0.0, config_.imu_heading_encoder_gain)));
    const int min_effective_encoder = std::clamp(
        std::max(deadband_start, config_.imu_heading_min_effective_encoder),
        0,
        Judgment::kMaxSteeringEncoder);
    if (magnitude > 0 && magnitude < min_effective_encoder) {
        magnitude = min_effective_encoder;
    }

    return Judgment::clampSteeringEncoder(sign * magnitude);
}

double AutoAvoidController::applyBoundarySteeringGuardDeg(
    double steering_angle_deg,
    const SensorSnapshot& snapshot,
    DebugInfo* debug) const {
    const double hard_boundary_m = Judgment::kVehicleBoundaryThresholdMeters;
    const double soft_boundary_m = hard_boundary_m +
        std::max(0.05, config_.boundary_steering_soft_margin_m);
    const auto steering_scale =
        [hard_boundary_m, soft_boundary_m](const SectorSample& sector) {
            if (!sector.valid || !std::isfinite(sector.nearest_m)) {
                return 1.0;
            }
            if (sector.nearest_m <= hard_boundary_m) {
                return 0.0;
            }
            if (sector.nearest_m >= soft_boundary_m) {
                return 1.0;
            }
            return std::clamp(
                (sector.nearest_m - hard_boundary_m) /
                    std::max(1e-6, soft_boundary_m - hard_boundary_m),
                0.0,
                1.0);
        };

    double guarded = clampSteeringAngle(steering_angle_deg);
    const double original = guarded;
    std::string constrained_side;
    if (guarded < 0.0) {
        constrained_side = "left";
        guarded *= steering_scale(snapshot.negative_front);
    } else if (guarded > 0.0) {
        constrained_side = "right";
        guarded *= steering_scale(snapshot.positive_front);
    }
    if (debug &&
        !constrained_side.empty() &&
        std::abs(guarded - original) > 0.01) {
        debug->wall_constraint_active = true;
        debug->wall_constraint_side = constrained_side;
        debug->wall_constraint_correction_deg = guarded - original;
    }
    return guarded;
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

std::string AutoAvoidController::formatDebugText(const DebugInfo& debug) const {
    auto format_number = [](double value, int precision) {
        if (!std::isfinite(value)) {
            return std::string("n/a");
        }
        std::ostringstream out;
        out << std::fixed << std::setprecision(precision) << value;
        return out.str();
    };

    std::ostringstream out;
    out << "reason=" << decisionReasonName(debug.reason_code)
        << ", state=" << avoidanceStageName(debug.state)
        << ", dir=" << turnDirectionName(debug.direction)
        << ", front=";
    if (debug.front_nearest_valid) {
        out << format_number(debug.front_nearest_m, 2)
            << "m@" << format_number(debug.front_angle_deg, 1)
            << "deg"
            << ", support=" << debug.front_support_points;
    } else {
        out << "n/a";
    }
    out << ", zone=" << Judgment::frontObstacleZoneName(debug.raw_zone)
        << "->" << Judgment::frontObstacleZoneName(debug.resolved_zone);
    if (debug.front_target_selection.valid) {
        out << ", cluster#=" << debug.front_target_selection.selected_front_cluster_id
            << "@score=" << format_number(
                debug.front_target_selection.selected_front_cluster_score, 2)
            << ", cluster_range=" << format_number(
                debug.front_target_selection.selected_front_cluster_median_range, 2)
            << "/" << format_number(
                debug.front_target_selection.selected_front_cluster_nearest_range, 2)
            << ", cluster_span=" << format_number(
                debug.front_target_selection.selected_front_cluster_span_deg, 1)
            << ", cluster_points=" <<
                debug.front_target_selection.selected_front_cluster_points;
        if (debug.front_target_selection.selected_front_cluster_wall_like) {
            out << ", wall_like=1";
        }
        if (debug.front_target_selection.wall_like_cluster_suppressed) {
            out << ", wall_like_suppressed=1";
        }
        if (!debug.front_target_selection.front_target_selection_reason.empty()) {
            out << ", target_reason="
                << debug.front_target_selection.front_target_selection_reason;
        }
    }
    if (!debug.front_target_selection.raw_zone_source.empty()) {
        out << ", raw_zone_source=" << debug.front_target_selection.raw_zone_source;
    }
    out << ", target_yaw=";
    if (debug.target_yaw_valid) {
        out << format_number(debug.target_yaw_deg, 1)
            << "@"
            << debug.target_yaw_locked_ms
            << "ms/"
            << avoidanceStageName(debug.target_yaw_locked_by_stage);
        if (debug.target_yaw_locked_this_cycle) {
            out << "/locked";
        }
    } else {
        out << "n/a";
    }
    if (debug.safety_fallback) {
        out << ", fallback=" << fallbackReasonName(debug.fallback_reason);
    }
    if (debug.spike_suppressed) {
        out << ", spike_filtered=1";
    }
    if (debug.zone_stabilized) {
        out << ", zone_hold=1";
    }
    if (debug.zone_ambiguous) {
        out << ", zone_ambiguous=1";
    }
    if (debug.resolved_zone_override_active) {
        out << ", zone_override=" << debug.resolved_zone_override_reason;
    }
    if (debug.committed_direction_override_active) {
        out << ", dir_override=" << debug.committed_direction_override_reason;
    }
    if (debug.sector_buffer_active_continue) {
        out << ", sector_buffer_continue=1";
    }
    if (debug.boundary_stop) {
        out << ", boundary_stop=1";
    }
    if (debug.emergency_stop) {
        out << ", emergency_stop=1";
    }
    if (debug.replan_triggered) {
        out << ", replan=1";
    }
    if (debug.return_heading_protected) {
        out << ", return_protected=" << debug.return_heading_protect_ticks_remaining;
    }
    if (debug.lateral_balance_active) {
        out << ", balance=" << format_number(debug.lateral_balance_correction_deg, 2);
    }
    if (debug.boundary_recovery_active) {
        out << ", boundary_recovery="
            << boundaryRiskSideName(debug.boundary_recovery_side)
            << "/" << boundaryRecoveryLevelName(debug.boundary_recovery_level)
            << ":" << format_number(debug.boundary_recovery_correction_deg, 2);
    }
    if (debug.boundary_recovery_limited_by_tail) {
        out << ", boundary_tail_limit=1";
    }
    if (debug.boundary_override_active) {
        out << ", boundary_override=" << debug.boundary_override_reason;
        if (debug.boundary_override_reduced_main_steering) {
            out << "/" << format_number(debug.boundary_override_reduced_by_deg, 2);
        }
    }
    if (std::abs(debug.boundary_risk_left) > 0.001 ||
        std::abs(debug.boundary_risk_right) > 0.001) {
        out << ", boundary_risk="
            << format_number(debug.boundary_risk_left, 2)
            << "/" << format_number(debug.boundary_risk_right, 2)
            << "/" << format_number(debug.boundary_risk_delta, 2);
    }
    if (debug.wall_constraint_active) {
        out << ", wall_constraint=" << debug.wall_constraint_side
            << ":" << format_number(debug.wall_constraint_correction_deg, 2);
    }
    if (debug.path_reference_valid) {
        out << ", ref_yaw=" << format_number(debug.reference_yaw_deg, 1)
            << ", ref_balance=" << format_number(debug.reference_side_balance, 2);
        out << ", ref_capture_ms=" << debug.path_reference_captured_ms
            << "/" << avoidanceStageName(debug.path_reference_captured_stage);
        if (debug.path_reference_captured_this_cycle) {
            out << ", ref_capture=1";
        }
    }
    if (debug.return_to_path_active) {
        out << ", return_to_path=1"
            << ", return_phase="
            << (debug.return_to_path_phase.empty() ? "n/a" : debug.return_to_path_phase)
            << ", return_progress="
            << format_number(debug.return_to_path_progress_score, 2)
            << ", yaw_recovery=" << format_number(debug.yaw_recovery_final_deg, 2)
            << "@g=" << format_number(debug.yaw_recovery_dynamic_gain, 2)
            << ", path_recovery=" << format_number(debug.path_recovery_final_deg, 2)
            << "@err=" << format_number(debug.path_recovery_balance_error, 2)
            << "/g=" << format_number(debug.path_recovery_dynamic_gain, 2)
            << ", return_combined=" << format_number(debug.combined_return_correction_deg, 2);
        if (debug.return_to_path_fast_recenter_active) {
            out << ", fast_recenter=1";
        }
        if (debug.return_to_path_settling_active) {
            out << ", settling=1";
        }
        if (debug.return_to_path_can_settle) {
            out << ", can_settle=1";
        }
        if (debug.yaw_recovery_retained_by_path) {
            out << ", yaw_retained_by_path=1";
        }
        if (debug.path_recovery_fast_recenter_boost > 0.001) {
            out << ", path_boost="
                << format_number(debug.path_recovery_fast_recenter_boost, 2);
        }
        if (debug.combined_return_correction_limited_by_tail) {
            out << ", return_tail_limit=1";
        }
        if (debug.return_to_path_near_reference) {
            out << ", near_reference=1";
        }
        if (!debug.return_to_path_blocked_reason.empty() &&
            debug.return_to_path_blocked_reason != "none") {
            out << ", return_blocked=" << debug.return_to_path_blocked_reason;
        }
        if (debug.boundary_recovery_and_path_aligned) {
            out << ", boundary_path=aligned";
        }
        if (debug.boundary_recovery_and_path_conflict) {
            out << ", boundary_path=conflict";
        }
    }
    if (debug.tail_clearance_complete) {
        out << ", tail_clear=1";
    }
    if (debug.tail_clearance_blocking) {
        out << ", tail_block=1";
    }
    if (debug.path_recovery_ready) {
        out << ", path_ready=1";
    }
    if (debug.path_recovery_settled) {
        out << ", path_settled=1";
    }
    if (debug.exit_to_idle_ready) {
        out << ", exit_idle_ready=1";
    }
    if (debug.used_imu_heading) {
        out << ", imu_hold=1";
    }
    if (debug.used_encoder_fallback) {
        out << ", encoder_fallback=" << encoderFallbackKindName(debug.encoder_fallback_kind);
    }
    if (debug.path_reference_clear_reason != PathReferenceClearReason::None) {
        out << ", path_ref_clear=" <<
            pathReferenceClearReasonName(debug.path_reference_clear_reason);
    }
    if (debug.target_yaw_clear_reason != TargetYawClearReason::None) {
        out << ", target_yaw_clear=" <<
            targetYawClearReasonName(debug.target_yaw_clear_reason);
    }
    return out.str();
}

AutoAvoidController::Command AutoAvoidController::driveCommand(
    TurnDirection direction,
    int speed_cm_s,
    double steering_angle_deg,
    const SensorSnapshot& snapshot,
    bool command_target_yaw_valid,
    double heading_correction_deg,
    DebugInfo debug) const {
    Command command;
    const double clamped_steering_angle_deg = clampSteeringAngle(steering_angle_deg);
    const bool imu_heading_used =
        snapshot.imu_valid &&
        command_target_yaw_valid &&
        std::abs(heading_correction_deg) > 0.001;
    int steering_encoder = Judgment::steeringAngleDegToEncoder(clamped_steering_angle_deg);
    double effective_steering_angle_deg = clamped_steering_angle_deg;
    if (imu_heading_used) {
        steering_encoder = compensatedImuHeadingEncoder(clamped_steering_angle_deg);
        effective_steering_angle_deg = Judgment::steeringEncoderToAngleDeg(steering_encoder);
    }

    command.valid = true;
    command.safety_fallback = false;
    command.mode = MotionMode::Drive;
    command.state = active_avoidance_state_.stage;
    command.direction = direction;
    command.speed_cm_s = clampSpeed(speed_cm_s);
    command.steering_angle_deg = effective_steering_angle_deg;
    command.steering_encoder = steering_encoder;
    debug.state = active_avoidance_state_.stage;
    debug.direction = direction;
    debug.safety_fallback = false;
    if (imu_heading_used) {
        debug.used_imu_heading = true;
    }
    command.reason_code = debug.reason_code;
    command.fallback_reason = debug.fallback_reason;
    command.debug = debug;
    command.debug_text = formatDebugText(debug);
    return command;
}

AutoAvoidController::Command AutoAvoidController::neutralSafeCommand(
    DebugInfo debug) const {
    return safetyFallbackCommand(std::move(debug));
}

AutoAvoidController::Command AutoAvoidController::safetyFallbackCommand(
    DebugInfo debug) const {
    debug.safety_fallback = true;
    auto command = stopCommand(std::move(debug));
    command.safety_fallback = true;
    return command;
}

AutoAvoidController::Command AutoAvoidController::stopCommand(DebugInfo debug) const {
    Command command;
    command.valid = true;
    command.safety_fallback = debug.safety_fallback;
    command.mode = MotionMode::Stop;
    command.state = active_avoidance_state_.stage;
    command.direction = TurnDirection::Stop;
    command.speed_cm_s = 0;
    command.steering_encoder = 0;
    command.steering_angle_deg = 0.0;
    debug.state = active_avoidance_state_.stage;
    debug.direction = TurnDirection::Stop;
    command.reason_code = debug.reason_code;
    command.fallback_reason = debug.fallback_reason;
    command.debug = debug;
    command.debug_text = formatDebugText(debug);
    return command;
}
