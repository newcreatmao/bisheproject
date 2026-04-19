#pragma once

#include "control/Judgment.hpp"

#include <cstdint>
#include <deque>
#include <string>

class AutoAvoidController {
public:
    struct SectorSample {
        bool valid = false;
        double nearest_m = 0.0;
        double nearest_angle_deg = 0.0;
        int valid_points = 0;
        int support_points = 0;
    };

    struct FrontTargetSelection {
        bool valid = false;
        int selected_front_cluster_id = -1;
        double selected_front_cluster_score = 0.0;
        bool selected_front_cluster_wall_like = false;
        int selected_front_cluster_points = 0;
        double selected_front_cluster_span_deg = 0.0;
        double selected_front_cluster_median_range = 0.0;
        double selected_front_cluster_nearest_range = 0.0;
        bool wall_like_cluster_suppressed = false;
        std::string front_target_selection_reason;
        std::string raw_zone_source;
    };

    struct AutoAvoidSnapshot {
        std::int64_t timestamp_steady_ms = 0;
        bool snapshot_fresh = false;
        bool lidar_valid = false;
        SectorSample negative_front;  // Lidar -90 to -62 degrees.
        SectorSample front;           // Lidar -58 to 58 degrees.
        SectorSample positive_front;  // Lidar 62 to 90 degrees.
        SectorSample sector_buffer;   // Lidar boundary deadbands: -62 to -58 and 58 to 62 degrees.
        bool front_nearest_valid = false;
        double front_nearest_m = 0.0;
        double front_angle_deg = 0.0;
        int front_support_points = 0;
        FrontTargetSelection front_target_selection;
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

    enum class AvoidanceStage {
        Idle,
        Turning,
        ClearanceHold,
        ReturnHeading
    };

    enum class DecisionReason {
        Unknown,
        InvalidDecision,
        LidarInvalid,
        BoundaryInvalid,
        BoundaryStop,
        FrontInvalid,
        SectorBuffer,
        SectorBufferContinue,
        EmergencyStop,
        FrontZoneBuffer,
        FrontAvoidanceImu,
        FrontAvoidanceEncoderFallback,
        ClearanceHoldImu,
        ClearanceHoldEncoderFallback,
        ReturnHeading,
        StraightDrive
    };

    enum class FallbackReason {
        None,
        InvalidDecision,
        LidarInvalid,
        BoundaryInvalid,
        FrontInvalid,
        SectorBuffer,
        FrontZoneBuffer
    };

    enum class EncoderFallbackKind {
        None,
        FrontObstacleLinear,
        ClearanceHoldFixed
    };

    enum class PathReferenceClearReason {
        None,
        ControllerReset,
        InvalidInput,
        SafetyStop,
        AvoidanceCompleted
    };

    enum class TargetYawClearReason {
        None,
        ControllerReset
    };

    struct AutoAvoidConfig {
        int cruise_speed_cm_s = 50;
        int caution_speed_cm_s = 40;
        int avoidance_speed_cm_s = 30;
        double fast_speed_min_distance_m = 1.80;
        double medium_speed_max_distance_m = 1.70;
        double medium_speed_min_distance_m = 1.30;
        double avoidance_turn_max_distance_m = 1.20;
        double emergency_stop_distance_m = 0.40;
        double emergency_stop_release_distance_m = 0.55;
        double avoidance_release_distance_m = 1.30;
        double active_stage_front_replan_distance_m = 1.50;
        double active_stage_front_replan_center_half_width_deg = 32.0;
        double linear_avoidance_front_half_width_deg = 58.0;
        int linear_avoidance_encoder_per_10deg = 40;
        int lidar_filter_window = 3;
        double front_spike_center_half_width_deg = 12.0;
        double front_spike_jump_distance_m = 0.45;
        double front_spike_side_clear_distance_m = 0.90;
        int front_spike_confirm_ticks = 2;
        int front_spike_max_support_points = 4;
        double caution_turn_angle_deg = 10.0;
        double center_turn_angle_deg = 18.0;
        int sector_buffer_continue_speed_cm_s = 24;
        double sector_buffer_continue_turn_angle_deg = 8.0;
        double heading_kp = 0.55;
        double straight_heading_tolerance_deg = 0.35;
        double return_heading_kp = 0.35;
        int return_heading_min_hold_ticks = 6;
        double avoidance_heading_kp_far = 0.65;
        double avoidance_heading_kp_near = 1.35;
        double avoidance_target_min_delta_deg = 13.0;
        double avoidance_target_max_delta_deg = 34.0;
        double avoidance_target_distance_weight = 0.70;
        double avoidance_heading_target_tolerance_deg = 1.0;
        double avoidance_heading_max_correction_far_deg = 9.0;
        double avoidance_heading_max_correction_near_deg = 14.0;
        double clearance_hold_heading_kp = 0.60;
        double clearance_hold_max_correction_deg = 8.0;
        double return_heading_tolerance_deg = 2.5;
        double straight_heading_max_correction_deg = 6.0;
        double caution_heading_max_correction_deg = 6.0;
        double lateral_balance_min_side_distance_m = 0.75;
        double lateral_balance_gain_deg_per_m = 5.0;
        double lateral_balance_max_correction_deg = 1.8;
        double return_to_path_balance_gain_deg_per_m = 6.0;
        double return_to_path_balance_max_correction_deg = 2.8;
        double return_to_path_balance_tolerance_m = 0.18;
        int return_to_path_settle_confirm_ticks = 4;
        double max_usable_yaw_error_deg = 45.0;
        int imu_heading_deadband_start_encoder = 6;
        int imu_heading_min_effective_encoder = 20;
        double imu_heading_encoder_gain = 1.20;
        int obstacle_zone_switch_confirm_ticks = 4;
        int obstacle_zone_override_confirm_ticks = 2;
        int committed_direction_override_confirm_ticks = 2;
        int clearance_hold_speed_cm_s = 30;
        int return_heading_speed_cm_s = 40;
        double straight_steering_slew_deg_per_tick = 2.0;
        double avoidance_steering_slew_far_deg_per_tick = 2.0;
        double avoidance_steering_slew_near_deg_per_tick = 4.0;
        double clearance_hold_steering_slew_deg_per_tick = 1.3;
        double return_heading_steering_slew_deg_per_tick = 1.0;
        double boundary_steering_soft_margin_m = 0.32;
        double vehicle_length_m = 1.30;
        double vehicle_width_m = 0.90;
        double lidar_x_from_base_link_m = 0.62;
        double tail_clearance_margin_m = 0.10;
    };

    struct AutoAvoidDebugInfo {
        bool snapshot_fresh = false;
        bool lidar_valid = false;
        bool imu_valid = false;
        bool front_nearest_valid = false;
        double front_nearest_m = 0.0;
        double front_angle_deg = 0.0;
        int front_support_points = 0;
        AvoidanceStage state = AvoidanceStage::Idle;
        TurnDirection direction = TurnDirection::Stop;
        DecisionReason reason_code = DecisionReason::Unknown;
        FallbackReason fallback_reason = FallbackReason::None;
        Judgment::FrontObstacleZone raw_zone = Judgment::FrontObstacleZone::Unknown;
        Judgment::FrontObstacleZone resolved_zone =
            Judgment::FrontObstacleZone::Unknown;
        bool safety_fallback = false;
        bool spike_suppressed = false;
        bool zone_stabilized = false;
        bool zone_ambiguous = false;
        FrontTargetSelection front_target_selection;
        bool sector_buffer_active_continue = false;
        bool boundary_stop = false;
        bool emergency_stop = false;
        bool replan_triggered = false;
        bool resolved_zone_override_active = false;
        std::string resolved_zone_override_reason;
        bool committed_direction_override_active = false;
        std::string committed_direction_override_reason;
        bool return_heading_protected = false;
        int return_heading_protect_ticks_remaining = 0;
        bool lateral_balance_active = false;
        double lateral_balance_correction_deg = 0.0;
        bool wall_constraint_active = false;
        std::string wall_constraint_side;
        double wall_constraint_correction_deg = 0.0;
        bool path_reference_valid = false;
        double reference_yaw_deg = 0.0;
        double reference_side_balance = 0.0;
        double reference_left_distance_m = 0.0;
        double reference_right_distance_m = 0.0;
        std::int64_t path_reference_captured_ms = 0;
        AvoidanceStage path_reference_captured_stage = AvoidanceStage::Idle;
        bool path_reference_captured_this_cycle = false;
        bool return_to_path_active = false;
        double yaw_recovery_correction_deg = 0.0;
        double path_recovery_correction_deg = 0.0;
        double combined_return_correction_deg = 0.0;
        bool tail_clearance_complete = false;
        bool tail_clearance_blocking = false;
        bool path_recovery_ready = false;
        bool path_recovery_settled = false;
        bool exit_to_idle_ready = false;
        bool used_imu_heading = false;
        bool used_encoder_fallback = false;
        EncoderFallbackKind encoder_fallback_kind = EncoderFallbackKind::None;
        PathReferenceClearReason path_reference_clear_reason =
            PathReferenceClearReason::None;
        bool target_yaw_valid = false;
        double target_yaw_deg = 0.0;
        std::int64_t target_yaw_locked_ms = 0;
        AvoidanceStage target_yaw_locked_by_stage = AvoidanceStage::Idle;
        bool target_yaw_locked_this_cycle = false;
        TargetYawClearReason target_yaw_clear_reason = TargetYawClearReason::None;
    };

    struct AutoAvoidDecision {
        bool valid = true;
        bool safety_fallback = false;
        MotionMode mode = MotionMode::Stop;
        AvoidanceStage state = AvoidanceStage::Idle;
        TurnDirection direction = TurnDirection::Stop;
        int speed_cm_s = 0;
        int steering_encoder = 0;
        double steering_angle_deg = 0.0;
        DecisionReason reason_code = DecisionReason::Unknown;
        FallbackReason fallback_reason = FallbackReason::None;
        AutoAvoidDebugInfo debug;
        std::string debug_text;
    };

    using SensorSnapshot = AutoAvoidSnapshot;
    using Config = AutoAvoidConfig;
    using Command = AutoAvoidDecision;
    using DebugInfo = AutoAvoidDebugInfo;

    AutoAvoidController();
    explicit AutoAvoidController(const Config& config);

    Command decide(const SensorSnapshot& snapshot);
    void reset();

    const Config& config() const;
    AvoidanceStage currentAvoidanceStage() const;

    static const char* motionModeName(MotionMode mode);
    static const char* turnDirectionName(TurnDirection direction);
    static const char* avoidanceStageName(AvoidanceStage stage);
    static const char* decisionReasonName(DecisionReason reason);
    static const char* fallbackReasonName(FallbackReason reason);
    static const char* encoderFallbackKindName(EncoderFallbackKind kind);
    static const char* pathReferenceClearReasonName(PathReferenceClearReason reason);
    static const char* targetYawClearReasonName(TargetYawClearReason reason);

private:
    struct SnapshotAssessment {
        SensorSnapshot snapshot;
        Judgment::VehicleBoundaryResult boundary;
        Judgment::FrontObstacleResult front_obstacle;
        bool front_distance_valid = false;
        double safety_negative_front_m = 0.0;
        double safety_front_m = 0.0;
        double safety_positive_front_m = 0.0;
    };

    struct FrontSpikeFilterState {
        bool has_last_stable_sample = false;
        SectorSample last_stable_sample;
        bool has_pending_sample = false;
        SectorSample pending_sample;
        int pending_ticks = 0;
        bool suppressed_this_cycle = false;
    };

    struct ObstacleZoneStabilizerState {
        Judgment::FrontObstacleZone stable_zone =
            Judgment::FrontObstacleZone::Unknown;
        Judgment::FrontObstacleZone pending_zone =
            Judgment::FrontObstacleZone::Unknown;
        int pending_ticks = 0;
    };

    struct MotionHysteresisState {
        bool emergency_stop_latched = false;
        bool avoidance_turn_latched = false;
    };

    struct ActiveAvoidanceState {
        AvoidanceStage stage = AvoidanceStage::Idle;
        TurnDirection committed_direction = TurnDirection::Straight;
        Judgment::FrontObstacleZone committed_obstacle_side_zone =
            Judgment::FrontObstacleZone::Unknown;
        bool committed_target_yaw_valid = false;
        double committed_target_yaw_deg = 0.0;
        bool tail_side_obstacle_seen = false;
        bool tail_side_clear_phase_started = false;
        double clearance_hold_progress_m = 0.0;
        double tail_post_clear_progress_m = 0.0;
        int return_heading_ticks = 0;
        int return_to_path_settle_ticks = 0;
        TurnDirection pending_override_direction = TurnDirection::Straight;
        int pending_override_ticks = 0;
    };

    struct SteeringSmoothingState {
        bool has_last_smoothed_angle = false;
        double last_smoothed_angle_deg = 0.0;
    };

    struct TargetYawState {
        bool valid = false;
        double yaw_deg = 0.0;
        std::int64_t locked_timestamp_steady_ms = 0;
        AvoidanceStage locked_by_stage = AvoidanceStage::Idle;
        bool locked_this_cycle = false;
        TargetYawClearReason last_clear_reason = TargetYawClearReason::None;
    };

    struct PathReferenceState {
        bool valid = false;
        double reference_yaw_deg = 0.0;
        double reference_side_balance = 0.0;
        double reference_left_distance_m = 0.0;
        double reference_right_distance_m = 0.0;
        std::int64_t captured_steady_ms = 0;
        bool captured_this_cycle = false;
        AvoidanceStage captured_stage = AvoidanceStage::Idle;
        PathReferenceClearReason clear_reason = PathReferenceClearReason::None;
    };

    struct FrontSpikeFilterResult {
        SectorSample sample;
        bool suppressed = false;
    };

    struct ObstacleZoneResolution {
        Judgment::FrontObstacleResult obstacle;
        bool ambiguous = false;
        bool stabilized = false;
        bool override_active = false;
        std::string override_reason;
    };

    SensorSnapshot normalizedSnapshot(const SensorSnapshot& snapshot) const;
    SensorSnapshot bindManagedTargetYaw(const SensorSnapshot& snapshot);
    SensorSnapshot filteredSnapshot(const SensorSnapshot& snapshot);
    SnapshotAssessment assessSnapshot(const SensorSnapshot& snapshot);
    DebugInfo baseDebugInfo(const SnapshotAssessment& assessment) const;
    void capturePathReferenceIfNeeded(const SnapshotAssessment& assessment);
    void clearPathReference(PathReferenceClearReason reason);
    Command decideTooCloseFront(
        const SnapshotAssessment& assessment,
        const Judgment::FrontObstacleResult& front_obstacle,
        DebugInfo debug);
    Command decideActiveAvoidanceStage(
        const SnapshotAssessment& assessment,
        const Judgment::FrontObstacleResult& front_obstacle,
        DebugInfo debug);
    Command decideSectorBufferDuringActiveStage(
        const SnapshotAssessment& assessment,
        DebugInfo debug);
    bool tryContinueClearanceHoldStage(
        const SnapshotAssessment& assessment,
        DebugInfo debug,
        Command& command);
    bool tryContinueReturnHeadingStage(
        const SnapshotAssessment& assessment,
        DebugInfo debug,
        Command& command);
    Command decideStraightDrive(
        const SnapshotAssessment& assessment,
        const Judgment::FrontObstacleResult& front_obstacle,
        DebugInfo debug);
    ObstacleZoneResolution resolveFrontObstacleZone(
        const SnapshotAssessment& assessment,
        const Judgment::FrontObstacleResult& front_obstacle);
    SectorSample filteredSectorSample(
        const SectorSample& sample,
        std::deque<SectorSample>& history) const;
    void resetFilteredLidarHistory();
    void resetFrontSpikeFilter();
    FrontSpikeFilterResult applyFrontSpikeFilter(
        const SectorSample& front,
        const SectorSample& negative_front,
        const SectorSample& positive_front);
    void resetMotionHysteresis();
    bool emergencyStopActive(double front_nearest_m);
    bool avoidanceTurnActive(double front_nearest_m);
    bool shouldReplanFrontDuringActiveAvoidance(
        const SensorSnapshot& snapshot,
        const Judgment::FrontObstacleResult& front_obstacle) const;
    void resetAvoidanceState();
    bool hasCommittedAvoidanceTurn() const;
    TurnDirection turnDirectionForObstacleZone(
        Judgment::FrontObstacleZone zone) const;
    Judgment::FrontObstacleZone obstacleSideZoneForTurnDirection(
        TurnDirection direction) const;
    const SectorSample* sectorForZone(
        const SensorSnapshot& snapshot,
        Judgment::FrontObstacleZone zone) const;
    void lockAvoidanceTurn(
        TurnDirection direction,
        const Judgment::FrontObstacleResult& front_obstacle,
        const SensorSnapshot& snapshot);
    void enterClearanceHold();
    double rearReachFromLidarM() const;
    double tailClearanceSideReleaseDistanceM() const;
    double tailClearanceSideCaptureDistanceM() const;
    double tailClearanceMinHoldDistanceM() const;
    double tailClearancePostSideClearTravelM() const;
    double estimatedTravelPerTickM(int speed_cm_s) const;
    bool updateClearanceHoldAndCheckDone(
        const SensorSnapshot& snapshot,
        int speed_cm_s);
    double normalizedAvoidanceDistanceRatio(double front_nearest_m) const;
    double avoidanceHeadingCorrectionDeg(
        double yaw_deg,
        double target_yaw_deg,
        double front_nearest_m) const;
    double clearanceHoldHeadingCorrectionDeg(
        double yaw_deg,
        double target_yaw_deg) const;
    double headingCorrectionDegForTarget(
        double yaw_deg,
        double target_yaw_deg,
        double kp,
        double max_correction_deg,
        double settle_tolerance_deg = 0.0) const;
    double headingCorrectionDeg(
        const SensorSnapshot& snapshot,
        double max_correction_deg) const;
    double lateralBalanceCorrectionDeg(
        const SnapshotAssessment& assessment,
        double max_correction_deg,
        DebugInfo& debug) const;
    double sideBalanceFromAssessment(const SnapshotAssessment& assessment) const;
    bool frontClearEnoughForPathRecovery(const SensorSnapshot& snapshot) const;
    bool tailClearanceComplete(const SensorSnapshot& snapshot) const;
    double pathRecoveryCorrectionDeg(
        const SnapshotAssessment& assessment,
        DebugInfo& debug) const;
    double avoidanceSteeringSlewDegPerTick(double front_nearest_m) const;
    void resetSteeringSmoothing();
    double smoothSteeringAngleDeg(
        double desired_steering_angle_deg,
        double max_step_deg);
    int compensatedImuHeadingEncoder(double steering_angle_deg) const;
    double applyBoundarySteeringGuardDeg(
        double steering_angle_deg,
        const SensorSnapshot& snapshot,
        DebugInfo* debug = nullptr) const;
    double linearAvoidanceTargetYawDeltaDeg(
        const Judgment::FrontObstacleResult& front_obstacle) const;
    int frontSpeedCmS(double front_nearest_m) const;
    double fallbackSteeringAngleDeg(
        EncoderFallbackKind kind,
        TurnDirection direction,
        const Judgment::FrontObstacleResult* front_obstacle,
        DebugInfo& debug) const;
    void rememberDriveSpeed(int speed_cm_s);
    void resetDriveSpeed();
    void resetObstacleZoneStabilizer();
    Judgment::FrontObstacleResult stabilizeFrontObstacleZone(
        const Judgment::FrontObstacleResult& front_obstacle,
        bool& stabilized,
        int confirm_ticks_override = 0);
    bool isSectorBufferNearest(const SensorSnapshot& snapshot) const;
    TurnDirection chooseCenterTurnDirection(const SensorSnapshot& snapshot) const;
    std::string formatDebugText(const DebugInfo& debug) const;
    Command driveCommand(
        TurnDirection direction,
        int speed_cm_s,
        double steering_angle_deg,
        const SensorSnapshot& snapshot,
        bool command_target_yaw_valid,
        double heading_correction_deg,
        DebugInfo debug) const;
    Command neutralSafeCommand(
        DebugInfo debug) const;
    Command safetyFallbackCommand(
        DebugInfo debug) const;
    Command stopCommand(DebugInfo debug) const;

    Config config_;
    Judgment judgment_;
    ObstacleZoneStabilizerState obstacle_zone_state_;
    std::deque<SectorSample> negative_front_history_;
    std::deque<SectorSample> front_history_;
    std::deque<SectorSample> positive_front_history_;
    std::deque<SectorSample> sector_buffer_history_;
    FrontSpikeFilterState front_spike_filter_state_;
    MotionHysteresisState motion_hysteresis_state_;
    bool has_last_drive_speed_ = false;
    int last_drive_speed_cm_s_ = 0;
    ActiveAvoidanceState active_avoidance_state_;
    SteeringSmoothingState steering_smoothing_state_;
    TargetYawState target_yaw_state_;
    PathReferenceState path_reference_state_;
};
