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
        bool selected_front_cluster_is_discrete_primary = false;
        bool selected_front_cluster_is_wall_like = false;
        int selected_front_cluster_points = 0;
        double selected_front_cluster_span_deg = 0.0;
        double selected_front_cluster_median_range = 0.0;
        double selected_front_cluster_nearest_range = 0.0;
        bool wall_like_cluster_suppressed = false;
        bool raw_zone_from_discrete_target = false;
        bool wall_like_suppressed_from_zone = false;
        std::string front_target_selection_reason;
        std::string front_target_role;
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

    enum class BoundaryRiskSide {
        None,
        Left,
        Right
    };

    enum class BoundaryRecoveryLevel {
        None,
        Soft,
        Strong,
        Critical
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
        double active_stage_wall_replan_distance_m = 1.05;
        double active_stage_wall_replan_center_half_width_deg = 18.0;
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
        double return_to_path_fast_recenter_balance_error_m = 0.30;
        double return_to_path_fast_recenter_yaw_error_deg = 4.5;
        double return_to_path_near_reference_balance_error_m = 0.14;
        double return_to_path_near_reference_yaw_error_deg = 1.8;
        double return_to_path_settle_balance_error_m = 0.18;
        double return_to_path_settle_yaw_error_deg = 2.6;
        double return_to_path_settling_hold_balance_error_m = 0.24;
        double return_to_path_settling_hold_yaw_error_deg = 3.2;
        double return_to_path_yaw_fast_gain_scale = 1.65;
        double return_to_path_yaw_settling_gain_scale = 0.90;
        double return_to_path_yaw_retained_gain_scale = 1.25;
        double straight_heading_max_correction_deg = 6.0;
        double caution_heading_max_correction_deg = 6.0;
        double lateral_balance_min_side_distance_m = 0.75;
        double lateral_balance_gain_deg_per_m = 5.0;
        double lateral_balance_max_correction_deg = 1.8;
        double boundary_recovery_soft_margin_m = 0.32;
        double boundary_recovery_strong_margin_m = 0.18;
        double boundary_recovery_critical_margin_m = 0.08;
        double boundary_recovery_max_correction_deg = 4.2;
        double boundary_recovery_straight_weight = 1.00;
        double boundary_recovery_turning_weight = 0.58;
        double boundary_recovery_clearance_hold_weight = 0.68;
        double boundary_recovery_return_to_path_weight = 0.90;
        double boundary_recovery_tail_limit_weight = 0.45;
        double boundary_override_activation_risk = 0.26;
        double boundary_override_max_reduction_ratio = 0.85;
        double return_to_path_balance_gain_deg_per_m = 6.0;
        double return_to_path_balance_max_correction_deg = 2.8;
        double return_to_path_balance_fast_gain_scale = 1.35;
        double return_to_path_balance_settling_gain_scale = 0.85;
        double return_to_path_balance_fast_boost_ratio = 0.35;
        double return_to_path_balance_fast_max_correction_deg = 4.2;
        double return_to_path_balance_tolerance_m = 0.18;
        double return_to_path_exit_correction_deg = 1.10;
        double return_to_path_tail_blocking_limit_scale = 0.45;
        double return_to_path_tail_blocking_min_correction_deg = 1.40;
        double return_to_path_tail_release_progress_score = 0.72;
        double return_to_path_tail_release_limit_scale = 0.78;
        int return_to_path_settle_entry_ticks = 2;
        int return_to_path_settle_confirm_ticks = 4;
        double max_usable_yaw_error_deg = 45.0;
        int imu_heading_deadband_start_encoder = 6;
        int imu_heading_min_effective_encoder = 20;
        double imu_heading_encoder_gain = 1.20;
        int obstacle_zone_switch_confirm_ticks = 4;
        int obstacle_zone_override_confirm_ticks = 2;
        int committed_direction_override_confirm_ticks = 2;
        double center_turn_min_side_distance_delta_m = 0.10;
        double center_turn_reference_balance_bias_m = 0.08;
        double center_turn_boundary_risk_delta = 0.08;
        int clearance_hold_speed_cm_s = 30;
        int return_heading_speed_cm_s = 40;
        double return_to_path_front_clear_distance_m = 1.18;
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
        std::string active_stage_priority_mode;
        bool replan_override_active = false;
        std::string replan_override_reason;
        bool active_stage_protection_active = false;
        std::string active_stage_protection_reason;
        bool resolved_zone_override_active = false;
        std::string resolved_zone_override_reason;
        bool committed_direction_override_active = false;
        std::string committed_direction_override_reason;
        std::string center_turn_decision_mode;
        bool center_turn_bias_removed = false;
        std::string center_turn_decision_reason;
        bool return_heading_protected = false;
        int return_heading_protect_ticks_remaining = 0;
        bool lateral_balance_active = false;
        double lateral_balance_correction_deg = 0.0;
        double main_steering_deg = 0.0;
        std::string main_steering_source;
        bool wall_constraint_active = false;
        std::string wall_constraint_side;
        double wall_constraint_correction_deg = 0.0;
        bool boundary_recovery_active = false;
        BoundaryRiskSide boundary_recovery_side = BoundaryRiskSide::None;
        BoundaryRecoveryLevel boundary_recovery_level = BoundaryRecoveryLevel::None;
        double boundary_recovery_correction_deg = 0.0;
        bool boundary_recovery_limited_by_tail = false;
        bool boundary_recovery_applied = false;
        double boundary_recovery_delta_deg = 0.0;
        bool boundary_override_active = false;
        std::string boundary_override_reason;
        bool boundary_override_reduced_main_steering = false;
        double boundary_override_reduced_by_deg = 0.0;
        bool boundary_override_applied = false;
        double boundary_override_delta_deg = 0.0;
        double boundary_risk_left = 0.0;
        double boundary_risk_right = 0.0;
        double boundary_risk_delta = 0.0;
        bool boundary_recovery_and_path_aligned = false;
        bool boundary_recovery_and_path_conflict = false;
        bool path_reference_valid = false;
        double reference_yaw_deg = 0.0;
        double reference_side_balance = 0.0;
        double reference_left_distance_m = 0.0;
        double reference_right_distance_m = 0.0;
        std::int64_t path_reference_captured_ms = 0;
        AvoidanceStage path_reference_captured_stage = AvoidanceStage::Idle;
        bool path_reference_captured_this_cycle = false;
        bool return_to_path_active = false;
        std::string return_to_path_phase;
        bool return_to_path_fast_recenter_active = false;
        bool return_to_path_settling_active = false;
        bool return_to_path_can_settle = false;
        std::string return_to_path_blocked_reason;
        double yaw_recovery_correction_deg = 0.0;
        double yaw_recovery_dynamic_gain = 0.0;
        bool yaw_recovery_retained_by_path = false;
        double yaw_recovery_final_deg = 0.0;
        double path_recovery_correction_deg = 0.0;
        double path_recovery_balance_error = 0.0;
        double path_recovery_dynamic_gain = 0.0;
        double path_recovery_fast_recenter_boost = 0.0;
        double path_recovery_final_deg = 0.0;
        double combined_return_correction_deg = 0.0;
        bool combined_return_correction_limited_by_tail = false;
        double return_to_path_progress_score = 0.0;
        bool return_to_path_near_reference = false;
        bool tail_clearance_complete = false;
        bool tail_clearance_blocking = false;
        bool path_recovery_ready = false;
        bool path_recovery_settled = false;
        bool exit_to_idle_ready = false;
        double smoothed_steering_deg = 0.0;
        double guarded_steering_deg = 0.0;
        int final_encoder_command = 0;
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
    static const char* boundaryRiskSideName(BoundaryRiskSide side);
    static const char* boundaryRecoveryLevelName(BoundaryRecoveryLevel level);

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
        int return_to_path_exit_ticks = 0;
        bool return_to_path_settling = false;
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

    struct BoundaryRecoveryDecision {
        BoundaryRiskSide side = BoundaryRiskSide::None;
        BoundaryRecoveryLevel level = BoundaryRecoveryLevel::None;
        double left_risk = 0.0;
        double right_risk = 0.0;
        double risk_delta = 0.0;
        bool active = false;
        bool limited_by_tail = false;
        bool override_active = false;
        std::string override_reason;
        bool reduced_main_steering = false;
        double reduced_by_deg = 0.0;
        double adjusted_main_steering_deg = 0.0;
        double correction_deg = 0.0;
        bool aligned_with_path = false;
        bool conflict_with_path = false;
    };

    struct ActiveStagePriorityDecision {
        bool continue_active_stage = false;
        bool protection_active = false;
        std::string protection_reason;
        bool replan_override_active = false;
        std::string replan_override_reason;
        std::string priority_mode = "standard_front_priority";
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
    ActiveStagePriorityDecision evaluateActiveStagePriority(
        const SnapshotAssessment& assessment,
        const Judgment::FrontObstacleResult& front_obstacle) const;
    void applyActiveStagePriorityDebug(
        const ActiveStagePriorityDecision& decision,
        DebugInfo& debug) const;
    void resetAvoidanceState();
    bool hasCommittedAvoidanceTurn() const;
    bool frontTargetIsDiscretePrimary(const SensorSnapshot& snapshot) const;
    bool frontTargetIsWallConstraint(const SensorSnapshot& snapshot) const;
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
    double boundaryRiskFromDistanceM(double distance_m) const;
    BoundaryRecoveryLevel boundaryRecoveryLevelForDistanceM(double distance_m) const;
    double boundaryRecoveryStageWeight(AvoidanceStage stage) const;
    bool directionTowardBoundary(
        TurnDirection direction,
        BoundaryRiskSide boundary_side) const;
    bool steeringTowardBoundary(
        double steering_angle_deg,
        BoundaryRiskSide boundary_side) const;
    BoundaryRecoveryDecision boundaryRecoveryDecision(
        const SnapshotAssessment& assessment,
        AvoidanceStage stage,
        TurnDirection direction,
        double main_steering_angle_deg,
        double path_recovery_correction_deg,
        bool tail_clearance_blocking) const;
    void applyBoundaryRecoveryDebug(
        const BoundaryRecoveryDecision& decision,
        DebugInfo& debug) const;
    void applySteeringArbitrationDebug(
        double main_steering_deg,
        const std::string& main_steering_source,
        const BoundaryRecoveryDecision& boundary_recovery,
        double smoothed_steering_deg,
        double guarded_steering_deg,
        DebugInfo& debug) const;
    double sideBalanceFromAssessment(const SnapshotAssessment& assessment) const;
    bool frontClearEnoughForPathRecovery(const SensorSnapshot& snapshot) const;
    bool tailClearanceComplete(const SensorSnapshot& snapshot) const;
    double pathRecoveryCorrectionDeg(
        const SnapshotAssessment& assessment,
        double gain_deg_per_m,
        double max_correction_deg,
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
    TurnDirection chooseCenterTurnDirection(
        const SnapshotAssessment& assessment,
        DebugInfo* debug = nullptr);
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
    TurnDirection center_turn_tie_break_direction_ = TurnDirection::Right;
};
