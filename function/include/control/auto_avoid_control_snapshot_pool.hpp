#pragma once

#include "control/auto_avoid.hpp"
#include "control/auto_avoid_input_builder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>

class AutoAvoidControlSnapshotPool {
public:
    struct ControlSnapshot {
        AutoAvoidController::SensorSnapshot snapshot;
        std::uint64_t control_snapshot_seq = 0;
        std::int64_t control_snapshot_stamp_ms = 0;
        std::uint64_t lidar_snapshot_seq = 0;
        std::uint64_t imu_snapshot_seq = 0;
        double lidar_snapshot_age_ms = std::numeric_limits<double>::quiet_NaN();
        double imu_snapshot_age_ms = std::numeric_limits<double>::quiet_NaN();
        bool control_snapshot_consistent = false;
        bool control_snapshot_fresh = false;
        std::string control_snapshot_source;
    };

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        lidar_state_ = LidarChannelState{};
        imu_state_ = ImuChannelState{};
        generation_ = 0;
    }

    std::string latestFrontNearestZone() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return lidar_state_.has_frame ? lidar_state_.frame.front_nearest_zone : std::string();
    }

    void updateLidar(
        const AutoAvoidInputBuilder::LidarInputFrame& frame,
        const std::chrono::steady_clock::time_point& received_steady) {
        std::lock_guard<std::mutex> lock(mutex_);
        lidar_state_.has_frame = true;
        lidar_state_.frame = frame;
        lidar_state_.received_steady = received_steady;
        ++lidar_state_.sequence;
        ++generation_;
    }

    void updateImu(
        bool has_attitude,
        double roll_deg,
        double pitch_deg,
        double yaw_deg,
        const std::chrono::steady_clock::time_point& received_steady) {
        std::lock_guard<std::mutex> lock(mutex_);
        imu_state_.has_sample = true;
        imu_state_.has_attitude = has_attitude;
        imu_state_.roll_deg = roll_deg;
        imu_state_.pitch_deg = pitch_deg;
        imu_state_.yaw_deg = yaw_deg;
        imu_state_.received_steady = received_steady;
        ++imu_state_.sequence;
        ++generation_;
    }

    ControlSnapshot buildControlSnapshot(
        const AutoAvoidInputBuilder& builder,
        const std::chrono::steady_clock::time_point& now,
        const std::chrono::steady_clock::duration& fresh_window) const {
        LidarChannelState lidar_state;
        ImuChannelState imu_state;
        std::uint64_t generation = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            lidar_state = lidar_state_;
            imu_state = imu_state_;
            generation = generation_;
        }

        const bool lidar_fresh =
            lidar_state.has_frame &&
            lidar_state.frame.valid &&
            lidar_state.frame.valid_points > 0 &&
            (now - lidar_state.received_steady) <= fresh_window;
        const bool imu_fresh =
            imu_state.has_sample &&
            imu_state.has_attitude &&
            (now - imu_state.received_steady) <= fresh_window;

        ControlSnapshot control_snapshot;
        control_snapshot.snapshot = builder.buildSnapshot(
            lidar_state.frame,
            lidar_fresh,
            imu_fresh,
            imu_state.yaw_deg,
            steadyMs(now));
        control_snapshot.control_snapshot_seq = generation;
        control_snapshot.control_snapshot_stamp_ms = steadyMs(
            latestChannelStamp(lidar_state, imu_state, now));
        control_snapshot.lidar_snapshot_seq = lidar_state.sequence;
        control_snapshot.imu_snapshot_seq = imu_state.sequence;
        control_snapshot.lidar_snapshot_age_ms =
            lidar_state.has_frame ? durationMs(now - lidar_state.received_steady) :
                                    std::numeric_limits<double>::quiet_NaN();
        control_snapshot.imu_snapshot_age_ms =
            imu_state.has_sample ? durationMs(now - imu_state.received_steady) :
                                   std::numeric_limits<double>::quiet_NaN();
        control_snapshot.control_snapshot_consistent =
            lidar_state.has_frame || imu_state.has_sample;
        control_snapshot.control_snapshot_fresh = lidar_fresh || imu_fresh;
        control_snapshot.control_snapshot_source =
            "auto_avoid_unified_lidar_imu_pool";

        control_snapshot.snapshot.control_snapshot_seq =
            control_snapshot.control_snapshot_seq;
        control_snapshot.snapshot.control_snapshot_stamp_ms =
            control_snapshot.control_snapshot_stamp_ms;
        control_snapshot.snapshot.lidar_snapshot_seq =
            control_snapshot.lidar_snapshot_seq;
        control_snapshot.snapshot.imu_snapshot_seq =
            control_snapshot.imu_snapshot_seq;
        control_snapshot.snapshot.lidar_snapshot_age_ms =
            control_snapshot.lidar_snapshot_age_ms;
        control_snapshot.snapshot.imu_snapshot_age_ms =
            control_snapshot.imu_snapshot_age_ms;
        control_snapshot.snapshot.control_snapshot_consistent =
            control_snapshot.control_snapshot_consistent;
        control_snapshot.snapshot.control_snapshot_fresh =
            control_snapshot.control_snapshot_fresh;
        control_snapshot.snapshot.control_snapshot_source =
            control_snapshot.control_snapshot_source;
        return control_snapshot;
    }

private:
    struct LidarChannelState {
        bool has_frame = false;
        std::uint64_t sequence = 0;
        std::chrono::steady_clock::time_point received_steady{};
        AutoAvoidInputBuilder::LidarInputFrame frame;
    };

    struct ImuChannelState {
        bool has_sample = false;
        std::uint64_t sequence = 0;
        std::chrono::steady_clock::time_point received_steady{};
        bool has_attitude = false;
        double roll_deg = 0.0;
        double pitch_deg = 0.0;
        double yaw_deg = 0.0;
    };

    static std::chrono::steady_clock::time_point latestChannelStamp(
        const LidarChannelState& lidar_state,
        const ImuChannelState& imu_state,
        const std::chrono::steady_clock::time_point& fallback) {
        if (lidar_state.has_frame && imu_state.has_sample) {
            return std::max(lidar_state.received_steady, imu_state.received_steady);
        }
        if (lidar_state.has_frame) {
            return lidar_state.received_steady;
        }
        if (imu_state.has_sample) {
            return imu_state.received_steady;
        }
        return fallback;
    }

    static std::int64_t steadyMs(
        const std::chrono::steady_clock::time_point& value) {
        return static_cast<std::int64_t>(std::llround(durationMs(value.time_since_epoch())));
    }

    template <typename Rep, typename Period>
    static double durationMs(const std::chrono::duration<Rep, Period>& value) {
        return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(value)
            .count();
    }

    mutable std::mutex mutex_;
    LidarChannelState lidar_state_;
    ImuChannelState imu_state_;
    std::uint64_t generation_ = 0;
};
