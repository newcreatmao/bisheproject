#pragma once

#include <algorithm>
#include <atomic>
#include <optional>

#include "common/fixed_ring_buffer.hpp"
#include "common/sensor_types.hpp"

namespace bishe::common
{

class SensorSnapshotPool
{
public:
    void pushLidar(const LidarObstacleSummary & summary)
    {
        lidar_buffer_.push(summary);
        markUpdated();
    }

    void pushDepth(const DepthObstacleSummary & summary)
    {
        depth_buffer_.push(summary);
        markUpdated();
    }

    void pushImu(const ImuStateSummary & summary)
    {
        imu_buffer_.push(summary);
        markUpdated();
    }

    std::optional<LidarObstacleSummary> latestLidar() const
    {
        return lidar_buffer_.latest();
    }

    std::optional<DepthObstacleSummary> latestDepth() const
    {
        return depth_buffer_.latest();
    }

    std::optional<ImuStateSummary> latestImu() const
    {
        return imu_buffer_.latest();
    }

    LocalObservationSnapshot buildSnapshot(const rclcpp::Time & now) const
    {
        LocalObservationSnapshot snapshot;
        snapshot.snapshot_time = now;
        snapshot.generation = generation_.load(std::memory_order_relaxed);
        snapshot.latest_lidar = lidar_buffer_.latest();
        snapshot.latest_depth = depth_buffer_.latest();
        snapshot.latest_imu = imu_buffer_.latest();
        snapshot.lidar_history = lidar_buffer_.recent();
        snapshot.depth_history = depth_buffer_.recent();
        snapshot.imu_history = imu_buffer_.recent();
        snapshot.lidar_stats = lidar_buffer_.stats();
        snapshot.depth_stats = depth_buffer_.stats();
        snapshot.imu_stats = imu_buffer_.stats();

        auto consider_reference = [&](const auto & latest) {
            if (!latest || latest->stamp.nanoseconds() == 0) {
                return;
            }
            if (snapshot.reference_stamp.nanoseconds() == 0 ||
                latest->stamp.nanoseconds() > snapshot.reference_stamp.nanoseconds()) {
                snapshot.reference_stamp = latest->stamp;
            }
        };

        consider_reference(snapshot.latest_lidar);
        consider_reference(snapshot.latest_depth);
        consider_reference(snapshot.latest_imu);
        return snapshot;
    }

    void clear(bool reset_stats = true)
    {
        lidar_buffer_.clear(reset_stats);
        depth_buffer_.clear(reset_stats);
        imu_buffer_.clear(reset_stats);
        markUpdated();
    }

private:
    void markUpdated()
    {
        generation_.fetch_add(1, std::memory_order_relaxed);
    }

    FixedRingBuffer<LidarObstacleSummary, kLidarSummaryHistoryCapacity> lidar_buffer_;
    FixedRingBuffer<DepthObstacleSummary, kDepthSummaryHistoryCapacity> depth_buffer_;
    FixedRingBuffer<ImuStateSummary, kImuSummaryHistoryCapacity> imu_buffer_;
    std::atomic<std::uint64_t> generation_ {0};
};

}  // namespace bishe::common
