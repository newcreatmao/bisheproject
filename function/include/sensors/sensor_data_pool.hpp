#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vehicle/vehicle_state.hpp"

namespace bishe::sensors
{

struct FixedSamplePoolStats
{
    std::size_t cached = 0;
    std::size_t capacity = 0;
    std::uint64_t total_pushed = 0;
    std::uint64_t overwritten = 0;
};

template<typename T>
class FixedSamplePool
{
public:
    explicit FixedSamplePool(std::size_t capacity = 32)
        : storage_(std::max<std::size_t>(capacity, 1))
    {
    }

    void push(const T & sample)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool will_overwrite = (count_ == storage_.size());
        storage_[next_index_] = sample;
        next_index_ = (next_index_ + 1) % storage_.size();
        count_ = std::min(count_ + 1, storage_.size());
        ++push_count_;
        if (will_overwrite) {
            ++overwrite_count_;
        }
    }

    std::optional<T> latest() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (count_ == 0) {
            return std::nullopt;
        }

        const std::size_t index =
            (next_index_ + storage_.size() - 1) % storage_.size();
        return storage_[index];
    }

    std::vector<T> snapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<T> result;
        result.reserve(count_);
        if (count_ == 0) {
            return result;
        }

        const std::size_t start =
            (count_ == storage_.size()) ? next_index_ : 0;
        for (std::size_t i = 0; i < count_; ++i) {
            result.push_back(storage_[(start + i) % storage_.size()]);
        }
        return result;
    }

    std::size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return count_;
    }

    FixedSamplePoolStats stats() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        FixedSamplePoolStats stats;
        stats.cached = count_;
        stats.capacity = storage_.size();
        stats.total_pushed = push_count_;
        stats.overwritten = overwrite_count_;
        return stats;
    }

    void clear(bool reset_stats = true)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        next_index_ = 0;
        count_ = 0;
        if (reset_stats) {
            push_count_ = 0;
            overwrite_count_ = 0;
        }
    }

private:
    mutable std::mutex mutex_;
    std::vector<T> storage_;
    std::size_t next_index_ = 0;
    std::size_t count_ = 0;
    std::uint64_t push_count_ = 0;
    std::uint64_t overwrite_count_ = 0;
};

struct SensorSampleHeader
{
    rclcpp::Time stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    std::uint64_t sequence = 0;
};

struct GpsSample
{
    SensorSampleHeader header;
    bool has_fix = false;
    bool valid_position = false;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    int status = -1;
};

struct ImuSample
{
    SensorSampleHeader header;
    bool calibrated = false;
    double orientation_x = 0.0;
    double orientation_y = 0.0;
    double orientation_z = 0.0;
    double orientation_w = 1.0;
    double angular_velocity_x = 0.0;
    double angular_velocity_y = 0.0;
    double angular_velocity_z = 0.0;
    double linear_acceleration_x = 0.0;
    double linear_acceleration_y = 0.0;
    double linear_acceleration_z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double roll_offset = 0.0;
    double pitch_offset = 0.0;
    double yaw_offset = 0.0;
};

struct LidarSample
{
    SensorSampleHeader header;
    bishe::vehicle::ObstacleState obstacle;
};

struct CameraSample
{
    SensorSampleHeader header;
    // Depth-observation sample only. RGB + YOLO recognition stays on the
    // independent rgb_yolo_detector chain and is not stored here.
    bool depth_frame_received = false;
    std::string encoding;
    int width = 0;
    int height = 0;
    bishe::vehicle::VisionObstacleState vision_obstacle;
};

struct FusionOptions
{
    double freshness_timeout_sec = 1.0;
    double alignment_tolerance_sec = 0.15;
};

struct FusedSensorFrame
{
    rclcpp::Time reference_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    std::uint64_t generation = 0;
    std::optional<GpsSample> gps;
    std::optional<ImuSample> imu;
    std::optional<LidarSample> lidar;
    std::optional<CameraSample> camera;
    bishe::vehicle::SensorChannelDiagnostics gps_diag;
    bishe::vehicle::SensorChannelDiagnostics imu_diag;
    bishe::vehicle::SensorChannelDiagnostics lidar_diag;
    bishe::vehicle::SensorChannelDiagnostics camera_diag;
};

class SensorDataPool
{
public:
    explicit SensorDataPool(std::size_t capacity = 32)
        : gps_pool_(capacity),
          imu_pool_(capacity),
          lidar_pool_(capacity),
          camera_pool_(capacity)
    {
    }

    void pushGps(const GpsSample & sample)
    {
        gps_pool_.push(sample);
        markUpdated();
    }

    void pushImu(const ImuSample & sample)
    {
        imu_pool_.push(sample);
        markUpdated();
    }

    void pushLidar(const LidarSample & sample)
    {
        lidar_pool_.push(sample);
        markUpdated();
    }

    void pushCamera(const CameraSample & sample)
    {
        camera_pool_.push(sample);
        markUpdated();
    }

    std::optional<GpsSample> latestGps() const
    {
        return gps_pool_.latest();
    }

    std::optional<ImuSample> latestImu() const
    {
        return imu_pool_.latest();
    }

    std::optional<LidarSample> latestLidar() const
    {
        return lidar_pool_.latest();
    }

    std::optional<CameraSample> latestCamera() const
    {
        return camera_pool_.latest();
    }

    std::size_t gpsCount() const
    {
        return gps_pool_.size();
    }

    std::size_t imuCount() const
    {
        return imu_pool_.size();
    }

    std::size_t lidarCount() const
    {
        return lidar_pool_.size();
    }

    std::size_t cameraCount() const
    {
        return camera_pool_.size();
    }

    std::uint64_t generation() const
    {
        std::lock_guard<std::mutex> lock(generation_mutex_);
        return generation_;
    }

    void clear(bool reset_stats = true)
    {
        gps_pool_.clear(reset_stats);
        imu_pool_.clear(reset_stats);
        lidar_pool_.clear(reset_stats);
        camera_pool_.clear(reset_stats);
        markUpdated();
    }

    bool waitForUpdate(
        std::uint64_t last_generation,
        std::chrono::milliseconds timeout,
        std::uint64_t & observed_generation) const
    {
        std::unique_lock<std::mutex> lock(generation_mutex_);
        generation_cv_.wait_for(
            lock,
            timeout,
            [&]() {
                return generation_ != last_generation;
            });
        observed_generation = generation_;
        return observed_generation != last_generation;
    }

    void notifyWaiters() const
    {
        generation_cv_.notify_all();
    }

    FusedSensorFrame buildFusedFrame(
        const rclcpp::Time & now,
        const FusionOptions & options) const
    {
        FusedSensorFrame frame;
        frame.generation = generation();

        const auto gps_latest = gps_pool_.latest();
        const auto imu_latest = imu_pool_.latest();
        const auto lidar_latest = lidar_pool_.latest();
        const auto camera_latest = camera_pool_.latest();

        auto consider_reference = [&](const auto & sample) {
            if (!sample || sample->header.stamp.nanoseconds() == 0) {
                return;
            }
            if (frame.reference_stamp.nanoseconds() == 0 ||
                sample->header.stamp.nanoseconds() > frame.reference_stamp.nanoseconds()) {
                frame.reference_stamp = sample->header.stamp;
            }
        };

        consider_reference(gps_latest);
        consider_reference(imu_latest);
        consider_reference(lidar_latest);
        consider_reference(camera_latest);

        frame.gps = chooseSample(gps_pool_, gps_latest, frame.reference_stamp, options.alignment_tolerance_sec);
        frame.imu = chooseSample(imu_pool_, imu_latest, frame.reference_stamp, options.alignment_tolerance_sec);
        frame.lidar = chooseSample(lidar_pool_, lidar_latest, frame.reference_stamp, options.alignment_tolerance_sec);
        frame.camera = chooseSample(camera_pool_, camera_latest, frame.reference_stamp, options.alignment_tolerance_sec);

        frame.gps_diag = buildDiagnostics(
            gps_pool_, gps_latest, frame.gps, now, frame.reference_stamp, options);
        frame.imu_diag = buildDiagnostics(
            imu_pool_, imu_latest, frame.imu, now, frame.reference_stamp, options);
        frame.lidar_diag = buildDiagnostics(
            lidar_pool_, lidar_latest, frame.lidar, now, frame.reference_stamp, options);
        frame.camera_diag = buildDiagnostics(
            camera_pool_, camera_latest, frame.camera, now, frame.reference_stamp, options);

        return frame;
    }

private:
    template<typename T>
    static std::optional<T> chooseSample(
        const FixedSamplePool<T> & pool,
        const std::optional<T> & latest,
        const rclcpp::Time & reference_stamp,
        double alignment_tolerance_sec)
    {
        if (!latest) {
            return std::nullopt;
        }

        if (reference_stamp.nanoseconds() == 0 || alignment_tolerance_sec <= 0.0) {
            return latest;
        }

        const rclcpp::Duration tolerance =
            rclcpp::Duration::from_seconds(alignment_tolerance_sec);
        const auto history = pool.snapshot();

        std::optional<T> selected;
        std::int64_t best_delta_ns = std::numeric_limits<std::int64_t>::max();
        for (const auto & sample : history) {
            if (sample.header.stamp.nanoseconds() == 0) {
                continue;
            }

            const std::int64_t delta_ns =
                sample.header.stamp.nanoseconds() - reference_stamp.nanoseconds();
            const std::int64_t abs_delta_ns =
                delta_ns >= 0 ? delta_ns : -delta_ns;
            if (abs_delta_ns > tolerance.nanoseconds()) {
                continue;
            }

            if (!selected ||
                abs_delta_ns < best_delta_ns ||
                (abs_delta_ns == best_delta_ns &&
                 sample.header.stamp.nanoseconds() > selected->header.stamp.nanoseconds())) {
                selected = sample;
                best_delta_ns = abs_delta_ns;
            }
        }

        return selected ? selected : latest;
    }

    template<typename T>
    static bishe::vehicle::SensorChannelDiagnostics buildDiagnostics(
        const FixedSamplePool<T> & pool,
        const std::optional<T> & latest,
        const std::optional<T> & selected,
        const rclcpp::Time & now,
        const rclcpp::Time & reference_stamp,
        const FusionOptions & options)
    {
        bishe::vehicle::SensorChannelDiagnostics diagnostics;
        const auto stats = pool.stats();
        diagnostics.cached = stats.cached;
        diagnostics.capacity = stats.capacity;
        diagnostics.total_pushed = stats.total_pushed;
        diagnostics.overwritten = stats.overwritten;

        const std::optional<T> effective_sample = selected ? selected : latest;
        if (!effective_sample || effective_sample->header.stamp.nanoseconds() == 0) {
            return diagnostics;
        }

        diagnostics.available = true;
        diagnostics.sequence = effective_sample->header.sequence;
        diagnostics.age_ms =
            static_cast<double>(
                now.nanoseconds() - effective_sample->header.stamp.nanoseconds()) / 1e6;
        diagnostics.fresh =
            diagnostics.age_ms <= (options.freshness_timeout_sec * 1000.0);

        if (reference_stamp.nanoseconds() == 0) {
            diagnostics.aligned = true;
            diagnostics.skew_ms = 0.0;
            return diagnostics;
        }

        diagnostics.skew_ms =
            static_cast<double>(
                effective_sample->header.stamp.nanoseconds() -
                reference_stamp.nanoseconds()) / 1e6;
        const double abs_skew_ms =
            diagnostics.skew_ms >= 0.0 ? diagnostics.skew_ms : -diagnostics.skew_ms;
        diagnostics.aligned =
            abs_skew_ms <= (options.alignment_tolerance_sec * 1000.0);
        return diagnostics;
    }

    void markUpdated()
    {
        {
            std::lock_guard<std::mutex> lock(generation_mutex_);
            ++generation_;
        }
        generation_cv_.notify_all();
    }

    FixedSamplePool<GpsSample> gps_pool_;
    FixedSamplePool<ImuSample> imu_pool_;
    FixedSamplePool<LidarSample> lidar_pool_;
    FixedSamplePool<CameraSample> camera_pool_;
    mutable std::mutex generation_mutex_;
    mutable std::condition_variable generation_cv_;
    std::uint64_t generation_ = 0;
};

}  // namespace bishe::sensors
