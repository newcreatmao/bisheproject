#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "control/auto_avoid_control_snapshot_pool.hpp"
#include "control/auto_avoid.hpp"
#include "control/auto_avoid_input_builder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "web/httplib.h"

class ToStm;
class UART;

class LogDashboardServer {
public:
    explicit LogDashboardServer(std::string web_root = "");
    ~LogDashboardServer();

    bool listen(const std::string& host = "0.0.0.0", int port = 8080);
    void stop();

private:
    struct TopicRuntimeState {
        std::uint64_t message_count = 0;
        std::chrono::steady_clock::time_point last_message_steady_{};
    };

    struct GpsRuntimeState {
        std::uint64_t message_count = 0;
        std::chrono::steady_clock::time_point last_message_steady_{};
        int status = -1;
        bool has_position = false;
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
    };

    struct ImuRuntimeState {
        std::uint64_t message_count = 0;
        std::chrono::steady_clock::time_point last_message_steady_{};
        bool has_attitude = false;
        double roll_deg = 0.0;
        double pitch_deg = 0.0;
        double yaw_deg = 0.0;
    };

    struct DepthRuntimeState {
        std::uint64_t message_count = 0;
        std::chrono::steady_clock::time_point last_message_steady_{};
        bool valid = false;
        bool obstacle_detected = false;
        double nearest_m = 0.0;
        std::string nearest_zone;
        double obstacle_ratio = 0.0;
        int valid_pixels = 0;
        int obstacle_pixels = 0;
    };

    struct LidarRuntimeState {
        using SectorRuntimeState = AutoAvoidInputBuilder::SectorState;

        std::uint64_t message_count = 0;
        std::chrono::steady_clock::time_point last_message_steady_{};
        bool valid = false;
        bool obstacle_detected = false;
        double nearest_m = 0.0;
        double nearest_angle_deg = 0.0;
        int valid_points = 0;
        SectorRuntimeState negative_front_sector;
        SectorRuntimeState front_sector;
        SectorRuntimeState auto_avoid_front_sector;
        SectorRuntimeState positive_front_sector;
        SectorRuntimeState avoidance_buffer_sector;
        AutoAvoidInputBuilder::FrontTargetSelection front_target_selection;
        std::string front_nearest_zone;
    };

    struct VehicleCommandState {
        bool has_speed = false;
        int speed = 0;
        bool has_angle = false;
        int angle = 0;
    };

    struct AutoAvoidCommandTrace {
        bool sent_start = false;
        bool sent_angle = false;
        bool sent_speed = false;
        bool sent_stop = false;
        bool success = false;
        std::string result = "not_sent";
    };

    struct AutoAvoidRuntimeState {
        bool active = false;
        bool has_decision = false;
        std::int64_t last_control_cycle_id = 0;
        bool snapshot_valid = false;
        bool lidar_valid = false;
        bool imu_valid = false;
        AutoAvoidController::Command last_decision;
        std::string last_apply_result = "not_sent";
    };

    void configureRoutes();
    void startRosBridge();
    void stopRosBridge();
    void startStm32Bridge();
    void stopStm32Bridge();
    void startRuntimeLogBridge();
    void stopRuntimeLogBridge();
    void startAutoAvoidControl();
    void stopAutoAvoidControl();
    void runAutoAvoidControlLoop();
    AutoAvoidController::SensorSnapshot autoAvoidSensorSnapshot() const;
    AutoAvoidCommandTrace applyAutoAvoidCommand(
        const AutoAvoidController::Command& command,
        AutoAvoidController::Command& last_command,
        bool& has_last_command,
        std::chrono::steady_clock::time_point& last_command_sent_steady,
        std::int64_t control_cycle_id);
    void onDepthImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void onLidarScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
    void onGpsFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void onRgbYolo(const std_msgs::msg::String::SharedPtr msg);
    std::string stateJson() const;
    std::string latestRgbYoloPayloadJson() const;
    std::string latestSavedRgbYoloPayloadJson() const;
    bool stopModeSwitchReady(std::string& reason) const;

private:
    std::string web_root_;
    std::string storage_root_;
    httplib::Server server_;
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rgb_yolo_sub_;
    std::unique_ptr<UART> stm32_uart_;
    std::unique_ptr<ToStm> to_stm_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;
    std::atomic<bool> runtime_log_bridge_running_{false};
    std::thread runtime_log_bridge_thread_;
    std::atomic<bool> auto_avoid_control_running_{false};
    std::atomic<bool> auto_avoid_stm32_drive_enabled_{false};
    std::atomic<bool> stm32_emergency_active_{false};
    std::thread auto_avoid_control_thread_;
    mutable std::mutex state_mutex_;
    mutable std::mutex stm32_mutex_;
    AutoAvoidController auto_avoid_controller_;
    AutoAvoidInputBuilder auto_avoid_input_builder_;
    AutoAvoidControlSnapshotPool auto_avoid_control_snapshot_pool_;
    DepthRuntimeState depth_state_;
    LidarRuntimeState lidar_state_;
    ImuRuntimeState imu_state_;
    GpsRuntimeState gps_state_;
    std::string latest_rgb_yolo_payload_;
    std::chrono::steady_clock::time_point latest_rgb_yolo_received_steady_{};
    std::string active_workspace_mode_ = "STOP";
    bool manual_workspace_working_ = false;
    VehicleCommandState vehicle_command_state_;
    AutoAvoidRuntimeState auto_avoid_runtime_state_;
};
