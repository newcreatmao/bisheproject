#include "common/sensor_snapshot_pool.hpp"
#include "sensors/imu/imu_node.hpp"
#include "sensors/sensor_data_pool.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("project_imu_corrector");
    auto data_pool = std::make_shared<bishe::sensors::SensorDataPool>();
    auto snapshot_pool = std::make_shared<bishe::common::SensorSnapshotPool>();
    bishe::sensors::imu::ImuSensor imu_sensor(node.get(), data_pool, snapshot_pool);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
