#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

constexpr uint8_t kHeader0 = 0xA5;
constexpr uint8_t kHeader1 = 0x5A;
constexpr int kPacketSize = 58;
constexpr int kPointsPerPacket = 16;
constexpr int kIntervalsPerPacket = kPointsPerPacket - 1;
// The N10 protocol and live serial capture both show an angular step of about
// 0.8 degrees per sample, which is roughly 450 points per full revolution.
constexpr int kFullScanPointCount = 450;
constexpr double kAngleIncrementDeg = 360.0 / static_cast<double>(kFullScanPointCount);

double wrapDegrees(double angle_deg)
{
  while (angle_deg < 0.0) {
    angle_deg += 360.0;
  }
  while (angle_deg >= 360.0) {
    angle_deg -= 360.0;
  }
  return angle_deg;
}

uint8_t packetCrc(const std::array<uint8_t, kPacketSize> & packet)
{
  unsigned int sum = 0;
  for (int i = 0; i < kPacketSize - 1; ++i) {
    sum += packet[static_cast<size_t>(i)];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

speed_t serialBaudConstant(int baud_rate)
{
  switch (baud_rate) {
    case 230400:
      return B230400;
    case 115200:
      return B115200;
    default:
      return B230400;
  }
}

}  // namespace

class Lsn10SerialScanNode : public rclcpp::Node
{
public:
  Lsn10SerialScanNode()
  : rclcpp::Node(
      "lslidar_driver_node",
      rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true))
  {
    frame_id_ = getParam("frame_id", std::string("laser"));
    scan_topic_ = getParam("scan_topic", std::string("/scan"));
    serial_port_ = getParam("serial_port_", std::string("/dev/lidar"));
    min_range_ = getParam("min_range", 0.2);
    max_range_ = getParam("max_range", 12.0);
    angle_disable_min_ = wrapDegrees(getParam("angle_disable_min", 0.0));
    angle_disable_max_ = wrapDegrees(getParam("angle_disable_max", 0.0));
    baud_rate_ = getParam("baud_rate", 230400);
    timing_debug_log_enabled_ = getParam("timing_debug_log_enabled", false);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, qos);
    resetScan();

    worker_ = std::thread(&Lsn10SerialScanNode::run, this);
    RCLCPP_INFO(
      get_logger(),
      "N10 serial scan node ready: port=%s baud=%d topic=%s frame=%s",
      serial_port_.c_str(),
      baud_rate_,
      scan_topic_.c_str(),
      frame_id_.c_str());
  }

  ~Lsn10SerialScanNode() override
  {
    running_.store(false);
    closeSerial();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  template<typename T>
  T getParam(const std::string & name, const T & default_value)
  {
    T value = default_value;
    this->get_parameter_or(name, value, default_value);
    return value;
  }

  bool serialOpen()
  {
    if (serial_fd_ >= 0) {
      return true;
    }

    const int fd = ::open(serial_port_.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "failed to open %s: %s",
        serial_port_.c_str(),
        std::strerror(errno));
      return false;
    }

    struct termios tio;
    std::memset(&tio, 0, sizeof(tio));
    tio.c_cflag = serialBaudConstant(baud_rate_) | CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cc[VTIME] = 1;
    tio.c_cc[VMIN] = 0;

    ::tcflush(fd, TCIFLUSH);
    if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
      RCLCPP_ERROR(
        get_logger(),
        "failed to configure %s: %s",
        serial_port_.c_str(),
        std::strerror(errno));
      ::close(fd);
      return false;
    }

    serial_fd_ = fd;
    buffer_.clear();
    last_packet_start_angle_ = 0.0;
    has_last_packet_ = false;
    RCLCPP_INFO(get_logger(), "opened N10 serial port %s", serial_port_.c_str());
    return true;
  }

  void closeSerial()
  {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void run()
  {
    while (running_.load() && rclcpp::ok()) {
      if (!serialOpen()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }

      std::array<uint8_t, 1024> chunk{};
      const ssize_t read_count = ::read(serial_fd_, chunk.data(), chunk.size());
      if (read_count < 0) {
        if (errno == EINTR || errno == EAGAIN) {
          continue;
        }
        RCLCPP_WARN(
          get_logger(),
          "serial read failed on %s: %s",
          serial_port_.c_str(),
          std::strerror(errno));
        closeSerial();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        continue;
      }

      if (read_count == 0) {
        continue;
      }

      buffer_.insert(buffer_.end(), chunk.begin(), chunk.begin() + read_count);
      parseBuffer();
    }
  }

  void parseBuffer()
  {
    size_t offset = 0;
    while (buffer_.size() >= offset + kPacketSize) {
      if (buffer_[offset] != kHeader0 || buffer_[offset + 1] != kHeader1) {
        ++offset;
        continue;
      }

      if (buffer_[offset + 2] != kPacketSize) {
        ++offset;
        continue;
      }

      std::array<uint8_t, kPacketSize> packet{};
      std::copy_n(buffer_.begin() + static_cast<std::ptrdiff_t>(offset), kPacketSize, packet.begin());
      if (packetCrc(packet) != packet[kPacketSize - 1]) {
        ++offset;
        continue;
      }

      handlePacket(packet);
      offset += kPacketSize;
    }

    if (offset > 0) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(offset));
    }

    if (buffer_.size() > static_cast<size_t>(kPacketSize * 4)) {
      buffer_.erase(buffer_.begin(), buffer_.end() - kPacketSize * 2);
    }
  }

  bool angleDisabled(double angle_deg) const
  {
    if (std::abs(angle_disable_min_ - angle_disable_max_) < 1e-6) {
      return false;
    }
    if (angle_disable_min_ < angle_disable_max_) {
      return angle_deg >= angle_disable_min_ && angle_deg <= angle_disable_max_;
    }
    return angle_deg >= angle_disable_min_ || angle_deg <= angle_disable_max_;
  }

  void handlePacket(const std::array<uint8_t, kPacketSize> & packet)
  {
    const auto packet_receive_time = this->now();
    const double start_angle =
      wrapDegrees(static_cast<double>((packet[5] << 8) | packet[6]) / 100.0);
    const double stop_angle =
      wrapDegrees(static_cast<double>((packet[55] << 8) | packet[56]) / 100.0);

    latest_scan_time_sec_ = static_cast<double>((packet[3] << 8) | packet[4]) * 24.0 / 1e6;

    if (has_last_packet_ && start_angle + 1.0 < last_packet_start_angle_ && packet_count_ > 10) {
      publishScan(packet_receive_time);
    }

    if (!has_current_scan_start_time_) {
      current_scan_start_time_ = packet_receive_time;
      has_current_scan_start_time_ = true;
    }

    addPacketPoints(packet, start_angle, stop_angle);
    last_packet_start_angle_ = start_angle;
    has_last_packet_ = true;
    ++packet_count_;
  }

  void addPacketPoints(
    const std::array<uint8_t, kPacketSize> & packet,
    double start_angle_deg,
    double stop_angle_deg)
  {
    double span_deg = stop_angle_deg - start_angle_deg;
    if (span_deg < 0.0) {
      span_deg += 360.0;
    }
    const double point_step_deg = span_deg / static_cast<double>(kIntervalsPerPacket);

    for (int i = 0; i < kPointsPerPacket; ++i) {
      const int base = 7 + i * 3;
      const uint16_t raw_distance = static_cast<uint16_t>((packet[base] << 8) | packet[base + 1]);
      const uint8_t raw_intensity = packet[base + 2];
      if (raw_distance == 0xFFFF) {
        continue;
      }

      const double angle_deg = wrapDegrees(start_angle_deg + point_step_deg * static_cast<double>(i));
      if (angleDisabled(angle_deg)) {
        continue;
      }

      const double range_m = static_cast<double>(raw_distance) / 1000.0;
      if (range_m < min_range_ || range_m > max_range_) {
        continue;
      }

      // Match the vendor driver publishing order for N10: 0 deg lands at the
      // last beam slot and angles decrease through the scan array.
      double ros_angle_deg = 360.0 - angle_deg;
      if (ros_angle_deg < 0.0) {
        ros_angle_deg += 360.0;
      }

      const double aligned_ros_angle_deg =
        wrapDegrees(ros_angle_deg + (kAngleIncrementDeg * 0.5));
      size_t point_index = static_cast<size_t>(aligned_ros_angle_deg / kAngleIncrementDeg);
      if (point_index >= static_cast<size_t>(kFullScanPointCount)) {
        point_index = 0;
      }

      if (range_m < current_ranges_[point_index]) {
        current_ranges_[point_index] = static_cast<float>(range_m);
        current_intensities_[point_index] = static_cast<float>(raw_intensity);
      }
    }
  }

  void publishScan(const rclcpp::Time & publish_time)
  {
    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.frame_id = frame_id_;
    scan_msg.angle_min = 0.0;
    scan_msg.angle_increment =
      static_cast<float>(2.0 * M_PI / static_cast<double>(kFullScanPointCount));
    scan_msg.angle_max = scan_msg.angle_min +
      scan_msg.angle_increment * static_cast<float>(kFullScanPointCount - 1);
    scan_msg.range_min = static_cast<float>(min_range_);
    scan_msg.range_max = static_cast<float>(max_range_);

    double scan_duration_sec = latest_scan_time_sec_;
    if (scan_duration_sec <= 0.0 && has_current_scan_start_time_) {
      scan_duration_sec = (publish_time - current_scan_start_time_).seconds();
    } else if (scan_duration_sec <= 0.0 && last_publish_time_.nanoseconds() > 0) {
      scan_duration_sec = (publish_time - last_publish_time_).seconds();
    }

    if (scan_duration_sec > 0.0) {
      scan_msg.scan_time = static_cast<float>(scan_duration_sec);
    } else {
      scan_msg.scan_time = 0.0F;
    }

    if (has_current_scan_start_time_) {
      scan_msg.header.stamp = current_scan_start_time_;
    } else if (scan_duration_sec > 0.0) {
      scan_msg.header.stamp = publish_time - rclcpp::Duration::from_seconds(scan_duration_sec);
    } else {
      scan_msg.header.stamp = publish_time;
    }
    scan_msg.time_increment = scan_msg.scan_time / static_cast<float>(kFullScanPointCount);
    scan_msg.ranges = current_ranges_;
    scan_msg.intensities = current_intensities_;
    scan_pub_->publish(scan_msg);

    if (timing_debug_log_enabled_) {
      const double stamp_age_ms = (publish_time - scan_msg.header.stamp).seconds() * 1000.0;
      const double measured_cycle_ms = has_current_scan_start_time_ ?
        (publish_time - current_scan_start_time_).seconds() * 1000.0 :
        stamp_age_ms;
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "scan timing: stamp=%.6f publish=%.6f age_ms=%.2f cycle_ms=%.2f vendor_scan_ms=%.2f",
        rclcpp::Time(scan_msg.header.stamp).seconds(),
        publish_time.seconds(),
        stamp_age_ms,
        measured_cycle_ms,
        latest_scan_time_sec_ * 1000.0);
    }

    last_publish_time_ = publish_time;
    resetScan();
  }

  void resetScan()
  {
    current_ranges_.assign(
      static_cast<size_t>(kFullScanPointCount),
      std::numeric_limits<float>::infinity());
    current_intensities_.assign(static_cast<size_t>(kFullScanPointCount), 0.0F);
    packet_count_ = 0;
    has_current_scan_start_time_ = false;
  }

  std::string frame_id_;
  std::string scan_topic_;
  std::string serial_port_;
  int baud_rate_ = 230400;
  double min_range_ = 0.2;
  double max_range_ = 12.0;
  double angle_disable_min_ = 0.0;
  double angle_disable_max_ = 0.0;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::thread worker_;
  std::atomic<bool> running_{true};
  int serial_fd_ = -1;
  std::vector<uint8_t> buffer_;
  std::vector<float> current_ranges_;
  std::vector<float> current_intensities_;
  int packet_count_ = 0;
  bool has_last_packet_ = false;
  double last_packet_start_angle_ = 0.0;
  double latest_scan_time_sec_ = 0.0;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time current_scan_start_time_{0, 0, RCL_ROS_TIME};
  bool has_current_scan_start_time_ = false;
  bool timing_debug_log_enabled_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Lsn10SerialScanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
