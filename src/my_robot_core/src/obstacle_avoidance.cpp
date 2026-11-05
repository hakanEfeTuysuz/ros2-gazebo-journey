// Nav2'den bağımsız basit, durum makineli engel kaçınma düğümü.
// Nav2 ile aynı anda ÇALIŞTIRILMAMALIDIR: ikisi de cmd_vel'e yazar.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

enum class RobotState
{
  MOVING_FORWARD,
  TURNING_LEFT,
  TURNING_RIGHT,
  BACKING_UP
};

class ObstacleAvoidance : public rclcpp::Node
{
public:
  ObstacleAvoidance()
  : Node("obstacle_avoidance")
  {
    // Eşikler lidar merkezine göredir; lidar ön tampondan ~0.15 m geridedir.
    critical_dist_ = declare_parameter("critical_dist", 0.40);
    enter_turn_dist_ = declare_parameter("enter_turn_dist", 0.65);
    exit_turn_dist_ = declare_parameter("exit_turn_dist", 0.80);
    back_safe_dist_ = declare_parameter("back_safe_dist", 0.30);
    // Ön koni yarı açısı: 30 derece, 38 cm genişliği ve ön çıkıntıyı kapsar.
    front_half_angle_ = declare_parameter("front_half_angle", 0.5236);
    forward_speed_ = declare_parameter("forward_speed", 0.20);
    turn_speed_ = declare_parameter("turn_speed", 0.65);
    backup_speed_ = declare_parameter("backup_speed", 0.15);
    scan_timeout_ = declare_parameter("scan_timeout", 0.5);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {scan_callback(msg);});

    // Lidar verisi kesilirse robotu durdur.
    watchdog_ = create_wall_timer(100ms, [this]() {watchdog_callback();});

    RCLCPP_INFO(get_logger(), "Engel kacinma dugumu baslatildi.");
  }

  // Ctrl+C ile kapanırken robot son komutla gitmeye devam etmesin.
  void stop()
  {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  }

private:
  // center ± half_width açı aralığındaki (radyan, lidar çerçevesinde) en küçük geçerli
  // mesafe. Aralık -pi/+pi sınırını geçebilir; açı farkı normalize edilerek kontrol edilir.
  float min_range_in_sector(
    const sensor_msgs::msg::LaserScan & msg, double center, double half_width) const
  {
    float min_val = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      const double angle = msg.angle_min + static_cast<double>(i) * msg.angle_increment;
      const double diff = std::atan2(std::sin(angle - center), std::cos(angle - center));
      if (std::fabs(diff) > half_width) {
        continue;
      }
      const float r = msg.ranges[i];
      if (std::isfinite(r) && r >= msg.range_min && r <= msg.range_max) {
        min_val = std::min(min_val, r);
      }
    }
    return min_val;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg)
  {
    last_scan_time_ = now();

    // Ön: [-h, +h], yanlar: [h, 90°] ve [-90°, -h], arka: 180° ± 35°.
    const double h = front_half_angle_;
    const double side_center = (h + M_PI_2) / 2.0;
    const double side_half = (M_PI_2 - h) / 2.0;
    const float min_front = min_range_in_sector(*msg, 0.0, h);
    const float min_left = min_range_in_sector(*msg, side_center, side_half);
    const float min_right = min_range_in_sector(*msg, -side_center, side_half);
    const float min_back = min_range_in_sector(*msg, M_PI, 0.61);

    const RobotState previous = state_;
    switch (state_) {
      case RobotState::MOVING_FORWARD:
        if (min_front < critical_dist_ && min_back > back_safe_dist_) {
          state_ = RobotState::BACKING_UP;
        } else if (min_front < enter_turn_dist_) {
          state_ = (min_left > min_right) ? RobotState::TURNING_LEFT : RobotState::TURNING_RIGHT;
        }
        break;

      case RobotState::TURNING_LEFT:
      case RobotState::TURNING_RIGHT:
        // Sadece arkası gerçekten boşsa geri vitese geç (sonsuz döngüyü engeller).
        if (min_front < critical_dist_ && min_back > back_safe_dist_ + 0.10) {
          state_ = RobotState::BACKING_UP;
        } else if (min_front > exit_turn_dist_) {
          state_ = RobotState::MOVING_FORWARD;
        }
        break;

      case RobotState::BACKING_UP:
        if (min_front > enter_turn_dist_ || min_back < back_safe_dist_) {
          state_ = (min_left > min_right) ? RobotState::TURNING_LEFT : RobotState::TURNING_RIGHT;
        }
        break;
    }

    if (state_ != previous) {
      RCLCPP_INFO(
        get_logger(), "Durum: %s (on=%.2f sol=%.2f sag=%.2f arka=%.2f)",
        state_name(state_), min_front, min_left, min_right, min_back);
    }

    geometry_msgs::msg::Twist cmd;
    switch (state_) {
      case RobotState::MOVING_FORWARD:
        cmd.linear.x = forward_speed_;
        break;
      case RobotState::TURNING_LEFT:
        cmd.angular.z = turn_speed_;
        break;
      case RobotState::TURNING_RIGHT:
        cmd.angular.z = -turn_speed_;
        break;
      case RobotState::BACKING_UP:
        cmd.linear.x = -backup_speed_;
        cmd.angular.z = 0.3;
        break;
    }
    cmd_vel_pub_->publish(cmd);
  }

  void watchdog_callback()
  {
    if (last_scan_time_.nanoseconds() == 0) {
      return;
    }
    if ((now() - last_scan_time_).seconds() > scan_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Lidar verisi yok, robot durduruldu.");
      stop();
    }
  }

  static const char * state_name(RobotState s)
  {
    switch (s) {
      case RobotState::MOVING_FORWARD: return "ILERI";
      case RobotState::TURNING_LEFT: return "SOLA DON";
      case RobotState::TURNING_RIGHT: return "SAGA DON";
      case RobotState::BACKING_UP: return "GERI";
    }
    return "?";
  }

  double critical_dist_, enter_turn_dist_, exit_turn_dist_, back_safe_dist_;
  double front_half_angle_, forward_speed_, turn_speed_, backup_speed_, scan_timeout_;
  RobotState state_{RobotState::MOVING_FORWARD};
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleAvoidance>();
  rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
    [weak = std::weak_ptr<ObstacleAvoidance>(node)]() {
      if (auto n = weak.lock()) {n->stop();}
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
