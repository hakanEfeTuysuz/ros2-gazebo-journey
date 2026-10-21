#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <vector>
#include <cmath>

using namespace std;
using std::placeholders::_1;

enum class RobotState {
    MOVING_FORWARD,
    TURNING_LEFT,
    TURNING_RIGHT,
    BACKING_UP
};

class ObstacleAvoidance : public rclcpp::Node {
public:
    ObstacleAvoidance() : Node("obstacle_avoidance_node"), current_state_(RobotState::MOVING_FORWARD) {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, bind(&ObstacleAvoidance::scan_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Cesur Engelden Kacma V3 (Dar Alan & Geri Vites Korumali) Baslatildi!");
    }

private:
    float get_min_range(const sensor_msgs::msg::LaserScan::SharedPtr& msg, int start_idx, int end_idx) {
        float min_val = 10.0f;
        int max_idx = std::min(end_idx, static_cast<int>(msg->ranges.size()));
        for (int i = std::max(0, start_idx); i < max_idx; ++i) {
            float r = msg->ranges[i];
            if (!std::isnan(r) && !std::isinf(r) && r >= msg->range_min) {
                min_val = std::min(min_val, r);
            }
        }
        return min_val;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Ön koni -30 ile +30 derece (150 - 210): Robotun 38 cm genişliğini ve 15 cm ön çıkıntısını korur
        float min_front = get_min_range(msg, 150, 210);
        float min_right = get_min_range(msg, 90, 150);
        float min_left  = get_min_range(msg, 210, 270);
        float min_back  = std::min(get_min_range(msg, 0, 35), get_min_range(msg, 325, 360));

        // LiDAR ön tampondan 15 cm geride olduğu için eşikler tampon payı dahil ayarlandı
        const float CRITICAL_DIST   = 0.40f; // Tampona ~25 cm kala
        const float ENTER_TURN_DIST = 0.65f; // Tampona ~50 cm kala manevra
        const float EXIT_TURN_DIST  = 0.80f;
        const float BACK_SAFE_DIST  = 0.30f;

        auto drive_cmd = geometry_msgs::msg::Twist();

        switch (current_state_) {
            case RobotState::MOVING_FORWARD:
                if (min_front < CRITICAL_DIST && min_back > BACK_SAFE_DIST) {
                    current_state_ = RobotState::BACKING_UP;
                    RCLCPP_ERROR(this->get_logger(), "COK YAKIN! Geri cikiliyor...");
                } 
                else if (min_front < ENTER_TURN_DIST) {
                    current_state_ = (min_left > min_right) ? RobotState::TURNING_LEFT : RobotState::TURNING_RIGHT;
                }
                break;

            case RobotState::TURNING_LEFT:
            case RobotState::TURNING_RIGHT:
                // Sadece arkası gerçekten boşsa geri vitese geç (sonsuz döngüyü engeller)
                if (min_front < CRITICAL_DIST && min_back > BACK_SAFE_DIST + 0.10f) {
                    current_state_ = RobotState::BACKING_UP;
                } 
                else if (min_front > EXIT_TURN_DIST) {
                    current_state_ = RobotState::MOVING_FORWARD;
                    RCLCPP_INFO(this->get_logger(), "Yol acildi, ileri!");
                }
                break;

            case RobotState::BACKING_UP:
                if (min_front > ENTER_TURN_DIST || min_back < BACK_SAFE_DIST) {
                    current_state_ = (min_left > min_right) ? RobotState::TURNING_LEFT : RobotState::TURNING_RIGHT;
                }
                break;
        }

        switch (current_state_) {
            case RobotState::MOVING_FORWARD:
                drive_cmd.linear.x  = 0.35;
                drive_cmd.angular.z = 0.0;
                break;
            case RobotState::TURNING_LEFT:
                drive_cmd.linear.x  = 0.0;
                drive_cmd.angular.z = 0.65;
                break;
            case RobotState::TURNING_RIGHT:
                drive_cmd.linear.x  = 0.0;
                drive_cmd.angular.z = -0.65;
                break;
            case RobotState::BACKING_UP:
                drive_cmd.linear.x  = -0.20; 
                drive_cmd.angular.z = 0.3;   
                break;
        }

        cmd_vel_pub_->publish(drive_cmd);
    }

    RobotState current_state_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ObstacleAvoidance>());
    rclcpp::shutdown();
    return 0;
}