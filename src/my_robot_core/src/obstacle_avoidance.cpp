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
    float get_min_range(const vector<float>& ranges, int start_idx, int end_idx) {
        float min_val = 10.0f;
        for (int i = start_idx; i < end_idx; ++i) {
            if (!isnan(ranges[i]) && !isinf(ranges[i])) {
                min_val = min(min_val, ranges[i]);
            }
        }
        return min_val;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Ön koniyi daralttık (-15 ile +15 derece arası), böylece yanından sıyırabileceği engellerden korkmayacak
        float min_front = get_min_range(msg->ranges, 165, 195);
        float min_right = get_min_range(msg->ranges, 90, 150);
        float min_left  = get_min_range(msg->ranges, 210, 270);
        
        // Geri viteste arkaya çarpmamak için arka bölgeleri kontrol et (-30 ile +30 derece arka koni)
        float min_back = min(get_min_range(msg->ranges, 0, 30), get_min_range(msg->ranges, 330, 360));

        // Daha cesur eşik değerleri (Metre)
        const float CRITICAL_DIST   = 0.30f; // 30 cm kalana kadar geri çıkma
        const float ENTER_TURN_DIST = 0.55f; // 55 cm kala manevraya başla
        const float EXIT_TURN_DIST  = 0.70f; // 70 cm açılınca düz gitmeye devam et

        auto drive_cmd = geometry_msgs::msg::Twist();

        switch (current_state_) {
            case RobotState::MOVING_FORWARD:
                if (min_front < CRITICAL_DIST) {
                    current_state_ = RobotState::BACKING_UP;
                    RCLCPP_ERROR(this->get_logger(), "COK YAKIN! Geri cikiliyor...");
                } 
                else if (min_front < ENTER_TURN_DIST) {
                    if (min_left > min_right) {
                        current_state_ = RobotState::TURNING_LEFT;
                        RCLCPP_WARN(this->get_logger(), "Manevra: SOLA");
                    } else {
                        current_state_ = RobotState::TURNING_RIGHT;
                        RCLCPP_WARN(this->get_logger(), "Manevra: SAGA");
                    }
                }
                break;

            case RobotState::TURNING_LEFT:
            case RobotState::TURNING_RIGHT:
                if (min_front < CRITICAL_DIST) {
                    current_state_ = RobotState::BACKING_UP;
                } 
                else if (min_front > EXIT_TURN_DIST) {
                    current_state_ = RobotState::MOVING_FORWARD;
                    RCLCPP_INFO(this->get_logger(), "Yol acildi, ileri!");
                }
                break;

            case RobotState::BACKING_UP:
                // Önü açıldıysa VEYA arkasında 25 cm'den az mesafe kaldıysa (duvara yaslanıyorsa) geri gitmeyi bırak
                if (min_front > ENTER_TURN_DIST || min_back < 0.25f) {
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