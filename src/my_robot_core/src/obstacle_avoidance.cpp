#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <vector>

using namespace std;
using std::placeholders::_1;

class ObstacleAvoidance : public rclcpp::Node {
public:
    ObstacleAvoidance() : Node("obstacle_avoidance_node") {
        // Motorlara hız komutu gönderecek Publisher (/cmd_vel)
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // LIDAR verisini dinleyecek Subscriber (/scan)
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, bind(&ObstacleAvoidance::scan_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Otonom Engelden Kacma Dugumu Aktif!");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // LIDAR 360 derecede 360 eleman gönderiyor:
        // 0: Arka, 90: Sağ, 180: Tam Ön, 270: Sol
        
        // Ön koni: 150. indeks (-30 derece) ile 210. indeks (+30 derece) arasındaki en yakın mesafe
        float min_front_dist = *min_element(msg->ranges.begin() + 150, msg->ranges.begin() + 210);

        // Sağ yan (90..150) ve Sol yan (210..270) bölgelerdeki en yakın mesafeler
        float min_right_dist = *min_element(msg->ranges.begin() + 90, msg->ranges.begin() + 150);
        float min_left_dist  = *min_element(msg->ranges.begin() + 210, msg->ranges.begin() + 270);

        auto drive_cmd = geometry_msgs::msg::Twist();
        const float SAFE_DISTANCE = 1.0; // 1.0 metre güvenlik sınırı

        if (min_front_dist < SAFE_DISTANCE) {
            // Önde engel var! İleri hareketi kes ve daha boş olan yöne dön
            drive_cmd.linear.x = 0.0;
            
            if (min_left_dist > min_right_dist) {
                drive_cmd.angular.z = 0.6;  // Sol taraf daha ferah -> Sola dön
                RCLCPP_WARN(this->get_logger(), "ENGEL! (On: %.2fm) -> Sola manevra yapiliyor...", min_front_dist);
            } else {
                drive_cmd.angular.z = -0.6; // Sağ taraf daha ferah -> Sağa dön
                RCLCPP_WARN(this->get_logger(), "ENGEL! (On: %.2fm) -> Saga manevra yapiliyor...", min_front_dist);
            }
        } else {
            // Önümüz açık, düz ilerle
            drive_cmd.linear.x = 0.35;
            drive_cmd.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Yol acik (On: %.2fm) -> Ilerleniyor.", min_front_dist);
        }

        // Hesaplanan hız komutunu motorlara gönder
        cmd_vel_pub_->publish(drive_cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ObstacleAvoidance>());
    rclcpp::shutdown();
    return 0;
}