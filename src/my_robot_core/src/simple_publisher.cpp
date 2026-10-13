#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <string>

using namespace std;
using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node {
public:
    // Düğümün adını belirliyoruz ve sayacı sıfırlıyoruz
    SimplePublisher() : Node("simple_publisher"), count_(0) {
        // "chatter" adında bir konu (topic) oluşturduk. 10, mesaj kuyruğu boyutudur.
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        
        // 1 saniyede bir timer_callback fonksiyonunu otomatik tetikle
        timer_ = this->create_wall_timer(1s, bind(&SimplePublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "ROS 2 Yayini Aktif! Mesaj No: " + to_string(count_++);
        
        // Terminale log basalım ki çalıştığını görelim
        RCLCPP_INFO(this->get_logger(), "Yayinlanan: '%s'", message.data.c_str());
        
        // Mesajı ağa gönder
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<SimplePublisher>()); // Düğümü sonsuz döngüde yaşat
    rclcpp::shutdown();
    return 0;
}