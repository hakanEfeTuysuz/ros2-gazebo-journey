#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node {
public:
    SimpleSubscriber() : Node("simple_subscriber") {
        // "chatter" konusuna abone ol. Gelen mesajı topic_callback fonksiyonuna yolla.
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter", 10, bind(&SimpleSubscriber::topic_callback, this, _1));
    }

private:
    // shared_ptr ile hafıza yönetimi otomatik yapılıyor (ROS 2 standardı)
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Alinan: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<SimpleSubscriber>());
    rclcpp::shutdown();
    return 0;
}