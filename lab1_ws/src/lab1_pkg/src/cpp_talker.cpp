#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

using namespace std::chrono_literals;

class CppTalker : public rclcpp::Node {
    public:
        CppTalker() : Node("cpp_talker") {
            pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
            declare_parameter<double>("v", 0.5);
            declare_parameter<double>("d", 0.5);
            timer_ = this->create_wall_timer(1ms, [this]() {tick(); });
        }

    private:
        void tick() {
            auto msg = ackermann_msgs::msg::AckermannDriveStamped();
            msg.drive.speed = this->get_parameter("v").as_double();
            msg.drive.steering_angle = this->get_parameter("d").as_double();
            RCLCPP_INFO(this->get_logger(), "Publishing speed =%.2f, angle=%.2f", msg.drive.speed, 
                        msg.drive.steering_angle);
            pub_->publish(msg);
        }
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CppTalker>());
    rclcpp::shutdown();
    return 0;
}
