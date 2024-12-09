#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class VelocityPublisher : public rclcpp::Node
{
public:
    VelocityPublisher()
    : Node("velocity_publisher"), velocity_(0.1)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/block_vel", 1);
        timer_ = this->create_wall_timer(
            17000ms, std::bind(&VelocityPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.y = velocity_;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.y);
        publisher_->publish(message);
        velocity_ = -velocity_;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double velocity_;
    rclcpp::Time last_toggle_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityPublisher>());
    rclcpp::shutdown();
    return 0;
}