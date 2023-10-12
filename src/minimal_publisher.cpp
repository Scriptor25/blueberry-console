#include "minimal_publisher.h"

using namespace std::chrono_literals;

MinimalPublisher::MinimalPublisher()
    : Node("minimal_publisher"), m_Count(0)
{
    m_Publisher = this->create_publisher<std_msgs::msg::String>("mini_pub", 10);
    m_Timer = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
}

void MinimalPublisher::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(m_Count++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    m_Publisher->publish(message);
}
