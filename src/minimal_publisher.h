#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher();

private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr m_Timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_Publisher;
    size_t m_Count;
};