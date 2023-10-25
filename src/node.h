#pragma once

#include <string>

#include <edu_robot/msg/mode.hpp>
#include <edu_robot/msg/robot_state.hpp>
#include <edu_robot/msg/robot_status_report.hpp>
#include <edu_robot/msg/state.hpp>
#include <edu_robot/srv/set_mode.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>

// every status and latest info about the robot
struct Robot
{
    std::vector<float> DriveCurrent;
    std::vector<float> DriveVoltage;
    std::vector<float> MCUCurrent;
    std::vector<float> MCUVoltage;
    std::vector<float> Temperature;

    size_t n = 0;
};

struct Camera
{
    int Width = 0, Height = 0;
    void *Ptr;
};

class MainNode : public rclcpp::Node
{
public:
    MainNode(const std::string &, const std::string &, const std::string &, const std::string &);

    const Robot &GetRobot() const { return m_Robot; }
    const Camera &GetCamera() const { return m_Camera; }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr GetVelPub() { return m_Publisher_Velocity; }
    rclcpp::Client<edu_robot::srv::SetMode>::SharedPtr &GetSetMode() { return m_Client_SetMode; }

private:
    void on_robot_status_report(const edu_robot::msg::RobotStatusReport::SharedPtr);
    void on_cam_image(const sensor_msgs::msg::Image::SharedPtr);

private:
    Robot m_Robot;
    Camera m_Camera;

    rclcpp::Subscription<edu_robot::msg::RobotStatusReport>::SharedPtr m_Subscription_RobotStatusReport;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_Subscription_CamImage;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_Publisher_Velocity;

    rclcpp::Client<edu_robot::srv::SetMode>::SharedPtr m_Client_SetMode;
};
