#pragma once

#include <string>

#include <edu_robot/msg/mode.hpp>
#include <edu_robot/msg/robot_state.hpp>
#include <edu_robot/msg/robot_status_report.hpp>
#include <edu_robot/msg/state.hpp>
#include <edu_robot/srv/set_mode.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

#include <GL/glew.h>

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
    GLuint Ptr = 0;
};

class MainNode : public rclcpp::Node
{
public:
    MainNode(const std::string &status, const std::string &velocity, const std::string &barcode, const std::string &setmode);
    ~MainNode();

    const Robot &GetRobot() const { return m_Robot; }
    const std::string &GetBarcode() const { return m_Barcode; }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr GetVelPub() { return m_Publisher_Velocity; }
    rclcpp::Client<edu_robot::srv::SetMode>::SharedPtr &GetSetMode() { return m_Client_SetMode; }

private:
    void on_status(const edu_robot::msg::RobotStatusReport::ConstSharedPtr &msg);
    void on_barcode(const std_msgs::msg::String::ConstSharedPtr &msg);

private:
    Robot m_Robot;
    std::string m_Barcode;

    rclcpp::Subscription<edu_robot::msg::RobotStatusReport>::SharedPtr m_Subscription_RobotStatusReport;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_Subscription_Barcode;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_Publisher_Velocity;

    rclcpp::Client<edu_robot::srv::SetMode>::SharedPtr m_Client_SetMode;
};
