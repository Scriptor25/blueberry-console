#pragma once

#include <string>

#include <edu_robot/msg/mode.hpp>
#include <edu_robot/msg/robot_state.hpp>
#include <edu_robot/msg/robot_status_report.hpp>
#include <edu_robot/msg/state.hpp>

#include <rclcpp/rclcpp.hpp>

// every status and latest info about the robot
struct Robot
{
    float DriveCurrent = 0;
    float DriveVoltage = 0;
    float MCUCurrent = 0;
    float MCUVoltage = 0;
    float RPM[4];
    float Temperature = 0;

    uint8_t Mode = 0;
    uint8_t DriveKinematic = 0;
    uint8_t FeatureMode = 0;

    uint8_t State = 0;

    std::string InfoMessage;
};

class MainNode : public rclcpp::Node
{
public:
    MainNode(const std::string &, const std::string &);

    const Robot &GetRobot() const { return m_Robot; }

private:
    void on_robot_status_report(const edu_robot::msg::RobotStatusReport::SharedPtr);
    void on_robot_state(const edu_robot::msg::RobotState::SharedPtr);

private:
    Robot m_Robot;

    rclcpp::Subscription<edu_robot::msg::RobotStatusReport>::SharedPtr m_Subscription_RobotStatusReport;
    rclcpp::Subscription<edu_robot::msg::RobotState>::SharedPtr m_Subscription_RobotState;
};
