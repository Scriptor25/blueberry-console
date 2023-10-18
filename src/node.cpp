#include "node.h"

using std::placeholders::_1;

MainNode::MainNode(const std::string &topic_robot_status_report, const std::string &topic_robot_state) : Node("blueberry_node")
{
    m_Subscription_RobotStatusReport = create_subscription<edu_robot::msg::RobotStatusReport>(
        topic_robot_status_report,
        10,
        std::bind(&MainNode::on_robot_status_report, this, _1));

    m_Subscription_RobotState = create_subscription<edu_robot::msg::RobotState>(
        topic_robot_state,
        10,
        std::bind(&MainNode::on_robot_state, this, _1));
}

void MainNode::on_robot_status_report(const edu_robot::msg::RobotStatusReport::SharedPtr msg)
{
    m_Robot.DriveCurrent = msg->drive_current;
    m_Robot.DriveVoltage = msg->drive_voltage;
    m_Robot.MCUCurrent = msg->mcu_current;
    m_Robot.MCUVoltage = msg->mcu_voltage;
    if (msg->rpm.size() == 4)
    {
        m_Robot.RPM[0] = msg->rpm[0];
        m_Robot.RPM[1] = msg->rpm[1];
        m_Robot.RPM[2] = msg->rpm[2];
        m_Robot.RPM[3] = msg->rpm[3];
    }
    m_Robot.Temperature = msg->temperature;
}

void MainNode::on_robot_state(const edu_robot::msg::RobotState::SharedPtr msg)
{
    m_Robot.InfoMessage = msg->info_message;
    m_Robot.Mode = msg->mode.mode;
    m_Robot.FeatureMode = msg->mode.feature_mode;
    m_Robot.DriveKinematic = msg->mode.drive_kinematic;
    m_Robot.State = msg->state.value;
}
