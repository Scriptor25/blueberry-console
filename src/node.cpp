#include "node.h"

#include <stb_image.h>

using std::placeholders::_1;

MainNode::MainNode(const std::string &status, const std::string &velocity, const std::string &barcode, const std::string &setmode) : Node("blueberry_node")
{
    auto qos_policy = rclcpp::QoS(10);
    qos_policy.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    m_Subscription_RobotStatusReport = create_subscription<edu_robot::msg::RobotStatusReport>(
        status,
        qos_policy,
        std::bind(&MainNode::on_status, this, _1));
    m_Subscription_Barcode = create_subscription<std_msgs::msg::String>(
        barcode,
        qos_policy,
        std::bind(&MainNode::on_barcode, this, _1));

    m_Publisher_Velocity = create_publisher<geometry_msgs::msg::Twist>(velocity, 10);

    m_Client_SetMode = create_client<edu_robot::srv::SetMode>(setmode);
}

MainNode::~MainNode()
{
}

void MainNode::on_status(const edu_robot::msg::RobotStatusReport::ConstSharedPtr &msg)
{
    m_Robot.n++;

    m_Robot.DriveCurrent.push_back(msg->drive_current);
    m_Robot.DriveVoltage.push_back(msg->drive_voltage);
    m_Robot.MCUCurrent.push_back(msg->mcu_current);
    m_Robot.MCUVoltage.push_back(msg->mcu_voltage);
    m_Robot.Temperature.push_back(msg->temperature);

    while (m_Robot.n > 36)
    {
        m_Robot.n--;
        m_Robot.DriveCurrent.erase(m_Robot.DriveCurrent.begin());
        m_Robot.DriveVoltage.erase(m_Robot.DriveVoltage.begin());
        m_Robot.MCUCurrent.erase(m_Robot.MCUCurrent.begin());
        m_Robot.MCUVoltage.erase(m_Robot.MCUVoltage.begin());
        m_Robot.Temperature.erase(m_Robot.Temperature.begin());
    }
}

void MainNode::on_barcode(const std_msgs::msg::String::ConstSharedPtr &msg)
{
    if (!m_Barcodes[msg->data])
        m_Barcodes[msg->data] = 0;
    m_Barcodes[msg->data]++;
}
