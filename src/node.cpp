#include "node.h"

#include <stb_image.h>

using std::placeholders::_1;

MainNode::MainNode(const std::string &topic_robot_status_report, const std::string &topic_velocity, const std::string &topic_cam_image, const std::string &service_setmode) : Node("blueberry_node")
{
    auto qos_policy = rclcpp::QoS(10);
    qos_policy.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    m_Subscription_RobotStatusReport = create_subscription<edu_robot::msg::RobotStatusReport>(
        topic_robot_status_report,
        qos_policy,
        std::bind(&MainNode::on_robot_status_report, this, _1));
    // m_Subscription_CamImage = create_subscription<sensor_msgs::msg::CompressedImage>(
    //     topic_cam_image,
    //     qos_policy,
    //     std::bind(&MainNode::on_cam_image, this, _1));

    m_Publisher_Velocity = create_publisher<geometry_msgs::msg::Twist>(topic_velocity, 10);

    m_Client_SetMode = create_client<edu_robot::srv::SetMode>(service_setmode);

    glGenTextures(1, &m_Camera.Ptr);
    glBindTexture(GL_TEXTURE_2D, m_Camera.Ptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
}

MainNode::~MainNode()
{
    glDeleteTextures(1, &m_Camera.Ptr);
}

void MainNode::on_robot_status_report(const edu_robot::msg::RobotStatusReport::ConstSharedPtr &msg)
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

void MainNode::on_cam_image(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg)
{
    int channels;
    auto data = stbi_load_from_memory(msg->data.data(), msg->data.size(), &m_Camera.Width, &m_Camera.Height, &channels, STBI_rgb);
    if (!data)
    {
        printf("%s\r\n", stbi_failure_reason());
        return;
    }

    printf("got image %s %d [%d %d]\r\n", msg->format.c_str(), channels, m_Camera.Width, m_Camera.Height);

    glBindTexture(GL_TEXTURE_2D, m_Camera.Ptr);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, m_Camera.Width, m_Camera.Height, 0, GL_BGR, GL_UNSIGNED_BYTE, data);

    stbi_image_free(data);
}
