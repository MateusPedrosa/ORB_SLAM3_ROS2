#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();

    // m_image_subscriber = this->create_subscription<ImageMsg>(
    //     "/camera/image_raw",
    //     qos,
    //     std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    // std::cout << "slam changed" << std::endl;

    m_image_subscriber = this->create_subscription<CompressedImageMsg>(
        "oceansim/robot/uw_img",
        qos,
        std::bind(&MonocularSlamNode::GrabCompressedImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save the point cloud
    m_SLAM->SavePointCloudMap("PointCloud.txt");

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout<<"one frame has been sent"<<std::endl;
    m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
}


void MonocularSlamNode::GrabCompressedImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    try
    {
        // Decompress the JPEG data
        cv::Mat cv_image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        
        if (cv_image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
            return;
        }
        
        // Convert BGR to grayscale (ORB_SLAM3 needs grayscale)
        cv::Mat gray_image;
        cv::cvtColor(cv_image, gray_image, cv::COLOR_BGR2GRAY);
        
        std::cout << "one frame has been sent" << std::endl;
        
        // Track with ORB_SLAM3
        m_SLAM->TrackMonocular(gray_image, Utility::StampToSec(msg->header.stamp));
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error processing compressed image: %s", e.what());
        return;
    }
}