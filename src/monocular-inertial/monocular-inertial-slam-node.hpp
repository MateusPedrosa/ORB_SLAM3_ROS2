#ifndef __MONOCULAR_INERTIAL_SLAM_NODE_HPP__
#define __MONOCULAR_INERTIAL_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;

class MonocularInertialSlamNode : public rclcpp::Node
{
public:
    MonocularInertialSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularInertialSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using CompressedImageMsg = sensor_msgs::msg::CompressedImage;

    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();

    void GrabCompressedImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;
    std::thread *syncThread_;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;

    // IMU
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image
    queue<ImageMsg::SharedPtr> img0Buf;
    std::mutex mBufMutex;

    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
};

#endif
