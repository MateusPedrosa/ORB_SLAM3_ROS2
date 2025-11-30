#ifndef __MONOCULAR_INERTIAL_SLAM_NODE_HPP__
#define __MONOCULAR_INERTIAL_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;

class MonocularInertialSlamNode : public rclcpp::Node
{
public:
    MonocularInertialSlamNode(ORB_SLAM3::System* pSLAM, const std::string &imu_topic, const std::string &image_topic, bool use_compressed);

    ~MonocularInertialSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using CompressedImageMsg = sensor_msgs::msg::CompressedImage;

    void GrabImu(const ImuMsg::SharedPtr msg);

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void GrabCompressedImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    cv::Mat GetCompressedImage(const CompressedImageMsg::SharedPtr msg);

    void SyncWithImu();
    

    ORB_SLAM3::System* m_SLAM;
    std::thread *syncThread_;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
    rclcpp::Subscription<CompressedImageMsg>::SharedPtr subImgCompressed_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgRaw_;
    // rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr m_image_subscriber;

    // IMU
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image
    // queue<ImageMsg::SharedPtr> img0Buf;
    // queue<CompressedImageMsg::SharedPtr> img0Buf;
    std::queue<CompressedImageMsg::SharedPtr> imgCompBuf_;
    std::queue<ImageMsg::SharedPtr> imgRawBuf_;
    std::mutex mBufMutex;

    bool bUseCompressed_;
    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
};

#endif
