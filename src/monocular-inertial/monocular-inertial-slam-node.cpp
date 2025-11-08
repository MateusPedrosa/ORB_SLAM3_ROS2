#include "monocular-inertial-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularInertialSlamNode::MonocularInertialSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();

    subImu_ = this->create_subscription<ImuMsg>("/rtimulib_node/imu", 1000, std::bind(&MonocularInertialSlamNode::GrabImu, this, _1));
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/camera/image_raw",//"oceansim/robot/uw_img",
        qos,
        std::bind(&MonocularInertialSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;

    syncThread_ = new std::thread(&MonocularInertialSlamNode::SyncWithImu, this);
}

MonocularInertialSlamNode::~MonocularInertialSlamNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularInertialSlamNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
    std::cout << "IMU - ang_vel: " << msg->angular_velocity.x << " " 
          << msg->angular_velocity.y << " " 
          << msg->angular_velocity.z << std::endl;
}

void MonocularInertialSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
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


void MonocularInertialSlamNode::GrabCompressedImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
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

cv::Mat MonocularInertialSlamNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonocularInertialSlamNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        cv::Mat im;
        double tIm = 0;
        if (!img0Buf.empty() && !imuBuf_.empty())
        {
            tIm = Utility::StampToSec(img0Buf.front()->header.stamp);

            if (tIm > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            mBufMutex.lock();
            im = GetImage(img0Buf.front());
            img0Buf.pop();
            mBufMutex.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tIm)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (bClahe_)
            {
                clahe_->apply(im, im);
            }

            // if (doRectify_)
            // {
            //     cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
            //     cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
            // }

            m_SLAM->TrackMonocular(im, tIm, vImuMeas);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}
