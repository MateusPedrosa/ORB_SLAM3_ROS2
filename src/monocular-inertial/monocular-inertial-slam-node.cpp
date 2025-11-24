#include "monocular-inertial-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularInertialSlamNode::MonocularInertialSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;

    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    subImu_ = this->create_subscription<ImuMsg>("/oceansim/robot/imu", 1000, std::bind(&MonocularInertialSlamNode::GrabImu, this, _1));
    m_image_subscriber = this->create_subscription<CompressedImageMsg>(
        "oceansim/robot/uw_img",
        qos,
        std::bind(&MonocularInertialSlamNode::GrabCompressedImage, this, std::placeholders::_1)
    );
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
    // std::cout << "IMU - ang_vel: " << msg->angular_velocity.x << " " 
    //       << msg->angular_velocity.y << " " 
    //       << msg->angular_velocity.z << std::endl;
}

// void MonocularInertialSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
// {
//     mBufMutex.lock();
//     img0Buf.push(msg);
//     mBufMutex.unlock();
// }


void MonocularInertialSlamNode::GrabCompressedImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    mBufMutex.lock();
    if(img0Buf.size() > 5)
    {
        img0Buf.pop();
        RCLCPP_WARN(this->get_logger(), "Image buffer overflow, dropping oldest frame");
    }
    img0Buf.push(msg);
    mBufMutex.unlock();
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

cv::Mat MonocularInertialSlamNode::GetCompressedImage(const CompressedImageMsg::SharedPtr msg)
{
    try
    {
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
        if (image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
            return cv::Mat();
        }

        if (image.type() == 0)
            return image.clone();

        RCLCPP_WARN(this->get_logger(), "Image type is %d, expected grayscale", image.type());
        return image.clone();
    }
    catch (const cv::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}

void MonocularInertialSlamNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (rclcpp::ok())
    {
        double tIm = 0;
        double tImu = 0;
        bool hasImg = false;
        bool hasImu = false;

        cv::Mat im;

        // Check Image Buffer
        mBufMutex.lock();
        if (!img0Buf.empty()) {
            hasImg = true;
            tIm = Utility::StampToSec(img0Buf.front()->header.stamp);
        }
        mBufMutex.unlock();

        // Check IMU Buffer (Just peek at the back)
        bufMutex_.lock();
        if (!imuBuf_.empty()) {
            hasImu = true;
            tImu = Utility::StampToSec(imuBuf_.back()->header.stamp);
        }
        bufMutex_.unlock();

        if (!hasImg || !hasImu || tIm > tImu)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        // Pop Image
        mBufMutex.lock();
        auto imgMsg = img0Buf.front();
        img0Buf.pop();
        mBufMutex.unlock();
        
        // Decode Image
        im = GetCompressedImage(imgMsg);
        if(im.empty()) continue; // Skip bad images

        // Resize image to improve performance (e.g., Scale by 0.5)
        cv::Mat imResized;
        float scale_factor = 0.5; 
        cv::resize(im, imResized, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);
        im = imResized;

        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        bufMutex_.lock();
        if (!imuBuf_.empty())
        {
            // Load imu measurements from buffer
            vImuMeas.clear();
            while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tIm)
            {
                double t = Utility::StampToSec(imuBuf_.front()->header.stamp);

                // Validate IMU data before creating Point
                auto& imu = imuBuf_.front();
                if (std::isnan(imu->linear_acceleration.x) || 
                    std::isnan(imu->angular_velocity.x))
                {
                    RCLCPP_WARN(this->get_logger(), "NaN in IMU data, skipping");
                    imuBuf_.pop();
                    continue;
                }

                cv::Point3f acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
                cv::Point3f gyr(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                imuBuf_.pop();
            }
        }
        bufMutex_.unlock();

        if (vImuMeas.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No IMU measurements for image at time %.6f", tIm);
            continue;
        }

        if (bClahe_)
            clahe_->apply(im, im);

        // if (doRectify_)
        // {
        //     cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
        //     cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
        // }

        if (vImuMeas.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty IMU vector, skipping frame");
            continue; 
        }

        // Safety Check: ORB-SLAM usually needs at least 2 IMU meas to integrate
        if (vImuMeas.size() < 2) 
        {
            continue;
        }

        std::cout << "Tracking with " << vImuMeas.size() << " IMU measurements" << std::endl;
        m_SLAM->TrackMonocular(im, tIm, vImuMeas);

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}
