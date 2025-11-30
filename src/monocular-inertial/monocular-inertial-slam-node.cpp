#include "monocular-inertial-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularInertialSlamNode::MonocularInertialSlamNode(ORB_SLAM3::System* pSLAM, const std::string &imu_topic, const std::string &image_topic, bool use_compressed)
:   Node("ORB_SLAM3_ROS2"),
    bUseCompressed_(use_compressed)
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;

    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    subImu_ = this->create_subscription<ImuMsg>(
        imu_topic,
        1000,
        std::bind(&MonocularInertialSlamNode::GrabImu, this, _1)
    );

    if(bUseCompressed_)
    {
        std::cout << "Subscribing to Compressed Image topic: " << image_topic << std::endl;
        subImgCompressed_ = this->create_subscription<CompressedImageMsg>(
            image_topic,
            qos,
            std::bind(&MonocularInertialSlamNode::GrabCompressedImage, this, std::placeholders::_1)
        );
    }
    else
    {
        std::cout << "Subscribing to Raw Image topic: " << image_topic << std::endl;
        subImgRaw_ = this->create_subscription<ImageMsg>(
            image_topic,
            qos,
            std::bind(&MonocularInertialSlamNode::GrabImage, this, std::placeholders::_1)
        );
    }

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

void MonocularInertialSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    if(imgRawBuf_.size() > 5)
    {
        imgRawBuf_.pop();
        RCLCPP_WARN(this->get_logger(), "Raw Image buffer overflow, dropping oldest frame");
    }
    imgRawBuf_.push(msg);
}

void MonocularInertialSlamNode::GrabCompressedImage(const CompressedImageMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    if(imgCompBuf_.size() > 5)
    {
        imgCompBuf_.pop();
        RCLCPP_WARN(this->get_logger(), "Compressed Image buffer overflow, dropping oldest frame");
    }
    imgCompBuf_.push(msg);
}

cv::Mat MonocularInertialSlamNode::GetImage(const ImageMsg::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        // Force conversion to MONO8 for ORB_SLAM
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }

    return cv_ptr->image.clone();
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

        // Check Image Buffer
        mBufMutex.lock();
        if (bUseCompressed_)
        {
            if (!imgCompBuf_.empty()) {
                hasImg = true;
                tIm = Utility::StampToSec(imgCompBuf_.front()->header.stamp);
            }
        }
        else
        {
            if (!imgRawBuf_.empty()) {
                hasImg = true;
                tIm = Utility::StampToSec(imgRawBuf_.front()->header.stamp);
            }
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

        // Pop and Decode Image
        cv::Mat im;
        mBufMutex.lock();
        if (bUseCompressed_)
        {
            auto imgMsg = imgCompBuf_.front();
            imgCompBuf_.pop();
            mBufMutex.unlock(); // Unlock before expensive decode
            im = GetCompressedImage(imgMsg);
        }
        else
        {
            auto imgMsg = imgRawBuf_.front();
            imgRawBuf_.pop();
            mBufMutex.unlock(); // Unlock before copy/decode
            im = GetImage(imgMsg);
        }

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
