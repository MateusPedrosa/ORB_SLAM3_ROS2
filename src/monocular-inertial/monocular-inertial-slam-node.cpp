#include "monocular-inertial-slam-node.hpp"

#include<opencv2/core/core.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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
            10,
            std::bind(&MonocularInertialSlamNode::GrabImage, this, std::placeholders::_1)
        );
    }

    // Point cloud publisher
    m_pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/orb_slam3/map_points", 
        qos);
    
    // Publish point cloud periodically (every 1 second)
    m_pointcloud_timer = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MonocularInertialSlamNode::PublishMapPoints, this));

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

    // Save the point cloud
    m_SLAM->SavePointCloudMap("PointCloud.txt");

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularInertialSlamNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    if (!std::isnan(msg->linear_acceleration.x) && !std::isnan(msg->linear_acceleration.y) &&
        !std::isnan(msg->linear_acceleration.z) && !std::isnan(msg->angular_velocity.x) &&
        !std::isnan(msg->angular_velocity.y) && !std::isnan(msg->angular_velocity.z))
    {
        bufMutex_.lock();
        imuBuf_.push(msg);
        bufMutex_.unlock();
        // std::cout << "IMU - ang_vel: " << msg->angular_velocity.x << " " 
        //       << msg->angular_velocity.y << " " 
        //       << msg->angular_velocity.z << std::endl;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid IMU data - Rxd NaN");
    }
}

void MonocularInertialSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    if(imgRawBuf_.size() > 0)
    {
        imgRawBuf_.pop();
        RCLCPP_WARN(this->get_logger(), "Raw Image buffer overflow, dropping oldest frame");
    }
    imgRawBuf_.push(msg);
}

void MonocularInertialSlamNode::GrabCompressedImage(const CompressedImageMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    if(imgCompBuf_.size() > 0)
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

// void MonocularInertialSlamNode::SyncWithImu()
// {
//     while (rclcpp::ok())
//     {
//         double tIm = 0;
//         double tImu = 0;
//         bool hasImg = false;
//         bool hasImu = false;

//         // Check Image Buffer
//         mBufMutex.lock();
//         if (bUseCompressed_)
//         {
//             if (!imgCompBuf_.empty()) {
//                 hasImg = true;
//                 tIm = Utility::StampToSec(imgCompBuf_.front()->header.stamp);
//             }
//         }
//         else
//         {
//             if (!imgRawBuf_.empty()) {
//                 hasImg = true;
//                 tIm = Utility::StampToSec(imgRawBuf_.front()->header.stamp);
//             }
//         }
//         mBufMutex.unlock();

//         // Check IMU Buffer (Just peek at the back)
//         bufMutex_.lock();
//         if (!imuBuf_.empty()) {
//             hasImu = true;
//             tImu = Utility::StampToSec(imuBuf_.back()->header.stamp);
//         }
//         bufMutex_.unlock();

//         if (!hasImg || !hasImu || tIm > tImu)
//         {
//             std::this_thread::sleep_for(std::chrono::milliseconds(2));
//             continue;
//         }

//         // Pop and Decode Image
//         cv::Mat im;
//         mBufMutex.lock();
//         if (bUseCompressed_)
//         {
//             auto imgMsg = imgCompBuf_.front();
//             imgCompBuf_.pop();
//             mBufMutex.unlock(); // Unlock before expensive decode
//             im = GetCompressedImage(imgMsg);
//         }
//         else
//         {
//             auto imgMsg = imgRawBuf_.front();
//             imgRawBuf_.pop();
//             mBufMutex.unlock(); // Unlock before copy/decode
//             im = GetImage(imgMsg);
//         }

//         if(im.empty()) continue; // Skip bad images

//         // Resize image to improve performance (e.g., Scale by 0.5)
//         // cv::Mat imResized;
//         // float scale_factor = 0.5; 
//         // cv::resize(im, imResized, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);
//         // im = imResized;

//         vector<ORB_SLAM3::IMU::Point> vImuMeas;
//         bufMutex_.lock();
//         if (!imuBuf_.empty())
//         {
//             // Load imu measurements from buffer
//             vImuMeas.clear();
//             while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tIm)
//             {
//                 double t = Utility::StampToSec(imuBuf_.front()->header.stamp);

//                 // Validate IMU data before creating Point
//                 auto& imu = imuBuf_.front();
//                 if (std::isnan(imu->linear_acceleration.x) || 
//                     std::isnan(imu->angular_velocity.x))
//                 {
//                     RCLCPP_WARN(this->get_logger(), "NaN in IMU data, skipping");
//                     imuBuf_.pop();
//                     continue;
//                 }

//                 cv::Point3f acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
//                 cv::Point3f gyr(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
//                 vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
//                 imuBuf_.pop();
//             }
//         }
//         bufMutex_.unlock();

//         if (vImuMeas.empty())
//         {
//             RCLCPP_WARN(this->get_logger(), "No IMU measurements for image at time %.6f", tIm);
//             continue;
//         }

//         if (bClahe_)
//             clahe_->apply(im, im);

//         // if (doRectify_)
//         // {
//         //     cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
//         //     cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
//         // }

//         if (vImuMeas.empty())
//         {
//             RCLCPP_WARN(this->get_logger(), "Empty IMU vector, skipping frame");
//             continue; 
//         }

//         // Safety Check: ORB-SLAM usually needs at least 2 IMU meas to integrate
//         if (vImuMeas.size() < 2) 
//         {
//             continue;
//         }

//         std::cout << "Tracking with " << vImuMeas.size() << " IMU measurements" << std::endl;
//         m_SLAM->TrackMonocular(im, tIm, vImuMeas);

//         std::chrono::milliseconds tSleep(1);
//         std::this_thread::sleep_for(tSleep);
//     }
// }

void MonocularInertialSlamNode::SyncWithImu()
{
    while (rclcpp::ok())
    {
        // Lock both mutexes simultaneously to prevent race conditions during the whole cycle
        std::unique_lock<std::mutex> img_lock(mBufMutex, std::defer_lock);
        std::unique_lock<std::mutex> imu_lock(bufMutex_, std::defer_lock);
        
        std::lock(img_lock, imu_lock);

        // Peek at the front based on the selected topic type
        bool hasImg = false;
        double tImage = 0.0;

        if (bUseCompressed_)
        {
            if (!imgCompBuf_.empty()) {
                hasImg = true;
                tImage = Utility::StampToSec(imgCompBuf_.front()->header.stamp);
            }
        }
        else
        {
            if (!imgRawBuf_.empty()) {
                hasImg = true;
                tImage = Utility::StampToSec(imgRawBuf_.front()->header.stamp);
            }
        }

        // Process only if we have both Image and IMU data
        if (hasImg && !imuBuf_.empty())
        {
            // Decode the image immediately
            cv::Mat im;
            if (bUseCompressed_) {
                im = GetCompressedImage(imgCompBuf_.front());
            } else {
                im = GetImage(imgRawBuf_.front());
            }

            // Collect ALL available IMU measurements up to the image timestamp
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImage)
            {
                auto imuMsg = imuBuf_.front();
                double t = Utility::StampToSec(imuMsg->header.stamp);

                imuBuf_.pop();
                cv::Point3f acc(imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z);
                cv::Point3f gyr(imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z);
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
            }

            // Pop the processed image from the correct buffer
            if (bUseCompressed_) {
                imgCompBuf_.pop();
            } else {
                imgRawBuf_.pop();
            }

            // Track (only if image valid and we have enough IMU data)
            if (!im.empty() && !vImuMeas.empty() )
            {
                if (bClahe_) clahe_->apply(im, im);
                
                // Note: Tracking is now done INSIDE the lock, consistent with the reference file logic
                m_SLAM->TrackMonocular(im, tImage, vImuMeas);
            }
            else if (vImuMeas.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Not enough IMU measurements (%zu) for image at %.6f", vImuMeas.size(), tImage);
            }
            else if (im.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Not enough image measurements at %.6f", tImage);
            }
        }

        img_lock.unlock();
        imu_lock.unlock();

        // Sleep to prevent CPU spinning
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void MonocularInertialSlamNode::PublishMapPoints()
{
    // Get the atlas from ORB-SLAM3
    ORB_SLAM3::Atlas* pAtlas = m_SLAM->GetAtlas();
    if (!pAtlas)
    return;

    // Get the current map
    ORB_SLAM3::Map* pMap = pAtlas->GetCurrentMap();
    if(!pMap)
    {
        cout << "Map is empty!" << endl;
        return;
    }
    
    // Get all map points
    std::vector<ORB_SLAM3::MapPoint*> vpMPs = pMap->GetAllMapPoints();
    
    if (vpMPs.empty())
        return;
    
    // Count valid points
    int valid_points = 0;
    for (size_t i = 0; i < vpMPs.size(); i++)
    {
        if (!vpMPs[i]->isBad())
            valid_points++;
    }
    
    if (valid_points == 0)
        return;
    
    // Create PointCloud2 message
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "map";  // or "world" depending on your setup
    cloud_msg.height = 1;
    cloud_msg.width = valid_points;
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;
    
    // Setup fields
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    
    // Iterator for filling in data
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    
    // Fill in the point cloud data
    for (size_t i = 0; i < vpMPs.size(); i++)
    {
        if (vpMPs[i]->isBad())
            continue;
        
        Eigen::Vector3f pos = vpMPs[i]->GetWorldPos();
        
        // Transform from ORB-SLAM3 to ROS coordinates
        // ORB-SLAM3: X=right, Y=down, Z=forward
        // ROS:       X=forward, Y=left, Z=up
        *iter_x = pos(2);   // Z -> X (forward)
        *iter_y = -pos(0);  // -X -> Y (left)
        *iter_z = -pos(1);  // -Y -> Z (up)
        
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
    
    // Publish
    m_pointcloud_publisher->publish(cloud_msg);
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Published %d map points", valid_points);
}