#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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

    // Point cloud publisher
    m_pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/orb_slam3/map_points", 
        qos);
    
    // Publish point cloud periodically (every 1 second)
    m_pointcloud_timer = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MonocularSlamNode::PublishMapPoints, this));
    
    std::cout << "SLAM node initialized with point cloud publisher" << std::endl;
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

void MonocularSlamNode::PublishMapPoints()
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