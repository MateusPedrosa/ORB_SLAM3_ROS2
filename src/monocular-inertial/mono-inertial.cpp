#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-inertial-slam-node.hpp"

#include "System.h"


int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono-inertial path_to_vocabulary path_to_settings [imu_topic] [image_topic] [use_compressed]" << std::endl;
        std::cerr << "Example: ... /imu /camera/image_raw false" << std::endl;
        return 1;
    }

    // Set default topics
    std::string imu_topic = "/oceansim/robot/imu";
    std::string img_topic = "oceansim/robot/uw_img";
    bool use_compressed = true;

    // Override defaults if arguments are provided
    if (argc > 3) {
        imu_topic = std::string(argv[3]);
    }
    if (argc > 4) {
        img_topic = std::string(argv[4]);
    }
    if (argc > 5) {
        std::string arg_bool = std::string(argv[5]);
        if (arg_bool == "false" || arg_bool == "0" || arg_bool == "False") {
            use_compressed = false;
        }
    }

    // Display selected configuration
    std::cout << "Using IMU Topic: " << imu_topic << std::endl;
    std::cout << "Using Image Topic: " << img_topic << std::endl;

    rclcpp::init(argc, argv);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool visualization = true;
    // ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, visualization);
    auto SLAM = std::make_shared<ORB_SLAM3::System>(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, visualization);

    // auto node = std::make_shared<MonocularInertialSlamNode>(&SLAM, imu_topic, img_topic, use_compressed);
    auto node = std::make_shared<MonocularInertialSlamNode>(SLAM.get(), imu_topic, img_topic, use_compressed);
    std::cout << "============================ " << std::endl;\

    rclcpp::spin(node);

    std::cout << "Exiting..." << std::endl;

    // Destroy Node (Saves Trajectory, calls SLAM->Shutdown)
    node.reset();
    std::cout << "Node destroyed." << std::endl;
    
    // Destroy SLAM System (Closes Pangolin Viewer/Threads)
    // We do this BEFORE rclcpp::shutdown to prevent thread conflicts.
    SLAM.reset(); 
    std::cout << "SLAM System destroyed." << std::endl;

    rclcpp::shutdown();

    return 0;
}
