#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <thread>
#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "utility.hpp"

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>

#define VOCA_PATH "/home/lucia/projeto/bombubi_ws/src/orbslam3/vocabulary/ORBvoc.txt"
#define CAM_INTRINSIC "/home/lucia/projeto/bombubi_ws/src/orbslam3/config/monocular/Gazebo.yaml"

using namespace std;
using namespace cv;

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System *pSLAM);

    ~MonocularSlamNode();

    void OccupancyGrid(ORB_SLAM3::System &SLAM);
    void PublishLaserScan(ORB_SLAM3::System &SLAM);
    // void PublishStaticTransform();
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Clock::SharedPtr clock;

    ORB_SLAM3::System *m_SLAM;

    std::vector<std::pair<float, float>> stored_points_;  // Stores (range, angle) pairs

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_publisher;

    // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void InitialPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);


    bool initial_pose_captured = false;

    cv_bridge::CvImagePtr m_cvImPtr;

    std::mutex pose_mutex;
    nav_msgs::msg::Odometry current_odom;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
};

#endif