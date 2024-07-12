#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <nav_msgs/msg/odometry.hpp>


#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
//#include <omp.h>
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

#define VOCA_PATH "/home/lucia/projeto/bombubi_ws/src/orbslam3/vocabulary/ORBvoc.txt"
#define CAM_INTRINSIC "/home/lucia/projeto/bombubi_ws/src/orbslam3/config/monocular/Gazebo.yaml"

using namespace std;
using namespace cv;

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularSlamNode();

    void OccupancyGrid(ORB_SLAM3::System& SLAM);
    //void BroadcastDynamicTransform(const nav_msgs::msg::Odometry::SharedPtr msg);
    void PublishStaticTransform();

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
    //std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster; // Changed to unique_ptr
    //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

    // Set up QoS with transient local durability for map publisher
    rclcpp::QoS map_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    nav_msgs::msg::OccupancyGrid grid_map;
    

private:
    using ImageMsg = sensor_msgs::msg::Image;
    

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    bool CheckBoundary(int r, int c);
    void Bresenham(int r1, int c1, int r2, int c2);
    void PublishOccupancyMap();
    void PublishPose();
    
    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<ImageMsg>::SharedPtr m_image_subscriber;
    rclcpp::TimerBase::SharedPtr timer;
};

#endif