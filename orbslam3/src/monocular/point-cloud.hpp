#ifndef __ORBSLAM3_POINTCLOUD_NODE_HPP__
#define __ORBSLAM3_POINTCLOUD_NODE_HPP__

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <thread>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>

#include "System.h"
#include "Frame.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "MapDrawer.h"
#include "MapPoint.h"
#include "LocalMapping.h"
#include "KeyFrame.h"
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


class ORBSLAM3PointCloud : public rclcpp::Node
{
public:
    ORBSLAM3PointCloud(ORB_SLAM3::System *pSLAM);

    ~ORBSLAM3PointCloud();

    void PublishMapPoint(ORB_SLAM3::System &SLAM);

    ORB_SLAM3::System *m_SLAM;

    geometry_msgs::msg::Pose current_robot_pose;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_publisher;
    
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_points_publisher;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr reference_map_points_publisher;

private:
    using ImageMsg = sensor_msgs::msg::Image;
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void UpdateRobotPose(const nav_msgs::msg::Odometry::SharedPtr msg);
    Eigen::Vector3f ComputeEndPoint(Eigen::Vector3f kframe, Eigen::Vector3f pos);

    cv_bridge::CvImagePtr m_cvImPtr;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Clock::SharedPtr clock;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;


    ORB_SLAM3::MapDrawer *mpMapDrawer;

};

#endif