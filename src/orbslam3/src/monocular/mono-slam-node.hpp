#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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
#define CAM_INTRINSIC "/home/lucia/projeto/bombubi_ws/src/orbslam3/config/monocular/TUM1.yaml"

using namespace std;
using namespace cv;

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularSlamNode();

    void OccupancyGrid(ORB_SLAM3::System& SLAM);

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    bool CheckBoundary(int r, int c);
    void Bresenham(int r1, int c1, int r2, int c2);
    void PublishOccupancyMap(Mat &canvas);
    void PublishPose();
    void SaveCanvas();
    
    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::TimerBase::SharedPtr timer;
};

#endif