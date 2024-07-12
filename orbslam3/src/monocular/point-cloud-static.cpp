#include "point-cloud.hpp"

using std::placeholders::_1;
using namespace std;
using namespace cv;

// Represents current state of the robot
Sophus::SE3f nowS;

ORBSLAM3PointCloud::ORBSLAM3PointCloud(ORB_SLAM3::System *pSLAM)
    : Node("orbslam_pointcloud")
{
    m_SLAM = pSLAM;

    // Enable the use of simulation time
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    clock = this->get_clock();

    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/camera/image_raw",
        5,
        std::bind(&ORBSLAM3PointCloud::GrabImage, this, std::placeholders::_1));

    map_points_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("orbslam_pointcloud", 10);

    // Subscribe to the robot's odometry in Gazebo
    odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 
        10, 
        std::bind(&ORBSLAM3PointCloud::UpdateRobotPose, this, _1));

    timer = this->create_wall_timer(
        500ms,
        [this]()
        {
            this->PublishMapPoint(*m_SLAM);
        });

    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 point cloud extraction initialized.");
}

ORBSLAM3PointCloud::~ORBSLAM3PointCloud()
{
    m_SLAM->Shutdown();
}

void ORBSLAM3PointCloud::GrabImage(const ImageMsg::SharedPtr msg)
{
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    nowS = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
}

void ORBSLAM3PointCloud::UpdateRobotPose(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_robot_pose = msg->pose.pose;
}

void ORBSLAM3PointCloud::PublishMapPoint(ORB_SLAM3::System &SLAM)
{
    if (nowS.matrix().isZero())
        return;

    mpMapDrawer = SLAM.mpMapDrawer;
    if (!mpMapDrawer)
        return;

    ORB_SLAM3::Map *pActiveMap = mpMapDrawer->mpAtlas->GetCurrentMap();
    if (!pActiveMap)
        return;

    std::vector<ORB_SLAM3::MapPoint*> map_points = SLAM.mpAtlas->GetAllMapPoints();

    if (map_points.empty())
        return;

    // Create a PointCloud2 message
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pointcloud_msg.header.stamp = this->now();
    pointcloud_msg.header.frame_id = "map"; 
    pointcloud_msg.height = 1;

    // Set the width to the number of points
    pointcloud_msg.width = map_points.size();
    pointcloud_msg.is_dense = true;

    // Define fields for x, y, and z
    pointcloud_msg.fields.resize(3);

    pointcloud_msg.fields[0].name = "x";
    pointcloud_msg.fields[0].offset = 0;
    pointcloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud_msg.fields[0].count = 1;

    pointcloud_msg.fields[1].name = "y";
    pointcloud_msg.fields[1].offset = 4;
    pointcloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud_msg.fields[1].count = 1;

    pointcloud_msg.fields[2].name = "z";
    pointcloud_msg.fields[2].offset = 8;
    pointcloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud_msg.fields[2].count = 1;

    size_t point_step = 12;  // 4 bytes each for x, y, z
    size_t row_step = point_step * map_points.size();
    pointcloud_msg.point_step = point_step;
    pointcloud_msg.row_step = row_step;
    pointcloud_msg.data.resize(row_step);
    pointcloud_msg.is_bigendian = false;

    // Create iterators for x, y, z
    auto it_x = sensor_msgs::PointCloud2Iterator<float>(pointcloud_msg, "x");
    auto it_y = sensor_msgs::PointCloud2Iterator<float>(pointcloud_msg, "y");
    auto it_z = sensor_msgs::PointCloud2Iterator<float>(pointcloud_msg, "z");

    double scale = 1.5;
    for (auto mp : map_points)
    {
        const auto pos = mp->GetWorldPos();
        *it_x = pos[0] * 2.4;
        *it_y = pos[2] * 2.3;
        *it_z = abs(pos[1]);
        ++it_x;
        ++it_y;
        ++it_z;
    }

    // Publish the PointCloud2 message
    map_points_publisher->publish(pointcloud_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    ORB_SLAM3::System SLAM(VOCA_PATH, CAM_INTRINSIC, ORB_SLAM3::System::MONOCULAR, true);

    auto node = std::make_shared<ORBSLAM3PointCloud>(&SLAM);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
