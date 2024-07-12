#include "mono-slam-node.hpp"
#include <mutex>

using std::placeholders::_1;
using namespace std;
using namespace cv;

// Represents current state of the robot
Sophus::SE3f nowS;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System *pSLAM)
    : Node("orbslam_map")
{
    m_SLAM = pSLAM;

    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/camera/image_raw",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

    laserscan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    // Enable the use of simulation time
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // Create a clock object that uses the simulation time
    clock = this->get_clock();

    timer = this->create_wall_timer(
        500ms,
        [this]()
        {
            this->ProcessPointCloud(*m_SLAM);
        });


    RCLCPP_INFO(this->get_logger(), "MonocularSlamNode initialized.");
}

MonocularSlamNode::~MonocularSlamNode()
{
    m_SLAM->Shutdown();
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
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

void MonocularSlamNode::ProcessPointCloud(ORB_SLAM3::System &SLAM)
{
    if (nowS.matrix().isZero())
        return;

    // Extract map points from ORB-SLAM3
    const auto mps = SLAM.mpAtlas->GetAllMapPoints();

    if (mps.empty())
        return;

    const float height_max = 1.5f;
    const float height_min = 0.01f;

    // Lock to ensure thread safety when accessing odom data
    std::lock_guard<std::mutex> lock(pose_mutex);

    // Scaling factor
    const float scale_factor = 3.0f;

    // Create a LaserScan message
    sensor_msgs::msg::LaserScan laser_scan;
    laser_scan.header.stamp = this->now();
    laser_scan.header.frame_id = "base_footprint";
    laser_scan.angle_min = -M_PI;
    laser_scan.angle_max = M_PI;
    laser_scan.angle_increment = M_PI / 180;
    laser_scan.time_increment = 0.0;
    laser_scan.scan_time = 1.0 / 30.0;
    laser_scan.range_min = 0.1;
    laser_scan.range_max = 200.0;

    int num_readings = (laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment;
    laser_scan.ranges.assign(num_readings, std::numeric_limits<float>::infinity());

    for (const auto &mp : mps)
    {
        try
        {
            const auto &p = mp->GetWorldPos();

            // Print map point coordinates for verification
            std::cout << "Map Point Coordinates: (" << p(0) << ", " << p(1) << ", " << p(2) << ")" << std::endl;


            // Cut height above the threshold
            if (p(1) > height_max || p(1) < height_min )
                continue;

            float x = p(2);
            float y = p(0);

            float range = sqrt(x * x + y * y) * scale_factor; 
            float angle = atan2(y, x);

            // Print range and angle for verification
            std::cout << "Range: " << range << ", Angle: " << angle << std::endl;


            if (range >= laser_scan.range_min && range <= laser_scan.range_max &&
                angle >= laser_scan.angle_min && angle <= laser_scan.angle_max)
            {
                int index = (angle - laser_scan.angle_min) / laser_scan.angle_increment;
                if (range < laser_scan.ranges[index])
                {
                    laser_scan.ranges[index] = range;
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception caught while accessing GetWorldPos: %s", e.what());
            continue;
        }
    }

    laserscan_publisher->publish(laser_scan);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    ORB_SLAM3::System SLAM(VOCA_PATH, CAM_INTRINSIC, ORB_SLAM3::System::MONOCULAR, true);

    auto node = std::make_shared<MonocularSlamNode>(&SLAM);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
