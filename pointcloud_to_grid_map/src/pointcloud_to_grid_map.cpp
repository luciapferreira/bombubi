#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <vector>
#include <algorithm>

class PointCloudToGridMap : public rclcpp::Node
{
public:
    PointCloudToGridMap()
        : Node("pointcloud_to_grid_map")
    {
        this->declare_parameter<std::string>("pointcloud_topic", "/orbslam_pointcloud");
        this->declare_parameter<std::string>("occupancy_grid_topic", "/map");
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<double>("resolution", 0.05);
        this->declare_parameter<double>("radius_search", 0.1);
        this->declare_parameter<int>("min_neighbors", 3);

        this->get_parameter("pointcloud_topic", pointcloud_topic_);
        this->get_parameter("occupancy_grid_topic", occupancy_grid_topic_);
        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("resolution", resolution_);
        this->get_parameter("radius_search", radius_search_);
        this->get_parameter("min_neighbors", min_neighbors_);

        map_qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);
        map_qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);


        grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(occupancy_grid_topic_, map_qos_profile);
        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, 10, std::bind(&PointCloudToGridMap::pointcloud_callback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&PointCloudToGridMap::odom_callback, this, std::placeholders::_1));

        robot_x_ = 0.0;
        robot_y_ = 0.0;

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        PublishStaticTransform();
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", cloud->points.size());

        // Remove outliers using RadiusOutlierRemoval
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setInputCloud(cloud);
        ror.setRadiusSearch(radius_search_);
        ror.setMinNeighborsInRadius(min_neighbors_);
        ror.filter(*cloud_filtered);

        RCLCPP_INFO(this->get_logger(), "Filtered out %zu points (removed %zu points)", cloud->points.size(), cloud->points.size() - cloud_filtered->points.size());

        if (cloud_filtered->points.empty())
        {
            RCLCPP_WARN(this->get_logger(), "All points were removed by the outlier filter");
            return;
        }

        std::vector<double> xs, ys;
        for (const auto &point : cloud_filtered->points)
        {
            xs.push_back(point.x);
            ys.push_back(point.y);
        }

        double min_x = *std::min_element(xs.begin(), xs.end());
        double max_x = *std::max_element(xs.begin(), xs.end());
        double min_y = *std::min_element(ys.begin(), ys.end());
        double max_y = *std::max_element(ys.begin(), ys.end());

        double extent_x = max_x - min_x;
        double extent_y = max_y - min_y;

        // Ensure the robot is inside the map
        min_x = std::min(min_x, robot_x_);
        max_x = std::max(max_x, robot_x_);
        min_y = std::min(min_y, robot_y_);
        max_y = std::max(max_y, robot_y_);

        extent_x = max_x - min_x;
        extent_y = max_y - min_y;
        double max_extent = std::max(extent_x, extent_y);
        int grid_size = static_cast<int>(std::ceil((max_extent + 30) / resolution_));

        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header.stamp = this->now();
        grid_msg.header.frame_id = "map";
        grid_msg.info.resolution = resolution_;
        
        grid_msg.info.width = grid_size;
        grid_msg.info.height = grid_size;
        grid_msg.info.origin.position.x = min_x;
        grid_msg.info.origin.position.y = min_y;
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;
        grid_msg.data.resize(grid_size * grid_size, -1);

        for (const auto &point : cloud_filtered->points)
        {
            int grid_x = static_cast<int>((point.x - min_x) / resolution_);
            int grid_y = static_cast<int>((point.y - min_y) / resolution_);
            if (grid_x >= 0 && grid_x < grid_size && grid_y >= 0 && grid_y < grid_size)
            {
                grid_msg.data[grid_y * grid_size + grid_x] = 100;
                GenerateFreePoints (grid_msg, robot_x_, robot_y_, point.x, point.y, min_x, min_y);
            }
        }

        grid_publisher_->publish(grid_msg);

        //RCLCPP_INFO(this->get_logger(), "Published a grid map of size %d x %d with resolution %.2f", grid_size, grid_size, resolution_);
    }

    void GenerateFreePoints (nav_msgs::msg::OccupancyGrid &grid_msg, double x0, double y0, double x1, double y1, double min_x, double min_y)
    {
        int grid_size = grid_msg.info.width;
        int x0_idx = static_cast<int>((x0 - min_x) / resolution_);
        int y0_idx = static_cast<int>((y0 - min_y) / resolution_);
        int x1_idx = static_cast<int>((x1 - min_x) / resolution_);
        int y1_idx = static_cast<int>((y1 - min_y) / resolution_);

        int dx = abs(x1_idx - x0_idx);
        int dy = abs(y1_idx - y0_idx);
        int sx = (x0_idx < x1_idx) ? 1 : -1;
        int sy = (y0_idx < y1_idx) ? 1 : -1;
        int err = dx - dy;

        while (x0_idx != x1_idx || y0_idx != y1_idx)
        {
            if (x0_idx >= 0 && x0_idx < grid_size && y0_idx >= 0 && y0_idx < grid_size)
            {
                grid_msg.data[y0_idx * grid_size + x0_idx] = 0;
            }

            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x0_idx += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y0_idx += sy;
            }
        }
    }

    void PublishStaticTransform()
    {
        geometry_msgs::msg::TransformStamped static_transform_stamped;

        static_transform_stamped.header.stamp = this->get_clock()->now();
        static_transform_stamped.header.frame_id = "map";
        static_transform_stamped.child_frame_id = "odom";
        static_transform_stamped.transform.translation.x = 0.0;
        static_transform_stamped.transform.translation.y = 0.0;
        static_transform_stamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        static_transform_stamped.transform.rotation.x = q.x();
        static_transform_stamped.transform.rotation.y = q.y();
        static_transform_stamped.transform.rotation.z = q.z();
        static_transform_stamped.transform.rotation.w = q.w();

        tf_static_broadcaster_->sendTransform(static_transform_stamped);
    }

    std::string pointcloud_topic_;
    rclcpp::QoS map_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    std::string occupancy_grid_topic_;
    std::string odom_topic_;
    double resolution_;
    double radius_search_;
    int min_neighbors_;
    double robot_x_;
    double robot_y_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudToGridMap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}