#include "mono-slam-node.hpp"

using std::placeholders::_1;

using namespace std;
using namespace cv;

const int cell_size = 800;
atomic<unsigned long long> atomic_cnts[2][cell_size][cell_size]; // 0:visited, 1:occupied
bool flag = 1;
// Represents current state of the robot
Sophus::SE3f nowS;
// Creates a blank canvas
Mat canvas(cell_size, cell_size, CV_8UC3, cv::Scalar(120, 120, 120));

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System *pSLAM)
    : Node("orbslam_map")
{
    m_SLAM = pSLAM;

    // Subscribe to the camera topic
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    // std::cout << "slam changed" << std::endl;

    // Publish the map topic
    map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "orboccupancygrid",
        10);

    // Publish robot pose
    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "bombubi_pose",
        10);

    // Timer to update the map and pose
    timer = this->create_wall_timer(
        200ms,
        [this]()
        {
            this->OccupancyGrid(*m_SLAM);
        });
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("/home/lucia/projeto/output/orbslam3map/KeyFrameTrajectory.txt");

    // Save canvas
    SaveCanvas();
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // std::cout<<"one frame has been sent"<<std::endl;

    nowS = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
}

bool MonocularSlamNode::CheckBoundary(int r, int c)
{
    if (r < 0 || c < 0 || r >= cell_size || c >= cell_size)
        return false;
    return true;
}

void MonocularSlamNode::Bresenham(int r1, int c1, int r2, int c2)
{
    if (CheckBoundary(r2, c2))
        atomic_cnts[1][r2][c2].fetch_add(1);
    if (c1 == c2)
    {
        if (r1 > r2)
            swap(r1, r2);

        while (r1 <= r2)
        {
            if (!CheckBoundary(r1, c1))
                break;
            atomic_cnts[0][r1][c1].fetch_add(1);
            r1++;
        }
    }
    else
    {
        if (c1 > c2)
        {
            swap(c1, c2);
            swap(r1, r2);
        }
        if (r1 == r2)
        {
            while (c1 <= c2)
            {
                if (!CheckBoundary(r1, c1))
                    break;
                atomic_cnts[0][r1][c1].fetch_add(1);
                c1++;
            }
        }
        else
        {
            if (r1 > r2)
            {
                r2 = r1 + (r1 - r2);

                int dr = r2 - r1;
                int dc = c2 - c1;

                if (dr <= dc)
                {
                    const int r0 = r1;
                    int p = 2 * dr - dc;
                    while (c1 <= c2)
                    {
                        if (!CheckBoundary(r0 - (r1 - r0), c1))
                            break;
                        atomic_cnts[0][r0 - (r1 - r0)][c1].fetch_add(1);
                        c1++;
                        if (p < 0)
                            p = p + 2 * dr;
                        else
                        {
                            p = p + 2 * dr - 2 * dc;
                            r1++;
                        }
                    }
                }
                else
                {
                    swap(dr, dc);
                    swap(c1, r1);
                    swap(c2, r2);
                    int p = 2 * dr - dc;
                    const int c0 = c1;
                    while (c1 <= c2)
                    {
                        if (!CheckBoundary(c0 - (c1 - c0), r1))
                            break;
                        atomic_cnts[0][c0 - (c1 - c0)][r1].fetch_add(1);
                        c1++;
                        if (p < 0)
                            p = p + 2 * dr;
                        else
                        {
                            p = p + 2 * dr - 2 * dc;
                            r1++;
                        }
                    }
                }
            }
            else
            {
                int dr = r2 - r1;
                int dc = c2 - c1;

                if (dc >= dr)
                {
                    int p = 2 * dr - dc;
                    while (c1 <= c2)
                    {
                        if (!CheckBoundary(r1, c1))
                            break;
                        atomic_cnts[0][r1][c1].fetch_add(1);
                        c1++;
                        if (p < 0)
                            p = p + 2 * dr;
                        else
                        {
                            p = p + 2 * dr - 2 * dc;
                            r1++;
                        }
                    }
                }
                else
                {
                    swap(dr, dc);
                    swap(c1, r1);
                    swap(c2, r2);
                    int p = 2 * dr - dc;
                    while (c1 <= c2)
                    {
                        if (!CheckBoundary(c1, r1))
                            break;
                        atomic_cnts[0][c1][r1].fetch_add(1);
                        c1++;
                        if (p < 0)
                            p = p + 2 * dr;
                        else
                        {
                            p = p + 2 * dr - 2 * dc;
                            r1++;
                        }
                    }
                }
            }
        }
    }
}

void MonocularSlamNode::PublishOccupancyMap(Mat &canvas)
{
    // Create the occupancy grid map object
    auto grid_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    grid_map->header.frame_id = "map";
    grid_map->info.resolution = 0.02; // Assuming 2 cm resolution

    grid_map->info.width = cell_size;
    grid_map->info.height = cell_size;

    grid_map->info.origin.position.x = 0.5 * grid_map->info.resolution * cell_size;
    grid_map->info.origin.position.y = 0.5 * grid_map->info.resolution * cell_size;
    grid_map->info.origin.position.z = 0;

    grid_map->info.origin.orientation.x = -0.7071068;
    grid_map->info.origin.orientation.y = 0.7071068;
    grid_map->info.origin.orientation.z = 0; // sin(90 degrees / 2)
    grid_map->info.origin.orientation.w = 0; // cos(90 degrees / 2)

    grid_map->data.resize(grid_map->info.width * grid_map->info.height, -1); // Initialize to unknown (-1)

    for (int i = 0; i < cell_size; ++i)
    {
        for (int j = 0; j < cell_size; ++j)
        {
            int visit_cnt = 0;
            int occupy_cnt = 0;
            for (int dr = -1; dr <= 1; ++dr)
            {
                for (int dc = -1; dc <= 1; ++dc)
                {
                    if (!CheckBoundary(i + dr, j + dc))
                        continue;
                    visit_cnt += atomic_cnts[0][i + dr][j + dc];
                    occupy_cnt += atomic_cnts[1][i + dr][j + dc];
                }
            }
            if (visit_cnt < 5)
                continue;

            const int percent = (occupy_cnt * 100) / visit_cnt;
            int index = i * cell_size + j;

            if (percent >= 15)
            {
                grid_map->data[index] = 100; // Occupied
                circle(canvas, Point(j, i), 0, Scalar(0, 0, 0), 3);
            }
            else
            {
                grid_map->data[index] = 0; // Free space
                circle(canvas, Point(j, i), 0, Scalar(255, 255, 255));
            }
        }
    }

    map_publisher->publish(*grid_map);
}

void MonocularSlamNode::PublishPose()
{
    // Create the pose stamped object
    auto pose_msg = geometry_msgs::msg::PoseStamped();

    // Set the frame ID
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = this->now();

    // Localization
    pose_msg.pose.position.x = nowS.translation().x();
    pose_msg.pose.position.y = 0;
    pose_msg.pose.position.z = nowS.translation().y();
    // pose_msg.pose.position.w = nowS.translation().w();

    pose_msg.pose.orientation.x = nowS.so3().unit_quaternion().x();
    pose_msg.pose.orientation.z = nowS.so3().unit_quaternion().y();
    pose_msg.pose.orientation.w = nowS.so3().unit_quaternion().w();

    pose_publisher->publish(pose_msg);
}

void MonocularSlamNode::OccupancyGrid(ORB_SLAM3::System &SLAM)
{
    const float res = 0.01; // 0.01 m/cell

    for (int i = 0; i < cell_size; ++i)
    {
        for (int j = 0; j < cell_size; ++j)
        {
            atomic_cnts[0][i][j].store(0);
            atomic_cnts[1][i][j].store(0);
        }
    }

    canvas.setTo(cv::Scalar(120, 120, 120));

    const auto mps = SLAM.mpAtlas->GetAllMapPoints();
    const int mps_len = mps.size();

    for (int i = 0; i < mps_len; ++i)
    {
        const auto p = mps[i]->GetWorldPos();
        const int c = cell_size / 2 + (int)(p(0) / res); // x
        const int r = cell_size / 2 - (int)(p(2) / res); // y
        const int z = (int)(p(1) / res);                 // z

        const auto p0 = mps[i]->GetReferenceKeyFrame()->GetPose().inverse().translation();
        const int c0 = cell_size / 2 + (int)(p0(0) / res); // x
        const int r0 = cell_size / 2 - (int)(p0(2) / res); // y
        const int z0 = (int)(p0(1) / res);                 // z

        // cut height above 1 meter
        if (abs(z - z0) > (int)(1.f / res))
            continue;

        Bresenham(r0, c0, r, c);
    }

    PublishOccupancyMap(canvas);

    PublishPose();

    // Draw the arrow that represents the location of the robot on the map
    /*const Sophus::Vector3f trans = nowS.translation();
    const Sophus::Vector3f dir = nowS.rotationMatrix().col(2);
    const int c = cell_size / 2 + (int)(trans(0) / res);              // x
    const int r = cell_size / 2 - (int)(trans(2) / res);              // y
    const int ratio[2] = {(int)(dir(0) / res), -(int)(dir(2) / res)}; // y
    const int dc = (20 * ratio[0]) / (abs(ratio[0]) + abs(ratio[1]));
    const int dr = (20 * ratio[1]) / (abs(ratio[0]) + abs(ratio[1]));

    if (CheckBoundary(r, c))
    {
        circle(canvas, Point(c, r), 0, Scalar(0, 0, 255), 10);
        // std::cout << dc << " " << dr << std::endl;
        arrowedLine(canvas, Point(c, r), Point(c + dc, r + dr), Scalar(0, 0, 255), 2, LINE_8, 0, 0.5);
    }

    imshow("Canvas", canvas);
    waitKey(1);*/
}

void MonocularSlamNode::SaveCanvas()
{
    // Create a filename with timestamp
    time_t rawtime;
    struct tm *timeinfo;
    char filename[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(filename, sizeof(filename), "/home/lucia/projeto/output/orbslam3map/canvas_%Y%m%d%H%M%S.png", timeinfo);

    // Save the canvas as an image
    Mat canvas(cell_size, cell_size, CV_8UC3, cv::Scalar(120, 120, 120));
    PublishOccupancyMap(canvas);
    imwrite(filename, canvas);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    ORB_SLAM3::System SLAM(VOCA_PATH, CAM_INTRINSIC, ORB_SLAM3::System::MONOCULAR, true);

    auto node = std::make_shared<MonocularSlamNode>(&SLAM);
    std::cout << "============================ " << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
