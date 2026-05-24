/**
 * @file go2_slam.cpp
 * @brief Go2 SLAM 建图与 Nav2 自主导航程序，桥接 Go2 传感器数据到 ROS2 导航栈
 *
 * @par 使用说明
 *       go2_slam <network_interface>
 *       示例: ./go2_slam eth0
 *       说明: 本程序作为 Go2 机器人与 ROS2 Nav2 导航栈之间的桥接节点，
 *             将 Go2 的雷达点云、里程计数据转换为标准 ROS2 消息，
 *             并通过 TF 广播坐标变换，同时接收 Nav2 规划的速度指令驱动 Go2 运动。
 *             订阅 /map 话题实时显示建图状态（ASCII 缩略图）。
 *       配合: 需同时在另一个终端启动如下 ROS2 节点：
 *             ros2 launch slam_toolbox online_async_launch.py
 *             以及 Nav2 导航栈节点 (amcl / map_server / planner_server / controller_server / bt_navigator)
 *       控制: 在终端输入命令后按回车确认：
 *             m     - 开始建图
 *             v     - 显示当前地图（ASCII 缩略图）
 *             s     - 保存地图
 *             g / 1 - 发送导航目标 (x=2, y=0, yaw=0)  向前 2 米
 *             2     - 发送导航目标 (x=0, y=2, yaw=π/2) 向左 2 米
 *             3     - 发送导航目标 (x=2, y=2, yaw=π/4) 右前方
 *             q     - 退出程序
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "go2_motion_bridge.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <cmath>
#include <cstring>
#include <limits>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <queue>

/// @brief 雷达扫描参数
constexpr double LIDAR_ANGLE_MIN      = -M_PI;
constexpr double LIDAR_ANGLE_MAX      =  M_PI;
constexpr double LIDAR_RANGE_MIN      =  0.05;
constexpr double LIDAR_RANGE_MAX      = 20.0;
constexpr int    LASER_SCAN_SAMPLES   = 720;
constexpr double LIDAR_Z_THRESHOLD    = 0.3;

/// @brief 导航控制参数
constexpr float NAV_FORWARD_SPEED_MAX = 0.30f;
constexpr float NAV_ANGULAR_SPEED_MAX = 1.00f;

/// @brief 数据发布频率 (Hz)
constexpr double PUBLISH_RATE_HZ = 30.0;

/// @brief ASCII 地图显示尺寸（字符列数 × 行数）
constexpr int ASCII_MAP_COLS = 80;
constexpr int ASCII_MAP_ROWS = 24;

/// @brief 地图自动显示间隔 (秒)，仅在有更新时打印
constexpr double MAP_DISPLAY_INTERVAL_S = 3.0;

/**
 * @brief 获取当前 ROS 时间戳
 * @return rclcpp::Time 当前时间
 */
inline rclcpp::Time now()
{
    return rclcpp::Clock().now();
}

/**
 * @brief Go2 SLAM 导航桥接节点
 *
 * 通过 go2_motion_bridge C API 获取 Go2 传感器数据和发送运动指令，
 * 将传感器数据桥接到 ROS2 导航栈（PointCloud2、LaserScan、Odometry、TF），
 * 订阅 /map 话题实时显示建图状态（ASCII 缩略图），
 * 并订阅 cmd_vel 驱动 Go2 运动，同时提供 Nav2 NavigateToPose 导航目标接口。
 */
class Go2SlamNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @param netInterface 网络接口名称
     */
    explicit Go2SlamNode(const std::string& netInterface)
        : Node("go2_slam_node")
    {
        (void)netInterface; // 运动桥接在主线程中已初始化

        // ---- ROS2 发布器 ----
        cloudPub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/utlidar/cloud", 10);
        scanPub_   = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        odomPub_   = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // ---- ROS2 订阅器 ----
        cmdVelSub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&Go2SlamNode::cmdVelCallback, this, std::placeholders::_1));

        mapSub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local(),
            std::bind(&Go2SlamNode::mapCallback, this, std::placeholders::_1));

        // ---- TF 广播器 ----
        tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // ---- Nav2 动作客户端 ----
        navActionClient_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");

        // ---- 定时发布传感器数据 ----
        publishTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / PUBLISH_RATE_HZ)),
            std::bind(&Go2SlamNode::publishData, this));

        // ---- 定时显示地图状态 ----
        displayTimer_ = this->create_wall_timer(
            std::chrono::duration<double>(MAP_DISPLAY_INTERVAL_S),
            std::bind(&Go2SlamNode::autoDisplayMap, this));

        RCLCPP_INFO(this->get_logger(), "Go2 SLAM 导航节点已就绪");
    }

    /**
     * @brief 发送导航目标到 Nav2
     * @param x 目标 x 坐标 (m)
     * @param y 目标 y 坐标 (m)
     * @param yaw 目标偏航角 (rad)
     */
    void navigateToPose(float x, float y, float yaw)
    {
        if (!navActionClient_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 动作服务器未就绪");
            return;
        }

        auto goalMsg = nav2_msgs::action::NavigateToPose::Goal();
        goalMsg.pose.header.frame_id = "map";
        goalMsg.pose.header.stamp = now();
        goalMsg.pose.pose.position.x = static_cast<double>(x);
        goalMsg.pose.pose.position.y = static_cast<double>(y);
        goalMsg.pose.pose.position.z = 0.0;
        goalMsg.pose.pose.orientation.z = std::sin(yaw / 2.0);
        goalMsg.pose.pose.orientation.w = std::cos(yaw / 2.0);

        RCLCPP_INFO(this->get_logger(),
            "发送导航目标: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);

        auto sendGoalOptions = rclcpp_action::Client<
            nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        sendGoalOptions.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<
                   nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "导航目标已到达");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "导航目标被中止");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_INFO(this->get_logger(), "导航目标已取消");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "导航目标结果未知");
                    break;
                }
            };

        navActionClient_->async_send_goal(goalMsg, sendGoalOptions);
    }

    /**
     * @brief 手动显示当前地图（由用户 'v' 命令触发）
     */
    void printMap()
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        if (!latestMap_) {
            std::cout << "[地图] 尚未收到地图数据，请确保 slam_toolbox 已启动并按下 m 开始建图"
                      << std::endl;
            return;
        }
        printMapStatus();
        flushLastDisplay_ = mapUpdateCount_;
    }

private:
    /**
     * @brief /map 话题回调，存储最新地图数据
     * @param msg 占据栅格地图消息
     */
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        latestMap_ = msg;
        mapUpdateCount_++;
    }

    /**
     * @brief 定时自动显示地图（仅在数据更新时打印）
     */
    void autoDisplayMap()
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        if (!latestMap_) return;
        if (mapUpdateCount_ == lastAutoDisplayCount_) return;
        lastAutoDisplayCount_ = mapUpdateCount_;
        printMapStatus();
        flushLastDisplay_ = mapUpdateCount_;
    }

    /**
     * @brief 打印地图状态信息与 ASCII 缩略图
     */
    void printMapStatus() const
    {
        const auto& map = *latestMap_;
        int width  = static_cast<int>(map.info.width);
        int height = static_cast<int>(map.info.height);

        // 计算覆盖面积（已探索区域，值 ≥ 0）
        size_t explored = 0;
        for (auto v : map.data) {
            if (v >= 0) explored++;
        }
        double coverage = (map.data.empty())
            ? 0.0 : (100.0 * static_cast<double>(explored) / static_cast<double>(map.data.size()));

        std::ostringstream oss;
        oss << "\n+" << std::string(ASCII_MAP_COLS, '-') << "+\n";
        oss << "| 地图状态  #" << std::setw(5) << mapUpdateCount_
            << "  |  尺寸: " << std::setw(5) << width << " x " << std::setw(5) << height
            << "  |  分辨率: " << std::fixed << std::setprecision(3) << map.info.resolution
            << " m/px  |  覆盖: "
            << std::fixed << std::setprecision(1) << coverage << "%"
            << std::string(ASCII_MAP_COLS > 80 ? ASCII_MAP_COLS - 80 : 0, ' ')
            << "|\n";
        oss << "+" << std::string(ASCII_MAP_COLS, '-') << "+\n";

        // ASCII 缩略图：降采样
        int stepX = std::max(1, width  / ASCII_MAP_COLS);
        int stepY = std::max(1, height / ASCII_MAP_ROWS);

        for (int row = 0; row < ASCII_MAP_ROWS; ++row) {
            oss << "|";
            int yStart = height - 1 - row * stepY;
            for (int col = 0; col < ASCII_MAP_COLS; ++col) {
                int xStart = col * stepX;
                // 采样区域内的多数值
                int occupied = 0, freeCount = 0, unknown = 0;
                int sampleCount = 0;
                for (int dy = 0; dy < stepY && (yStart - dy) >= 0; ++dy) {
                    for (int dx = 0; dx < stepX && (xStart + dx) < width; ++dx) {
                        int idx = (yStart - dy) * width + (xStart + dx);
                        int8_t val = map.data[static_cast<size_t>(idx)];
                        if (val < 0)       unknown++;
                        else if (val > 65) occupied++;
                        else               freeCount++;
                        sampleCount++;
                    }
                }
                char ch;
                if (sampleCount == 0) {
                    ch = ' ';
                } else if (unknown > sampleCount / 2) {
                    ch = ' ';
                } else if (occupied > sampleCount / 2) {
                    ch = '#';
                } else if (occupied > 0) {
                    ch = '+';
                } else {
                    ch = '.';
                }
                oss << ch;
            }
            oss << "|\n";
        }

        oss << "+" << std::string(ASCII_MAP_COLS, '-') << "+\n";
        oss << "  图例:  . =空闲  + =部分占据  # =障碍物  空格=未知\n";
        std::cout << oss.str() << std::flush;
    }

    /**
     * @brief 定时发布传感器数据（Odometry、TF、PointCloud2、LaserScan）
     */
    void publishData()
    {
        auto stamp = now();

        // ---- 获取并发布里程计 + TF ----
        float odomX = 0, odomY = 0, odomYaw = 0, odomVx = 0, odomVyaw = 0;
        if (go2_motion_get_odom(&odomX, &odomY, &odomYaw, &odomVx, &odomVyaw)) {
            auto odomMsg = std::make_unique<nav_msgs::msg::Odometry>();
            odomMsg->header.stamp = stamp;
            odomMsg->header.frame_id = "odom";
            odomMsg->child_frame_id = "base_link";
            odomMsg->pose.pose.position.x    = static_cast<double>(odomX);
            odomMsg->pose.pose.position.y    = static_cast<double>(odomY);
            odomMsg->pose.pose.position.z    = 0.0;
            odomMsg->pose.pose.orientation.z = std::sin(odomYaw / 2.0);
            odomMsg->pose.pose.orientation.w = std::cos(odomYaw / 2.0);
            odomMsg->twist.twist.linear.x    = static_cast<double>(odomVx);
            odomMsg->twist.twist.angular.z   = static_cast<double>(odomVyaw);
            odomPub_->publish(std::move(odomMsg));

            // 广播 TF (odom → base_link)
            auto tfMsg = geometry_msgs::msg::TransformStamped();
            tfMsg.header.stamp = stamp;
            tfMsg.header.frame_id = "odom";
            tfMsg.child_frame_id = "base_link";
            tfMsg.transform.translation.x = static_cast<double>(odomX);
            tfMsg.transform.translation.y = static_cast<double>(odomY);
            tfMsg.transform.translation.z = 0.0;
            tfMsg.transform.rotation.z = std::sin(odomYaw / 2.0);
            tfMsg.transform.rotation.w = std::cos(odomYaw / 2.0);
            tfBroadcaster_->sendTransform(tfMsg);

            // 广播 TF (base_link → utlidar_lidar) 将雷达帧挂载到机器人上
            auto lidarTf = geometry_msgs::msg::TransformStamped();
            lidarTf.header.stamp = stamp;
            lidarTf.header.frame_id = "base_link";
            lidarTf.child_frame_id = "utlidar_lidar";
            lidarTf.transform.translation.x = 0.0;
            lidarTf.transform.translation.y = 0.0;
            lidarTf.transform.translation.z = 0.4;
            lidarTf.transform.rotation.x = 0.0;
            lidarTf.transform.rotation.y = 0.0;
            lidarTf.transform.rotation.z = 0.0;
            lidarTf.transform.rotation.w = 1.0;
            tfBroadcaster_->sendTransform(lidarTf);
        }

        // ---- 获取并发布点云 ----
        constexpr uint32_t MAX_POINTS = 50000;
        static float pointBuf[MAX_POINTS * 4]; // x,y,z,intensity
        uint32_t pointCount = 0, width = 0, height = 0;

        if (go2_motion_get_lidar(pointBuf, MAX_POINTS, &pointCount, &width, &height)) {
            // 发布 ROS2 PointCloud2
            auto rosCloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
            rosCloud->header.stamp = stamp;
            rosCloud->header.frame_id = "utlidar_lidar";
            rosCloud->height       = height;
            rosCloud->width        = width;
            rosCloud->is_bigendian = false;
            rosCloud->point_step   = 16;
            rosCloud->row_step     = 16 * width;
            rosCloud->is_dense     = true;

            sensor_msgs::msg::PointField fieldX, fieldY, fieldZ, fieldI;
            fieldX.name = "x";         fieldX.offset = 0;  fieldX.datatype = 7; fieldX.count = 1;
            fieldY.name = "y";         fieldY.offset = 4;  fieldY.datatype = 7; fieldY.count = 1;
            fieldZ.name = "z";         fieldZ.offset = 8;  fieldZ.datatype = 7; fieldZ.count = 1;
            fieldI.name = "intensity"; fieldI.offset = 12; fieldI.datatype = 7; fieldI.count = 1;
            rosCloud->fields = {fieldX, fieldY, fieldZ, fieldI};

            uint8_t* rawPtr = reinterpret_cast<uint8_t*>(pointBuf);
            rosCloud->data.assign(rawPtr, rawPtr + pointCount * 16);
            cloudPub_->publish(std::move(rosCloud));

            // 转换为 LaserScan
            auto scanMsg = std::make_unique<sensor_msgs::msg::LaserScan>();
            scanMsg->header.stamp    = stamp;
            scanMsg->header.frame_id = "utlidar_lidar";
            scanMsg->angle_min       = LIDAR_ANGLE_MIN;
            scanMsg->angle_max       = LIDAR_ANGLE_MAX;
            scanMsg->angle_increment = (LIDAR_ANGLE_MAX - LIDAR_ANGLE_MIN) / LASER_SCAN_SAMPLES;
            scanMsg->time_increment  = 0.0;
            scanMsg->scan_time       = 0.1;
            scanMsg->range_min       = LIDAR_RANGE_MIN;
            scanMsg->range_max       = LIDAR_RANGE_MAX;
            scanMsg->ranges.assign(LASER_SCAN_SAMPLES, std::numeric_limits<float>::infinity());

            for (uint32_t i = 0; i < pointCount; ++i) {
                float px = pointBuf[i * 4 + 0];
                float py = pointBuf[i * 4 + 1];
                float pz = pointBuf[i * 4 + 2];

                if (std::fabs(pz) > LIDAR_Z_THRESHOLD) continue;

                float range = std::sqrt(px * px + py * py);
                if (range < LIDAR_RANGE_MIN || range > LIDAR_RANGE_MAX) continue;

                float angle = std::atan2(py, px);
                int idx = static_cast<int>(
                    (angle - LIDAR_ANGLE_MIN) / scanMsg->angle_increment + 0.5);
                if (idx < 0 || idx >= LASER_SCAN_SAMPLES) continue;

                if (range < scanMsg->ranges[static_cast<size_t>(idx)]) {
                    scanMsg->ranges[static_cast<size_t>(idx)] = range;
                }
            }
            scanPub_->publish(std::move(scanMsg));
        }
    }

    /**
     * @brief ROS2 cmd_vel 回调
     * @param msg Twist 速度指令
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float vx   = std::clamp(static_cast<float>(msg->linear.x),
                               -NAV_FORWARD_SPEED_MAX, NAV_FORWARD_SPEED_MAX);
        float vyaw = std::clamp(static_cast<float>(msg->angular.z),
                               -NAV_ANGULAR_SPEED_MAX, NAV_ANGULAR_SPEED_MAX);
        go2_motion_move(vx, 0.0f, vyaw);
        lastCmdTime_ = now();
    }

    // ---- ROS2 接口 ----
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr   scanPub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       odomPub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr  cmdVelSub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub_;
    rclcpp::TimerBase::SharedPtr                                publishTimer_;
    rclcpp::TimerBase::SharedPtr                                displayTimer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>              tfBroadcaster_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navActionClient_;

    rclcpp::Time lastCmdTime_{0, 0, RCL_SYSTEM_TIME};

    // ---- 地图显示状态 ----
    mutable std::mutex mapMutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latestMap_;
    int mapUpdateCount_ = 0;
    int lastAutoDisplayCount_ = 0;
    int flushLastDisplay_ = 0;
};

// ====================================================================
// 命令行输入线程与命令队列
// ====================================================================

static std::atomic<bool> g_running{true};
static std::mutex g_cmdMutex;
static std::queue<std::string> g_cmdQueue;

/**
 * @brief 输入线程函数，持续从标准输入读取命令行指令
 * @param node SLAM 节点共享指针
 */
static void inputThreadFunc()
{
    std::string line;
    while (g_running && std::getline(std::cin, line)) {
        if (line.empty()) continue;
        {
            std::lock_guard<std::mutex> lock(g_cmdMutex);
            g_cmdQueue.push(line);
        }
    }
}

/**
 * @brief 打印命令提示
 */
static void printHelp()
{
    std::cout << "\n+------------------------------------------------+\n";
    std::cout <<   "|  Go2 SLAM + Nav2 导航程序                       |\n";
    std::cout <<   "+------------------------------------------------+\n";
    std::cout <<   "|  命令 (输入后按回车):                           |\n";
    std::cout <<   "|    m     - 开始建图 (SLAM)                      |\n";
    std::cout <<   "|    v     - 显示当前地图 (ASCII 缩略图)           |\n";
    std::cout <<   "|    s     - 保存地图                             |\n";
    std::cout <<   "|    g / 1 - 导航目标 (x=2, y=0) 向前 2m          |\n";
    std::cout <<   "|    2     - 导航目标 (x=0, y=2) 向左 2m          |\n";
    std::cout <<   "|    3     - 导航目标 (x=2, y=2) 右前方            |\n";
    std::cout <<   "|    q     - 退出程序                             |\n";
    std::cout <<   "+------------------------------------------------+\n";
    std::cout <<   "|  需在其他终端启动:                              |\n";
    std::cout <<   "|    ros2 launch slam_toolbox online_async...     |\n";
    std::cout <<   "|    Nav2 导航栈相关节点                          |\n";
    std::cout <<   "+------------------------------------------------+\n";
    std::cout << "\n> " << std::flush;
}

/**
 * @brief 处理来自命令队列的指令
 * @param node SLAM 节点共享指针
 */
static void processCommands(std::shared_ptr<Go2SlamNode> node)
{
    std::lock_guard<std::mutex> lock(g_cmdMutex);
    while (!g_cmdQueue.empty()) {
        std::string cmd = g_cmdQueue.front();
        g_cmdQueue.pop();

        char key = cmd[0];

        switch (key) {
        case 'q':
        case 'Q':
            std::cout << "\n程序退出中..." << std::endl;
            g_running = false;
            rclcpp::shutdown();
            return;

        case 'm':
        case 'M':
            std::cout << ">>> 建图模式: 请遥控机器人移动以构建地图\n"
                      << "    地图将每 " << MAP_DISPLAY_INTERVAL_S
                      << " 秒自动更新显示，或按 v 手动查看\n> " << std::flush;
            break;

        case 'v':
        case 'V':
            node->printMap();
            std::cout << "\n> " << std::flush;
            break;

        case 's':
        case 'S':
            std::cout << ">>> 提示: 请在另一个终端手动保存地图:\n"
                      << "    ros2 service call /slam_toolbox/save_map "
                         "slam_toolbox/srv/SaveMap "
                         "\"{name: {data: '/tmp/map'}}\"\n> " << std::flush;
            break;

        case 'g':
        case 'G':
        case '1':
            node->navigateToPose(2.0f, 0.0f, 0.0f);
            std::cout << "> " << std::flush;
            break;

        case '2':
            node->navigateToPose(0.0f, 2.0f, M_PI_2);
            std::cout << "> " << std::flush;
            break;

        case '3':
            node->navigateToPose(2.0f, 2.0f, M_PI_4);
            std::cout << "> " << std::flush;
            break;

        default:
            std::cout << "未知命令: '" << cmd << "'，输入 h 查看帮助\n> " << std::flush;
            break;
        }
    }
}

/**
 * @brief Go2 SLAM 导航程序入口
 * @param argc 参数个数
 * @param argv 参数列表，argv[1] 为网络接口名称
 * @return int 0 正常退出，-1 异常退出
 */
int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "用法: " << argv[0] << " <network_interface>" << std::endl;
        std::cout << "示例: " << argv[0] << " eth0" << std::endl;
        return -1;
    }
    std::string netInterface = argv[1];

    // ---- 初始化 Go2 运动桥接 ----
    std::cout << "正在初始化 Go2 运动桥接 (接口: " << netInterface << ")..." << std::endl;
    if (!go2_motion_init(netInterface.c_str())) {
        std::cerr << "运动桥接初始化失败" << std::endl;
        return -1;
    }
    std::cout << "Go2 运动桥接已就绪" << std::endl;

    // ---- 初始化 ROS2 ----
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Go2SlamNode>(netInterface);

    // ---- 机器人站立 ----
    go2_motion_stand_up();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ---- 打印控制说明并启动输入线程 ----
    printHelp();

    std::thread inputThread(inputThreadFunc);
    inputThread.detach();

    // ---- 主循环: ROS2 spin + 处理命令行输入 ----
    while (rclcpp::ok() && g_running) {
        rclcpp::spin_some(node);
        processCommands(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    go2_motion_stop();
    go2_motion_stand_down();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
