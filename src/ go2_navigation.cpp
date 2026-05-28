/**
 * @file go2_navigation.cpp
 * @brief Go2 自主导航程序，读取 route.yaml 路径点并通过 Nav2 动作接口进行导航
 *
 * @par 使用说明
 *       go2_navigation <network_interface> [route_file]
 *       示例: ./go2_navigation eth0
 *             ./go2_navigation eth0 custom_route.yaml
 *
 *       说明: 本程序桥接 Go2 传感器数据到 ROS2 导航栈，启动后自动读取
 *             route.yaml 中的路径点并通过 Nav2 动作服务器发送导航目标。
 *             优先使用 FollowWaypoints 一次性发送全部路径点，
 *             不可用时回退到 NavigateToPose 逐个发送。
 *
 *       配合: 需在其他终端启动 Nav2 导航栈 (bt_navigator / planner_server /
 *             controller_server)、AMCL 定位与地图服务器。
 *
 *       控制:
 *             r     - 重新发送所有路径点
 *             s     - 停止导航
 *             q     - 退出程序
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>

#include "go2_motion_bridge.hpp"

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <queue>
#include <vector>
#include <limits>
#include <fstream>

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

/**
 * @brief 获取当前 ROS 时间戳
 * @return rclcpp::Time 当前时间
 */
inline rclcpp::Time now()
{
    return rclcpp::Clock().now();
}

/**
 * @brief YAML 路径点数据结构
 */
struct Waypoint
{
    std::string frame;
    double x;
    double y;
    double yaw;
};

/**
 * @brief 解析 route.yaml 文件，提取路径点列表
 * @param filepath YAML 文件路径
 * @return std::vector<Waypoint> 解析后的路径点列表
 */
static std::vector<Waypoint> parseRouteYaml(const std::string& filepath)
{
    std::vector<Waypoint> waypoints;

    // 检查文件是否存在
    std::ifstream checkFile(filepath);
    if (!checkFile.good()) {
        std::cerr << "[错误] 文件不存在: " << filepath << std::endl;
        return waypoints;
    }
    checkFile.close();

    try {
        YAML::Node config = YAML::LoadFile(filepath);
        YAML::Node wpNode = config["waypoints"];
        if (!wpNode || !wpNode.IsSequence()) {
            std::cerr << "[错误] 未找到 'waypoints' 字段或格式不正确" << std::endl;
            return waypoints;
        }

        for (const auto& wp : wpNode) {
            Waypoint w;
            w.frame = wp["frame"] ? wp["frame"].as<std::string>() : "map";
            w.x     = wp["x"].as<double>();
            w.y     = wp["y"].as<double>();
            w.yaw   = wp["yaw"].as<double>();
            waypoints.push_back(w);
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "[错误] YAML 解析失败: " << e.what() << std::endl;
    }

    return waypoints;
}

/**
 * @brief Go2 导航节点
 *
 * 桥接 Go2 传感器数据到 ROS2，通过 Nav2 动作接口发送路径点进行自主导航。
 */
class Go2NavigationNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @param netInterface 网络接口名称（预留，运动桥接在主线程中初始化）
     * @param routeFile route.yaml 文件路径
     */
    explicit Go2NavigationNode(const std::string& netInterface, const std::string& routeFile)
        : Node("go2_navigation_node"), routeFile_(routeFile)
    {
        (void)netInterface;

        // ---- ROS2 发布器 ----
        cloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/utlidar/cloud", 10);
        scanPub_  = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        odomPub_  = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // ---- ROS2 订阅器 ----
        cmdVelSub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&Go2NavigationNode::cmdVelCallback, this, std::placeholders::_1));

        // ---- TF 广播器 ----
        tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // ---- Nav2 动作客户端 ----
        navToPoseClient_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");
        followWpClient_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
            this, "follow_waypoints");

        // ---- 定时发布传感器数据 ----
        publishTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / PUBLISH_RATE_HZ)),
            std::bind(&Go2NavigationNode::publishSensorData, this));

        RCLCPP_INFO(this->get_logger(), "Go2 导航节点已就绪");
    }

    /**
     * @brief 启动导航：解析 route.yaml 并发送路径点
     */
    void startNavigation()
    {
        waypoints_ = parseRouteYaml(routeFile_);
        if (waypoints_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "路径点为空，文件: %s", routeFile_.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "已读取 %zu 个路径点", waypoints_.size());
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            RCLCPP_INFO(this->get_logger(),
                "  [%zu] x=%.3f y=%.3f yaw=%.3f frame=%s",
                i + 1, waypoints_[i].x, waypoints_[i].y,
                waypoints_[i].yaw, waypoints_[i].frame.c_str());
        }

        // 优先使用 FollowWaypoints，不可用时回退到 NavigateToPose
        if (followWpClient_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_INFO(this->get_logger(), "使用 FollowWaypoints 发送全部路径点");
            sendFollowWaypoints();
        } else {
            RCLCPP_INFO(this->get_logger(), "FollowWaypoints 不可用，使用 NavigateToPose 逐个发送");
            currentWpIndex_ = 0;
            sendNextWaypoint();
        }
    }

    /**
     * @brief 停止导航，取消当前动作并停止机器人
     */
    void stopNavigation()
    {
        allDone_ = false;
        cancelAllGoals();
        go2_motion_stop();
        RCLCPP_INFO(this->get_logger(), "导航已停止");
    }

private:
    // ---- YAML 路径点 ----
    std::string routeFile_;
    std::vector<Waypoint> waypoints_;
    size_t currentWpIndex_ = 0;
    std::atomic<bool> allDone_{false};

    // ---- NavigateToPose 动作客户端 ----
    /**
     * @brief 使用 NavigateToPose 逐个发送路径点
     */
    void sendNextWaypoint()
    {
        if (currentWpIndex_ >= waypoints_.size()) {
            allDone_ = true;
            RCLCPP_INFO(this->get_logger(), "所有 %zu 个路径点已完成!", waypoints_.size());
            return;
        }

        if (!navToPoseClient_->wait_for_action_server(std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(), "NavigateToPose 动作服务器未就绪");
            return;
        }

        const auto& wp = waypoints_[currentWpIndex_];
        auto goalMsg = nav2_msgs::action::NavigateToPose::Goal();
        goalMsg.pose.header.frame_id = wp.frame;
        goalMsg.pose.header.stamp = now();
        goalMsg.pose.pose.position.x = wp.x;
        goalMsg.pose.pose.position.y = wp.y;
        goalMsg.pose.pose.position.z = 0.0;
        goalMsg.pose.pose.orientation.z = std::sin(wp.yaw / 2.0);
        goalMsg.pose.pose.orientation.w = std::cos(wp.yaw / 2.0);

        RCLCPP_INFO(this->get_logger(),
            "发送路径点 [%zu/%zu]: x=%.2f y=%.2f yaw=%.2f",
            currentWpIndex_ + 1, waypoints_.size(), wp.x, wp.y, wp.yaw);

        auto sendGoalOptions = rclcpp_action::Client<
            nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        sendGoalOptions.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<
                   nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(),
                        "路径点 [%zu/%zu] 已到达",
                        currentWpIndex_ + 1, waypoints_.size());
                    currentWpIndex_++;
                    sendNextWaypoint();
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(),
                        "路径点 [%zu/%zu] 被中止",
                        currentWpIndex_ + 1, waypoints_.size());
                    allDone_ = true;
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_INFO(this->get_logger(),
                        "路径点 [%zu/%zu] 已取消",
                        currentWpIndex_ + 1, waypoints_.size());
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "路径点结果未知");
                    allDone_ = true;
                    break;
                }
            };

        auto goalHandleFuture = navToPoseClient_->async_send_goal(goalMsg, sendGoalOptions);
        {
            std::lock_guard<std::mutex> lock(goalHandleMutex_);
            navGoalHandle_ = goalHandleFuture.get();
        }
    }

    // ---- FollowWaypoints 动作客户端 ----
    /**
     * @brief 使用 FollowWaypoints 一次性发送全部路径点
     */
    void sendFollowWaypoints()
    {
        auto goalMsg = nav2_msgs::action::FollowWaypoints::Goal();
        for (const auto& wp : waypoints_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = wp.frame;
            pose.header.stamp = now();
            pose.pose.position.x = wp.x;
            pose.pose.position.y = wp.y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.z = std::sin(wp.yaw / 2.0);
            pose.pose.orientation.w = std::cos(wp.yaw / 2.0);
            goalMsg.poses.push_back(pose);
        }

        RCLCPP_INFO(this->get_logger(),
            "发送 FollowWaypoints 目标: %zu 个路径点", waypoints_.size());

        auto sendGoalOptions = rclcpp_action::Client<
            nav2_msgs::action::FollowWaypoints>::SendGoalOptions();

        sendGoalOptions.feedback_callback =
            [this](rclcpp_action::ClientGoalHandle<
                   nav2_msgs::action::FollowWaypoints>::SharedPtr,
                   const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(),
                    "FollowWaypoints: 正在前往路径点 %u/%zu",
                    feedback->current_waypoint + 1, waypoints_.size());
            };

        sendGoalOptions.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<
                   nav2_msgs::action::FollowWaypoints>::WrappedResult& result) {
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "全部路径点已完成!");
                    if (!result.result->missed_waypoints.empty()) {
                        RCLCPP_WARN(this->get_logger(),
                            "未到达的路径点: %zu 个",
                            result.result->missed_waypoints.size());
                        for (auto idx : result.result->missed_waypoints) {
                            RCLCPP_WARN(this->get_logger(), "  - 路径点索引: %d", idx);
                        }
                    }
                    allDone_ = true;
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "FollowWaypoints 被中止");
                    allDone_ = true;
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_INFO(this->get_logger(), "FollowWaypoints 已取消");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "FollowWaypoints 结果未知");
                    allDone_ = true;
                    break;
                }
            };

        auto goalHandleFuture = followWpClient_->async_send_goal(goalMsg, sendGoalOptions);
        {
            std::lock_guard<std::mutex> lock(goalHandleMutex_);
            followGoalHandle_ = goalHandleFuture.get();
        }
    }

    /**
     * @brief 取消所有进行中的导航动作
     */
    void cancelAllGoals()
    {
        std::lock_guard<std::mutex> lock(goalHandleMutex_);
        if (navGoalHandle_) {
            navToPoseClient_->async_cancel_goal(navGoalHandle_);
            navGoalHandle_.reset();
        }
        if (followGoalHandle_) {
            followWpClient_->async_cancel_goal(followGoalHandle_);
            followGoalHandle_.reset();
        }
    }

    // ---- 传感器数据发布 ----
    /**
     * @brief 定时发布传感器数据（Odometry、TF、PointCloud2、LaserScan）
     */
    void publishSensorData()
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

            // 广播 TF (base_link → utlidar_lidar)
            auto lidarTf = geometry_msgs::msg::TransformStamped();
            lidarTf.header.stamp = stamp;
            lidarTf.header.frame_id = "base_link";
            lidarTf.child_frame_id = "utlidar_lidar";
            lidarTf.transform.translation.z = 0.4;
            lidarTf.transform.rotation.w = 1.0;
            tfBroadcaster_->sendTransform(lidarTf);
        }

        // ---- 获取并发布点云 / LaserScan ----
        constexpr uint32_t MAX_POINTS = 50000;
        static float pointBuf[MAX_POINTS * 4];
        uint32_t pointCount = 0, width = 0, height = 0;

        if (go2_motion_get_lidar(pointBuf, MAX_POINTS, &pointCount, &width, &height)) {
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
            fieldX.name = "x"; fieldX.offset = 0;  fieldX.datatype = 7; fieldX.count = 1;
            fieldY.name = "y"; fieldY.offset = 4;  fieldY.datatype = 7; fieldY.count = 1;
            fieldZ.name = "z"; fieldZ.offset = 8;  fieldZ.datatype = 7; fieldZ.count = 1;
            fieldI.name = "intensity"; fieldI.offset = 12; fieldI.datatype = 7; fieldI.count = 1;
            rosCloud->fields = {fieldX, fieldY, fieldZ, fieldI};

            uint8_t* rawPtr = reinterpret_cast<uint8_t*>(pointBuf);
            rosCloud->data.assign(rawPtr, rawPtr + pointCount * 16);
            cloudPub_->publish(std::move(rosCloud));

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
            scanMsg->ranges.assign(LASER_SCAN_SAMPLES,
                std::numeric_limits<float>::infinity());

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
     * @brief ROS2 cmd_vel 回调，转发速度指令到 Go2 运动桥接
     * @param msg Twist 速度指令
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float vx   = std::clamp(static_cast<float>(msg->linear.x),
                               -NAV_FORWARD_SPEED_MAX, NAV_FORWARD_SPEED_MAX);
        float vyaw = std::clamp(static_cast<float>(msg->angular.z),
                               -NAV_ANGULAR_SPEED_MAX, NAV_ANGULAR_SPEED_MAX);
        go2_motion_move(vx, 0.0f, vyaw);
    }

    // ---- ROS2 接口 ----
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr   scanPub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       odomPub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr  cmdVelSub_;
    rclcpp::TimerBase::SharedPtr                                publishTimer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>              tfBroadcaster_;

    // ---- Nav2 动作客户端 ----
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navToPoseClient_;
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr followWpClient_;

    // ---- 动作目标句柄（用于取消） ----
    mutable std::mutex goalHandleMutex_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navGoalHandle_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr followGoalHandle_;
};

// ====================================================================
// 命令行输入线程与命令队列
// ====================================================================

static std::atomic<bool> g_running{true};
static std::mutex g_cmdMutex;
static std::queue<std::string> g_cmdQueue;

/**
 * @brief 输入线程函数，持续从标准输入读取命令行指令
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
    std::cout << "\n+----------------------------------------------------+\n";
    std::cout <<   "|  Go2 自主导航程序 (Nav2 动作接口)                  |\n";
    std::cout <<   "+----------------------------------------------------+\n";
    std::cout <<   "|  启动后自动读取 route.yaml 并发送路径点              |\n";
    std::cout <<   "|  优先使用 FollowWaypoints，回退 NavigateToPose       |\n";
    std::cout <<   "+----------------------------------------------------+\n";
    std::cout <<   "|  命令 (输入后按回车):                               |\n";
    std::cout <<   "|    r     - 重新发送所有路径点                       |\n";
    std::cout <<   "|    s     - 停止导航                                |\n";
    std::cout <<   "|    q     - 退出程序                                |\n";
    std::cout <<   "+----------------------------------------------------+\n";
    std::cout <<   "|  配合: 需在其他终端启动 Nav2 导航栈 + AMCL + 地图    |\n";
    std::cout <<   "+----------------------------------------------------+\n";
    std::cout << "\n> " << std::flush;
}

/**
 * @brief 处理来自命令队列的指令
 * @param node 导航节点共享指针
 */
static void processCommands(std::shared_ptr<Go2NavigationNode> node)
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

        case 'r':
        case 'R':
            std::cout << ">>> 重新发送路径点\n> " << std::flush;
            node->stopNavigation();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            node->startNavigation();
            break;

        case 's':
        case 'S':
            node->stopNavigation();
            std::cout << "> " << std::flush;
            break;

        default:
            std::cout << "未知命令: '" << cmd << "'\n> " << std::flush;
            break;
        }
    }
}

/**
 * @brief Go2 自主导航程序入口
 * @param argc 参数个数
 * @param argv 参数列表，argv[1] 为网络接口，argv[2] 为可选 route.yaml 路径
 * @return int 0 正常退出，-1 异常退出
 */
int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "用法: " << argv[0] << " <network_interface> [route_file]" << std::endl;
        std::cout << "示例: " << argv[0] << " eth0" << std::endl;
        std::cout << "      " << argv[0] << " eth0 ../src/route.yaml" << std::endl;
        return -1;
    }
    std::string netInterface = argv[1];
    std::string routeFile = (argc >= 3) ? argv[2] : "src/route.yaml";

    // ---- 初始化 Go2 运动桥接 ----
    std::cout << "正在初始化 Go2 运动桥接 (接口: " << netInterface << ")..." << std::endl;
    if (!go2_motion_init(netInterface.c_str())) {
        std::cerr << "运动桥接初始化失败" << std::endl;
        return -1;
    }
    std::cout << "Go2 运动桥接已就绪" << std::endl;

    // ---- 初始化 ROS2 ----
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Go2NavigationNode>(netInterface, routeFile);

    // ---- 机器人站立 ----
    go2_motion_stand_up();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ---- 打印帮助并启动输入线程 ----
    printHelp();

    std::thread inputThread(inputThreadFunc);
    inputThread.detach();

    // ---- 自动开始导航 ----
    std::cout << ">>> 正在启动自动导航..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    node->startNavigation();
    std::cout << "> " << std::flush;

    // ---- 主循环 ----
    while (rclcpp::ok() && g_running) {
        rclcpp::spin_some(node);
        processCommands(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    node->stopNavigation();
    go2_motion_stop();
    go2_motion_stand_down();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
