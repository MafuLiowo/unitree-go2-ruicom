/**
 * @file go2_navigation.cpp
 * @brief Go2 自主导航程序，集成 AMCL 定位、NavFn 全局规划、DWB 局部规划与 Go2 运动控制
 *
 * @par 使用说明
 *       需配合 AMCL 和 map_server 节点使用：
 *       1. ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/path/to/map.yaml
 *       2. ros2 run nav2_amcl amcl
 *       3. ./go2_navigation <network_interface>
 *
 * @par 控制命令（终端输入）
 *       g / 1  - 发送导航目标 (x=2, y=0, yaw=0)  向前 2 米
 *       2      - 发送导航目标 (x=0, y=2, yaw=π/2) 向左 2 米
 *       3      - 发送导航目标 (x=2, y=2, yaw=π/4) 右前方
 *       p      - 打印当前位姿信息
 *       s      - 停止导航
 *       q      - 退出程序
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/string.hpp>

#include "go2_motion_bridge.hpp"
#include "NavFnPlanner.hpp"
#include "DWBPlanner.hpp"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <queue>

/// @brief 最大线速度 (m/s)
constexpr float CMD_VX_MAX = 0.30f;
/// @brief 最大角速度 (rad/s)
constexpr float CMD_VYAW_MAX = 1.00f;

/// @brief 传感器发布频率 (Hz)
constexpr double PUBLISH_RATE_HZ = 30.0;
/// @brief 规划控制频率 (Hz)
constexpr double CONTROL_RATE_HZ = 20.0;

/// @brief 雷达扫描参数
constexpr double LIDAR_ANGLE_MIN  = -M_PI;
constexpr double LIDAR_ANGLE_MAX  =  M_PI;
constexpr int    LASER_SCAN_SAMPLES = 720;
constexpr double LIDAR_RANGE_MIN  = 0.05;
constexpr double LIDAR_RANGE_MAX  = 20.0;
constexpr double LIDAR_Z_THRESHOLD = 0.3;

/// @brief 路径重规划距离阈值 (m)
constexpr float REPLAN_DIST_THRESHOLD = 0.5f;

/// @brief 机器人足迹多边形顶点（相对于 baselink 中心，逆时针），
///        修改此数组可调整碰撞检测形状
static constexpr FootprintPoint ROBOT_FOOTPRINT[] = {
    { 0.35f,  0.20f, 0.0f},  // 右前
    { 0.35f, -0.20f, 0.0f},  // 左前
    {-0.35f, -0.20f, 0.0f},  // 左后
    {-0.35f,  0.20f, 0.0f},  // 右后
};

inline rclcpp::Time now()
{
    return rclcpp::Clock().now();
}

/**
 * @brief Go2 自主导航节点
 *
 * 整合 AMCL 定位、NavFn 全局路径规划与 DWB 局部规划，
 * 通过 go2_motion_bridge C API 驱动 Go2 机器人运动。
 */
class Go2NavigationNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @param netInterface 网络接口名称
     */
    explicit Go2NavigationNode(const std::string& netInterface)
        : Node("go2_navigation_node")
    {
        (void)netInterface;

        // ---- 传感器发布器 ----
        cloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/utlidar/cloud", 10);
        scanPub_  = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        odomPub_  = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // ---- 订阅器 ----
        amclPoseSub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&Go2NavigationNode::amclPoseCallback, this, std::placeholders::_1));

        mapSub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local(),
            std::bind(&Go2NavigationNode::mapCallback, this, std::placeholders::_1));

        // ---- TF 广播器 ----
        tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // ---- 规划器初始化 ----
        navFnPlanner_.setParams(1.0f, 1000.0f, 100.0f);

        DWBConfig dwbConfig;
        dwbConfig.maxLinearVel  = CMD_VX_MAX;
        dwbConfig.minLinearVel  = 0.0f;
        dwbConfig.maxAngularVel = CMD_VYAW_MAX;
        dwbConfig.minAngularVel = -CMD_VYAW_MAX;
        dwbConfig.linearStep    = 0.05f;
        dwbConfig.angularStep   = 0.1f;
        dwbConfig.goalTolerance = 0.3f;
        dwbConfig.simTime       = 1.5f;

        // 硬编码机器人足迹（修改 ROBOT_FOOTPRINT 数组调整碰撞检测形状）
        dwbConfig.footprint.assign(std::begin(ROBOT_FOOTPRINT), std::end(ROBOT_FOOTPRINT));
        dwbPlanner_.setConfig(dwbConfig);

        // ---- cmd_vel 发布器 ----
        cmdVelPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // ---- 定时器 ----
        publishTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / PUBLISH_RATE_HZ)),
            std::bind(&Go2NavigationNode::publishSensorData, this));

        controlTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / CONTROL_RATE_HZ)),
            std::bind(&Go2NavigationNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Go2 导航节点已就绪");
        RCLCPP_INFO(this->get_logger(), "等待 AMCL 和地图数据...");
    }

    /**
     * @brief 设置导航目标并开始规划
     * @param x 目标 x 坐标 (m)
     * @param y 目标 y 坐标 (m)
     * @param yaw 目标偏航角 (rad)
     */
    void setGoal(float x, float y, float yaw)
    {
        goalPose_.x   = x;
        goalPose_.y   = y;
        goalPose_.yaw = yaw;

        if (!latestMap_) {
            RCLCPP_ERROR(this->get_logger(), "尚未收到地图数据，无法规划");
            return;
        }

        if (!planGlobalPath()) {
            RCLCPP_ERROR(this->get_logger(), "全局路径规划失败");
            navigating_ = false;
            return;
        }

        navigating_ = true;
        lastReplanTime_ = now();
        RCLCPP_INFO(this->get_logger(),
            "开始导航: (%.2f, %.2f) -> (%.2f, %.2f)",
            currentPose_.x, currentPose_.y, x, y);
    }

    /**
     * @brief 停止导航
     */
    void stopNavigation()
    {
        navigating_ = false;
        globalPath_.clear();
        go2_motion_stop();
        RCLCPP_INFO(this->get_logger(), "导航已停止");
    }

    /**
     * @brief 打印当前状态信息
     */
    void printStatus()
    {
        std::ostringstream oss;
        oss << "\n=== Go2 导航状态 ===" << std::endl;
        oss << "AMCL 位姿: (" << std::fixed << std::setprecision(3)
            << currentPose_.x << ", " << currentPose_.y << ", "
            << currentPose_.yaw * 180.0 / M_PI << "°)" << std::endl;
        oss << "目标点:   (" << goalPose_.x << ", " << goalPose_.y << ")" << std::endl;
        oss << "导航状态: " << (navigating_ ? "运行中" : "空闲") << std::endl;
        oss << "地图状态: " << (latestMap_ ? "已加载" : "未加载") << std::endl;
        if (latestMap_) {
            oss << "地图尺寸: " << latestMap_->info.width
                << " x " << latestMap_->info.height
                << " @ " << latestMap_->info.resolution << " m/px" << std::endl;
        }
        oss << "全局路径: " << globalPath_.size() << " 点" << std::endl;
        oss << "====================" << std::endl;
        std::cout << oss.str() << std::flush;
    }

private:
    /**
     * @brief AMCL 位姿回调
     * @param msg AMCL 发布的带协方差的位姿消息
     */
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(poseMutex_);
        amclPose_.x   = static_cast<float>(msg->pose.pose.position.x);
        amclPose_.y   = static_cast<float>(msg->pose.pose.position.y);
        float qz = static_cast<float>(msg->pose.pose.orientation.z);
        float qw = static_cast<float>(msg->pose.pose.orientation.w);
        amclPose_.yaw = std::atan2(2.0f * qz * qw, 1.0f - 2.0f * qz * qz);
        amclPoseValid_ = true;
    }

    /**
     * @brief 地图数据回调
     * @param msg 占据栅格地图消息
     */
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        bool mapChanged = !latestMap_
            || latestMap_->info.width  != msg->info.width
            || latestMap_->info.height != msg->info.height;

        latestMap_ = msg;

        if (mapChanged) {
            int w = static_cast<int>(msg->info.width);
            int h = static_cast<int>(msg->info.height);
            navFnPlanner_.setCostmap(
                msg->data.data(), w, h,
                static_cast<float>(msg->info.resolution),
                static_cast<float>(msg->info.origin.position.x),
                static_cast<float>(msg->info.origin.position.y));

            // 构建局部代价地图（用于 DWB 避障）
            localCostmap_.resize(static_cast<size_t>(w * h));
            std::copy(msg->data.begin(), msg->data.end(), localCostmap_.begin());
            localCostmapWidth_  = w;
            localCostmapHeight_ = h;
            localCostmapRes_    = static_cast<float>(msg->info.resolution);
            localCostmapOrigX_  = static_cast<float>(msg->info.origin.position.x);
            localCostmapOrigY_  = static_cast<float>(msg->info.origin.position.y);

            RCLCPP_INFO(this->get_logger(),
                "地图已加载: %d x %d @ %.3f m/px",
                w, h, msg->info.resolution);
        }
    }

    /**
     * @brief 全局路径规划
     * @return true 规划成功
     */
    bool planGlobalPath()
    {
        std::lock_guard<std::mutex> lockPose(poseMutex_);
        std::lock_guard<std::mutex> lockMap(mapMutex_);

        if (!latestMap_ || !amclPoseValid_) return false;

        currentPose_ = amclPose_;

        return navFnPlanner_.planPath(
            currentPose_.x, currentPose_.y,
            goalPose_.x, goalPose_.y,
            globalPath_);
    }

    /**
     * @brief 定时发布传感器数据
     */
    void publishSensorData()
    {
        auto stamp = now();

        // 里程计 + TF
        float odomX = 0, odomY = 0, odomYaw = 0, odomVx = 0, odomVyaw = 0;
        if (go2_motion_get_odom(&odomX, &odomY, &odomYaw, &odomVx, &odomVyaw)) {
            auto odomMsg = std::make_unique<nav_msgs::msg::Odometry>();
            odomMsg->header.stamp = stamp;
            odomMsg->header.frame_id = "odom";
            odomMsg->child_frame_id = "base_link";
            odomMsg->pose.pose.position.x    = static_cast<double>(odomX);
            odomMsg->pose.pose.position.y    = static_cast<double>(odomY);
            odomMsg->pose.pose.orientation.z = std::sin(odomYaw / 2.0);
            odomMsg->pose.pose.orientation.w = std::cos(odomYaw / 2.0);
            odomMsg->twist.twist.linear.x    = static_cast<double>(odomVx);
            odomMsg->twist.twist.angular.z   = static_cast<double>(odomVyaw);
            odomPub_->publish(std::move(odomMsg));

            auto tfMsg = geometry_msgs::msg::TransformStamped();
            tfMsg.header.stamp = stamp;
            tfMsg.header.frame_id = "odom";
            tfMsg.child_frame_id = "base_link";
            tfMsg.transform.translation.x = static_cast<double>(odomX);
            tfMsg.transform.translation.y = static_cast<double>(odomY);
            tfMsg.transform.rotation.z = std::sin(odomYaw / 2.0);
            tfMsg.transform.rotation.w = std::cos(odomYaw / 2.0);
            tfBroadcaster_->sendTransform(tfMsg);

            auto lidarTf = geometry_msgs::msg::TransformStamped();
            lidarTf.header.stamp = stamp;
            lidarTf.header.frame_id = "base_link";
            lidarTf.child_frame_id = "utlidar_lidar";
            lidarTf.transform.translation.z = 0.4;
            lidarTf.transform.rotation.w = 1.0;
            tfBroadcaster_->sendTransform(lidarTf);
        }

        // LiDAR 数据
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

            // LaserScan
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

            // 更新局部代价地图（用当前激光扫描填充障碍物）
            updateLocalCostmap(scanMsg->ranges.data(), odomX, odomY, odomYaw);
        }
    }

    /**
     * @brief 用激光扫描数据更新局部代价地图
     * @param scanRanges 激光距离数组
     * @param robotX 机器人当前 x 坐标 (m)
     * @param robotY 机器人当前 y 坐标 (m)
     * @param robotYaw 机器人当前偏航角 (rad)
     */
    void updateLocalCostmap(const float* scanRanges, float robotX, float robotY, float robotYaw)
    {
        if (localCostmap_.empty() || !latestMap_) return;

        int w = localCostmapWidth_;
        int h = localCostmapHeight_;
        float res  = localCostmapRes_;
        float origX = localCostmapOrigX_;
        float origY = localCostmapOrigY_;

        std::lock_guard<std::mutex> lock(localCostmapMutex_);

        // 将激光点映射到代价地图
        for (int i = 0; i < LASER_SCAN_SAMPLES; ++i) {
            float range = scanRanges[i];
            if (std::isinf(range) || range <= 0.0f) continue;

            float angle = static_cast<float>(LIDAR_ANGLE_MIN)
                        + i * (static_cast<float>(LIDAR_ANGLE_MAX - LIDAR_ANGLE_MIN)
                               / LASER_SCAN_SAMPLES);
            float worldAngle = angle + robotYaw;
            float wx = robotX + range * std::cos(worldAngle);
            float wy = robotY + range * std::sin(worldAngle);

            int mx = static_cast<int>(std::floor((wx - origX) / res));
            int my = static_cast<int>(std::floor((wy - origY) / res));
            if (mx < 0 || mx >= w || my < 0 || my >= h) continue;

            int obsSize = 2; // 障碍物膨胀半径（栅格）
            for (int dy = -obsSize; dy <= obsSize; ++dy) {
                for (int dx = -obsSize; dx <= obsSize; ++dx) {
                    int nx = mx + dx;
                    int ny = my + dy;
                    if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                    localCostmap_[static_cast<size_t>(ny * w + nx)] = 100;
                }
            }

            // 从机器人到障碍物之间标记为空闲区域
            float step = res * 0.5f;
            float d = step;
            while (d < range - step) {
                float cx = robotX + d * std::cos(worldAngle);
                float cy = robotY + d * std::sin(worldAngle);
                int cmx = static_cast<int>(std::floor((cx - origX) / res));
                int cmy = static_cast<int>(std::floor((cy - origY) / res));
                if (cmx >= 0 && cmx < w && cmy >= 0 && cmy < h) {
                    size_t cidx = static_cast<size_t>(cmy * w + cmx);
                    if (localCostmap_[cidx] >= 0) {
                        localCostmap_[cidx] = 0;
                    }
                }
                d += step;
            }
        }
    }

    /**
     * @brief 规划与控制主循环（定时器回调）
     */
    void controlLoop()
    {
        if (!navigating_) return;

        // 获取当前位置
        float odomX = 0, odomY = 0, odomYaw = 0, odomVx = 0, odomVyaw = 0;
        go2_motion_get_odom(&odomX, &odomY, &odomYaw, &odomVx, &odomVyaw);

        Pose2D currentOdom;
        currentOdom.x   = odomX;
        currentOdom.y   = odomY;
        currentOdom.yaw = odomYaw;

        {
            std::lock_guard<std::mutex> lock(poseMutex_);
            currentPose_ = currentOdom;
        }

        // 检查是否到达目标
        if (dwbPlanner_.isGoalReached(currentOdom, globalPath_)) {
            RCLCPP_INFO(this->get_logger(), "目标已到达!");
            navigating_ = false;
            go2_motion_stop();
            return;
        }

        // 检查是否需要重规划
        rclcpp::Time nowTime = now();
        double dt = (nowTime - lastReplanTime_).seconds();
        if (dt > 5.0) {
            planGlobalPath();
            lastReplanTime_ = nowTime;
        }

        // DWB 局部规划
        float cmdVx = 0.0f, cmdVyaw = 0.0f;
        {
            std::lock_guard<std::mutex> lock(localCostmapMutex_);
            dwbPlanner_.computeVelocity(
                currentOdom, odomVx, odomVyaw,
                globalPath_,
                localCostmap_.data(),
                localCostmapWidth_, localCostmapHeight_,
                localCostmapRes_, localCostmapOrigX_, localCostmapOrigY_,
                cmdVx, cmdVyaw);
        }

        cmdVx   = std::clamp(cmdVx,   -CMD_VX_MAX, CMD_VX_MAX);
        cmdVyaw = std::clamp(cmdVyaw, -CMD_VYAW_MAX, CMD_VYAW_MAX);

        go2_motion_move(cmdVx, 0.0f, cmdVyaw);

        // 发布 cmd_vel 供其他节点使用
        auto cmdMsg = std::make_unique<geometry_msgs::msg::Twist>();
        cmdMsg->linear.x  = static_cast<double>(cmdVx);
        cmdMsg->angular.z = static_cast<double>(cmdVyaw);
        cmdVelPub_->publish(std::move(cmdMsg));
    }

    // ---- ROS2 接口 ----
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr  scanPub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odomPub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    cmdVelPub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amclPoseSub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub_;
    rclcpp::TimerBase::SharedPtr                               publishTimer_;
    rclcpp::TimerBase::SharedPtr                               controlTimer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>             tfBroadcaster_;

    // ---- 规划器 ----
    NavFnPlanner navFnPlanner_;
    DWBPlanner   dwbPlanner_;

    // ---- 状态 ----
    std::atomic<bool> navigating_{false};
    std::atomic<bool> amclPoseValid_{false};
    Pose2D amclPose_{};
    Pose2D currentPose_{};
    Pose2D goalPose_{};
    std::vector<PathPoint> globalPath_;
    rclcpp::Time lastReplanTime_{0, 0, RCL_SYSTEM_TIME};

    // ---- 线程安全 ----
    mutable std::mutex poseMutex_;
    mutable std::mutex mapMutex_;
    mutable std::mutex localCostmapMutex_;

    // ---- 地图数据 ----
    nav_msgs::msg::OccupancyGrid::SharedPtr latestMap_;
    std::vector<int8_t> localCostmap_;
    int localCostmapWidth_  = 0;
    int localCostmapHeight_ = 0;
    float localCostmapRes_  = 0.05f;
    float localCostmapOrigX_ = 0.0f;
    float localCostmapOrigY_ = 0.0f;
};

// ====================================================================
// 命令行输入线程与命令队列
// ====================================================================

static std::atomic<bool> g_running{true};
static std::mutex g_cmdMutex;
static std::queue<std::string> g_cmdQueue;

/**
 * @brief 输入线程函数，持续从标准输入读取指令
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
    std::cout <<   "|  Go2 自主导航程序 (NavFn + DWB)                   |\n";
    std::cout <<   "+----------------------------------------------------+\n";
    std::cout <<   "|  需配合 AMCL + map_server 节点使用                 |\n";
    std::cout <<   "+----------------------------------------------------+\n";
    std::cout <<   "|  命令 (输入后按回车):                              |\n";
    std::cout <<   "|    g / 1 - 导航目标 (x=2, y=0) 向前 2m            |\n";
    std::cout <<   "|    n x y yaw  - 导航到自定义坐标 (如: n 1.5 -2 0)   |
";
    std::cout <<   "|    2     - 导航目标 (x=0, y=2) 向左 2m            |\n";
    std::cout <<   "|    3     - 导航目标 (x=2, y=2) 右前方              |\n";
    std::cout <<   "|    p     - 打印当前导航状态                        |\n";
    std::cout <<   "|    s     - 停止导航                               |\n";
    std::cout <<   "|    q     - 退出程序                               |\n";
    std::cout <<   "+----------------------------------------------------+\n";
    std::cout <<   "|  启动示例:                                         |\n";
    std::cout <<   "|    ros2 run nav2_map_server map_server \\           |\n";
    std::cout <<   "|      --ros-args -p yaml_filename:=./map.yaml        |\n";
    std::cout <<   "|    ros2 run nav2_amcl amcl                         |\n";
    std::cout <<   "|    ./go2_navigation eth0                           |\n";
    std::cout <<   "+----------------------------------------------------+\n";
    std::cout << "\n> " << std::flush;
}

/**
 * @brief 处理命令行指令
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

        case 'g':
        case 'G':
        case '1':
            node->setGoal(2.0f, 0.0f, 0.0f);
            std::cout << "> " << std::flush;
            break;

        case '2':
            node->setGoal(0.0f, 2.0f, static_cast<float>(M_PI_2));
            std::cout << "> " << std::flush;
            break;

        case '3':
            node->setGoal(2.0f, 2.0f, static_cast<float>(M_PI_4));
            std::cout << "> " << std::flush;
            break;

        case 'p':
        case 'P':
            node->printStatus();
            std::cout << "\n> " << std::flush;
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
 * @param argv 参数列表，argv[1] 为网络接口名称
 * @return int 0 正常退出，-1 异常
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

    auto node = std::make_shared<Go2NavigationNode>(netInterface);

    // ---- 机器人站立 ----
    go2_motion_stand_up();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ---- 打印帮助并启动输入线程 ----
    printHelp();

    std::thread inputThread(inputThreadFunc);
    inputThread.detach();

    // ---- 主循环 ----
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
