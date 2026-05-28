/**
 * @file go2_jump.cpp
 * @brief Go2 横棒检测与越障程序，通过 PCL 点云库识别前方横棒并执行前跳越障
 *
 * @par 使用说明
 *       go2_jump <network_interface>
 *       示例: ./go2_jump eth0
 *
 *       说明: 本程序订阅 /utlidar/cloud_base 点云话题，使用 PCL 库进行横棒识别，
 *             检测到横棒后自动行走至棒前，到达合适距离后执行前跳（FrontJump）越障。
 *
 *       控制:
 *             q     - 退出程序
 *             Space - 暂停/恢复运动
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include "go2_motion_bridge.hpp"

#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>

// ==========================================================================
// 横棒检测参数（可微调）
// ==========================================================================
constexpr float ROI_X_MIN          = 0.2f;     ///< 前方最近检测距离 (m)
constexpr float ROI_X_MAX          = 3.0f;     ///< 前方最远检测距离 (m)
constexpr float ROI_Y_RANGE        = 2.0f;     ///< 左右检测范围 (±m)
constexpr float ROI_Z_MIN          = 0.02f;    ///< 横棒最低高度 (m)
constexpr float ROI_Z_MAX          = 0.10f;    ///< 横棒最高高度 (m)
constexpr float VOXEL_LEAF_SIZE    = 0.02f;    ///< 体素降采样叶子大小 (m)
constexpr float RANSAC_THRESHOLD   = 0.03f;    ///< RANSAC 直线拟合距离阈值 (m)
constexpr int   RANSAC_MAX_ITER    = 1000;     ///< RANSAC 最大迭代次数
constexpr int   MIN_LINE_INLIERS   = 15;       ///< 有效横棒最少内点数
constexpr float LINE_HORIZONTAL_Z  = 0.15f;    ///< 直线方向 Z 分量阈值（小于此值认为水平）

// ==========================================================================
// 运动控制参数（可微调）
// ==========================================================================
constexpr float JUMP_DISTANCE      = 0.40f;    ///< 起跳距离 (m)，到达此距离后执行前跳
constexpr float APPROACH_SPEED     = 0.20f;    ///< 接近速度 (m/s)
constexpr float APPROACH_SPEED_MIN = 0.08f;    ///< 最小接近速度 (m/s)
constexpr float ANGULAR_KP         = 0.6f;     ///< 角速度比例系数 (rad/s per meter offset)
constexpr float MAX_ANGULAR_SPEED  = 0.8f;     ///< 最大角速度 (rad/s)
constexpr float DISTANCE_SLOWDOWN  = 1.0f;     ///< 减速起始距离 (m)，距离小于此值时线性降低速度

// ==========================================================================
// 状态机枚举
// ==========================================================================
enum class JumpState {
    SEARCHING,   ///< 搜索横棒中，机器人站立不动
    APPROACHING, ///< 已检测到横棒，正在接近
    JUMPING,     ///< 到达起跳距离，执行前跳
    DONE         ///< 越障完成
};

// ==========================================================================
// 横棒检测结果
// ==========================================================================
struct StickResult
{
    bool   detected     = false; ///< 是否检测到横棒
    float  distance     = 0.0f;  ///< 横棒最近点到机器人前方距离 (m)
    float  centerY      = 0.0f;  ///< 横棒中心 Y 偏移 (m)，正=右侧
    float  centerZ      = 0.0f;  ///< 横棒中心高度 (m)
    int    inlierCount  = 0;     ///< RANSAC 内点数
};

// ==========================================================================
// 全局状态（线程安全）
// ==========================================================================
static std::mutex g_stateMutex;
static JumpState   g_state    = JumpState::SEARCHING;
static StickResult g_lastStick;
static std::atomic<bool> g_running{true};
static std::atomic<bool> g_paused{false};

/**
 * @brief Go2 横棒跳跃节点类
 *
 * 订阅 /utlidar/cloud_base 点云话题，使用 PCL 库进行横棒识别，
 * 通过定时器驱动状态机控制机器狗接近横棒并执行前跳越障。
 */
class Go2JumpNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    Go2JumpNode()
        : Node("go2_jump_node")
    {
        // ---- 点云订阅器 ----
        cloudSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/utlidar/cloud_base", rclcpp::SensorDataQoS(),
            std::bind(&Go2JumpNode::cloudCallback, this, std::placeholders::_1));

        // ---- 控制定时器 (30Hz) ----
        controlTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&Go2JumpNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Go2 横棒跳跃节点已就绪");
    }

private:
    /**
     * @brief 点云回调：将 ROS2 PointCloud2 转换为 PCL 点云并检测横棒
     * @param msg ROS2 PointCloud2 消息
     */
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 仅在搜索和接近状态下处理点云
        JumpState currentState;
        {
            std::lock_guard<std::mutex> lock(g_stateMutex);
            currentState = g_state;
        }
        if (currentState != JumpState::SEARCHING && currentState != JumpState::APPROACHING)
            return;

        // 转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) return;

        StickResult result = detectStick(cloud);

        std::lock_guard<std::mutex> lock(g_stateMutex);
        g_lastStick = result;

        if (result.detected && g_state == JumpState::SEARCHING) {
            g_state = JumpState::APPROACHING;
            RCLCPP_INFO(this->get_logger(),
                "检测到横棒! 距离: %.2fm, 偏移: %.2fm, 高度: %.2fm",
                result.distance, result.centerY, result.centerZ);
        }
    }

    /**
     * @brief 控制循环：根据状态机驱动机器人运动
     */
    void controlLoop()
    {
        if (g_paused) return;

        JumpState currentState;
        StickResult stick;
        {
            std::lock_guard<std::mutex> lock(g_stateMutex);
            currentState = g_state;
            stick = g_lastStick;
        }

        switch (currentState) {
        case JumpState::SEARCHING:
            // 搜索中，机器人原地待命
            break;

        case JumpState::APPROACHING:
            if (!stick.detected) {
                // 丢失横棒信号，减速等待重新检测
                go2_motion_move(0.0f, 0.0f, 0.0f);
                RCLCPP_WARN_THROTTLE(this->get_logger(),
                    *this->get_clock(), 1000,
                    "横棒信号丢失，等待重新检测...");
                break;
            }

            if (stick.distance <= JUMP_DISTANCE) {
                // 到达起跳距离
                go2_motion_stop();
                RCLCPP_INFO(this->get_logger(),
                    "到达起跳距离 (%.2fm)，准备前跳!", stick.distance);
                std::this_thread::sleep_for(std::chrono::milliseconds(300));

                // 执行前跳
                go2_motion_front_jump();
                RCLCPP_INFO(this->get_logger(), "前跳指令已发出!");

                std::this_thread::sleep_for(std::chrono::milliseconds(1500));

                go2_motion_stop();
                {
                    std::lock_guard<std::mutex> lock(g_stateMutex);
                    g_state = JumpState::DONE;
                }
                RCLCPP_INFO(this->get_logger(), "越障完成!");
                break;
            }

            // 计算运动指令
            {
                float vx   = computeApproachSpeed(stick.distance);
                float vyaw = computeAngularCorrection(stick.centerY);

                go2_motion_move(vx, 0.0f, vyaw);

                RCLCPP_INFO_THROTTLE(this->get_logger(),
                    *this->get_clock(), 500,
                    "接近横棒: 距离=%.2fm 偏移=%.2fm | vx=%.2f vyaw=%.2f",
                    stick.distance, stick.centerY, vx, vyaw);
            }
            break;

        case JumpState::JUMPING:
        case JumpState::DONE:
            // 越障完成，保持不动
            go2_motion_stop();
            break;
        }
    }

    /**
     * @brief 使用 PCL 检测点云中的横棒
     *
     * 处理流程：
     *   1. PassThrough 滤波器截取 ROI 区域
     *   2. VoxelGrid 降采样加速处理
     *   3. StatisticalOutlierRemoval 去除离群点
     *   4. RANSAC 直线分割，检查直线是否近似水平
     *
     * @param cloud 输入点云
     * @return StickResult 检测结果
     */
    StickResult detectStick(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        StickResult result;

        // ---- 1. PassThrough 滤波器 ----
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(
            new pcl::PointCloud<pcl::PointXYZ>());

        // X 方向（前方）
        pcl::PassThrough<pcl::PointXYZ> passX;
        passX.setInputCloud(cloud);
        passX.setFilterFieldName("x");
        passX.setFilterLimits(ROI_X_MIN, ROI_X_MAX);
        passX.filter(*cloudFiltered);

        // Y 方向（左右）
        pcl::PassThrough<pcl::PointXYZ> passY;
        passY.setInputCloud(cloudFiltered);
        passY.setFilterFieldName("y");
        passY.setFilterLimits(-ROI_Y_RANGE, ROI_Y_RANGE);
        passY.filter(*cloudFiltered);

        // Z 方向（高度）
        pcl::PassThrough<pcl::PointXYZ> passZ;
        passZ.setInputCloud(cloudFiltered);
        passZ.setFilterFieldName("z");
        passZ.setFilterLimits(ROI_Z_MIN, ROI_Z_MAX);
        passZ.filter(*cloudFiltered);

        if (cloudFiltered->size() < static_cast<size_t>(MIN_LINE_INLIERS))
            return result;

        // ---- 2. VoxelGrid 降采样 ----
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDownsampled(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloudFiltered);
        voxel.setLeafSize(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
        voxel.filter(*cloudDownsampled);

        if (cloudDownsampled->size() < static_cast<size_t>(MIN_LINE_INLIERS))
            return result;

        // ---- 3. 统计离群点去除 ----
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudClean(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloudDownsampled);
        sor.setMeanK(10);
        sor.setStddevMulThresh(1.5);
        sor.filter(*cloudClean);

        if (cloudClean->size() < static_cast<size_t>(MIN_LINE_INLIERS))
            return result;

        // ---- 4. RANSAC 直线分割 ----
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(RANSAC_THRESHOLD);
        seg.setMaxIterations(RANSAC_MAX_ITER);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        seg.setInputCloud(cloudClean);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty() ||
            static_cast<int>(inliers->indices.size()) < MIN_LINE_INLIERS) {
            return result;
        }

        // ---- 5. 检查直线是否近似水平 ----
        // coefficients: [x0, y0, z0, dx, dy, dz]
        float dz = std::fabs(coefficients->values[5]);
        float norm = std::sqrt(
            coefficients->values[3] * coefficients->values[3] +
            coefficients->values[4] * coefficients->values[4] +
            coefficients->values[5] * coefficients->values[5]);
        float dz_norm = (norm > 1e-6f) ? (dz / norm) : 1.0f;

        if (dz_norm > LINE_HORIZONTAL_Z)
            return result;

        // ---- 6. 计算横棒位置信息 ----
        float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
        float minX = std::numeric_limits<float>::max();
        for (int idx : inliers->indices) {
            const auto& pt = cloudClean->points[idx];
            sumX += pt.x;
            sumY += pt.y;
            sumZ += pt.z;
            if (pt.x < minX) minX = pt.x;
        }

        size_t count = inliers->indices.size();
        result.detected    = true;
        result.distance    = minX;
        result.centerY     = sumY / static_cast<float>(count);
        result.centerZ     = sumZ / static_cast<float>(count);
        result.inlierCount = static_cast<int>(count);

        return result;
    }

    /**
     * @brief 根据距离计算接近速度（距离越近速度越慢）
     * @param distance 到横棒的距离 (m)
     * @return float 前进速度 (m/s)
     */
    float computeApproachSpeed(float distance)
    {
        if (distance <= JUMP_DISTANCE)
            return 0.0f;
        if (distance >= DISTANCE_SLOWDOWN)
            return APPROACH_SPEED;

        float ratio = (distance - JUMP_DISTANCE) / (DISTANCE_SLOWDOWN - JUMP_DISTANCE);
        return APPROACH_SPEED_MIN + (APPROACH_SPEED - APPROACH_SPEED_MIN) * ratio;
    }

    /**
     * @brief 根据横棒横向偏移计算角速度修正
     * @param centerY 横棒中心 Y 偏移 (m)，正=右侧
     * @return float 角速度 (rad/s)，正值=左转（以对准横棒）
     */
    float computeAngularCorrection(float centerY)
    {
        float vyaw = -ANGULAR_KP * centerY;
        return std::clamp(vyaw, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
    }

    // ---- ROS2 接口 ----
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSub_;
    rclcpp::TimerBase::SharedPtr controlTimer_;
};

// ==========================================================================
// 主函数
// ==========================================================================

/**
 * @brief Go2 横棒检测与越障程序入口
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

    // ---- 机器人站立 ----
    go2_motion_stand_up();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ---- 初始化 ROS2 ----
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Go2JumpNode>();

    std::cout << "\n+------------------------------------------------+\n";
    std::cout <<   "|  Go2 横棒检测与越障程序                         |\n";
    std::cout <<   "+------------------------------------------------+\n";
    std::cout <<   "|  参数:                                          |\n";
    std::cout <<   "|    起跳距离: " << JUMP_DISTANCE << " m                             |\n";
    std::cout <<   "|    接近速度: " << APPROACH_SPEED << " m/s                          |\n";
    std::cout <<   "|  ROI 前方: " << ROI_X_MIN << "~" << ROI_X_MAX << " m                          |\n";
    std::cout <<   "|  ROI 高度: " << ROI_Z_MIN << "~" << ROI_Z_MAX << " m                        |\n";
    std::cout <<   "+------------------------------------------------+\n";
    std::cout <<   "|  控制: [q] 退出  [Space] 暂停/恢复              |\n";
    std::cout <<   "+------------------------------------------------+\n";
    std::cout << "\n等待横棒检测..." << std::endl;

    // ---- 主循环: ROS2 spin + 键盘控制 ----
    // 由于控制逻辑在 ROS2 定时器中运行，主线程仅处理键盘输入
    while (rclcpp::ok() && g_running) {
        rclcpp::spin_some(node);

        // 检查标准输入是否有键盘输入（非阻塞轮询）
        // 注意: 由于终端可能被其他输出打断，这里使用简单的轮询方式
        struct timeval tv = {0, 50000}; // 50ms timeout
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        int ret = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
        if (ret > 0 && FD_ISSET(STDIN_FILENO, &fds)) {
            char ch;
            if (read(STDIN_FILENO, &ch, 1) == 1) {
                if (ch == 'q' || ch == 'Q') {
                    std::cout << "\n退出程序中..." << std::endl;
                    g_running = false;
                    break;
                } else if (ch == ' ') {
                    g_paused = !g_paused;
                    if (g_paused) {
                        go2_motion_stop();
                        std::cout << ">>> 运动已暂停" << std::endl;
                    } else {
                        std::cout << ">>> 运动已恢复" << std::endl;
                    }
                }
            }
        }
    }

    // ---- 清理 ----
    go2_motion_stop();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
