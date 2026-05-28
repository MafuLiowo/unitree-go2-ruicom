/**
 * @file main.cpp
 * @brief Go2 机器人全流程自动化程序，整合各模块按顺序执行完整任务链路
 *
 * @par 使用说明
 *       main [network_interface]
 *       示例: ./main eth0
 *             ./main           # 使用默认 eth0
 *
 *       任务流程:
 *         1. 关闭避障并前进 move1 步
 *         2. PCL 横棒检测 + 前跳越障
 *         3. 巡线前进 move2 步
 *         4. Nav2 自主导航 (route.yaml 路径点 + DWB 局部避障)
 *         5. 上下楼梯 (爬梯步态 → IMU 水平检测 → 旋转45° → 下楼梯)
 *         6. 巡线行走至红色区域，逆时针旋转45°
 *         7. YOLO 动作识别执行 (stretch/hello/light)
 *         8. 逆时针旋转45°
 *         9. 巡线行走至蓝色区域，左转45°停止
 *
 *       可配置变量: NET_INTERFACE, MOVE1_STEPS, MOVE2_STEPS, ROUTE_FILE 等
 */

// ==========================================================================
// 系统与库头文件
// ==========================================================================
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <yaml-cpp/yaml.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include "go2_motion_bridge.hpp"
#include "DWBPlanner.hpp"
#include "Go2Action.hpp"
#include "YOLODetector.hpp"

#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <vector>
#include <algorithm>
#include <fstream>
#include <queue>
#include <limits>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <sys/select.h>

// ==========================================================================
// 可配置变量
// ==========================================================================
static const std::string NET_INTERFACE = "eth0";               ///< 网络接口
static const int MOVE1_STEPS           = 5;                    ///< 第一阶段前进步数（避障关闭后）
static const int MOVE2_STEPS           = 10;                   ///< 第三阶段巡线前进步数
static const std::string ROUTE_FILE    = "/home/mafu/ai-unitree-go2-ruicom/src/route.yaml"; ///< 导航路径文件
static const std::string YOLO_MODEL    = "/home/unitree/ai-unitree-go2-ruicom/data/best.onnx"; ///< YOLO 模型路径

// ==========================================================================
// 参数常量
// ==========================================================================
constexpr float STEP_DISTANCE          = 0.15f;   ///< 每步前进距离 (m)
constexpr float MOVE_SPEED             = 0.25f;   ///< 普通前进速度 (m/s)
constexpr float ROTATE_SPEED           = 0.785f;  ///< 旋转角速度 (rad/s)，约 45°/s
constexpr float ROTATE_ANGLE_45        = 0.785398f; ///< 45° = π/4
constexpr float ROTATE_DURATION        = ROTATE_ANGLE_45 / ROTATE_SPEED; ///< 旋转 45° 耗时 (s)

// ---- 图像尺寸 ----
constexpr int IMG_WIDTH  = 640;
constexpr int IMG_HEIGHT = 480;
constexpr int IMG_FPS    = 30;

// ---- 巡线参数 ----
constexpr float WALK_FORWARD_SPEED     = 0.30f;
constexpr float WALK_TURN_SPEED        = 0.15f;
constexpr float WALK_MAX_ANGULAR       = 1.20f;
constexpr float WALK_ANGULAR_KP        = 0.025f;
constexpr float RIGHT_ANGLE_SPEED      = 1.50f;
constexpr float RIGHT_ANGLE_DURATION   = 1.05f;
constexpr int   WALK_SPLIT_ROW         = 240;
constexpr int   WALK_UPPER_STRIPS      = 5;
constexpr int   WALK_CURVE_THRESH      = 50;
constexpr int   WALK_SHARP_THRESH      = 100;
constexpr int   CROSS_MIN_WIDTH        = 120;
constexpr int   WALK_BINARY_THRESH     = 128;
constexpr int   WALK_ERODE_ITER        = 1;
constexpr int   WALK_DILATE_ITER       = 2;
constexpr int   WALK_MORPH_KERNEL      = 3;

// ---- 颜色检测参数 ----
constexpr int   RED_HUE_LOW1           = 0;
constexpr int   RED_HUE_HIGH1          = 10;
constexpr int   RED_HUE_LOW2           = 170;
constexpr int   RED_HUE_HIGH2          = 180;
constexpr int   BLUE_HUE_LOW           = 100;
constexpr int   BLUE_HUE_HIGH          = 130;
constexpr int   COLOR_SAT_MIN          = 100;
constexpr int   COLOR_VAL_MIN          = 100;
constexpr float COLOR_AREA_RATIO       = 0.15f;   ///< 颜色区域占比阈值

// ---- 横棒检测参数 ----
constexpr float ROI_X_MIN          = 0.2f;
constexpr float ROI_X_MAX          = 3.0f;
constexpr float ROI_Y_RANGE        = 2.0f;
constexpr float ROI_Z_MIN          = 0.02f;
constexpr float ROI_Z_MAX          = 0.10f;
constexpr float VOXEL_LEAF_SIZE    = 0.02f;
constexpr float RANSAC_THRESHOLD   = 0.03f;
constexpr int   RANSAC_MAX_ITER    = 1000;
constexpr int   MIN_LINE_INLIERS   = 15;
constexpr float LINE_HORIZONTAL_Z  = 0.15f;
constexpr float JUMP_DISTANCE      = 0.40f;
constexpr float APPROACH_SPEED     = 0.20f;
constexpr float APPROACH_SPEED_MIN = 0.08f;
constexpr float JUMP_ANGULAR_KP    = 0.6f;
constexpr float JUMP_MAX_ANGULAR   = 0.8f;
constexpr float DISTANCE_SLOWDOWN  = 1.0f;

// ---- 导航参数 ----
constexpr double LIDAR_ANGLE_MIN      = -M_PI;
constexpr double LIDAR_ANGLE_MAX      =  M_PI;
constexpr double LIDAR_RANGE_MIN      =  0.05;
constexpr double LIDAR_RANGE_MAX      = 20.0;
constexpr int    LASER_SCAN_SAMPLES   = 720;
constexpr double LIDAR_Z_THRESHOLD    = 0.3;
constexpr float NAV_FORWARD_SPEED_MAX = 0.30f;
constexpr float NAV_ANGULAR_SPEED_MAX = 1.00f;
constexpr double PUBLISH_RATE_HZ      = 30.0;
constexpr double DWB_CONTROL_RATE_HZ  = 20.0;

// ---- 楼梯参数 ----
constexpr float UPSTAIR_VEL            = 0.3f;
constexpr float DOWNSTAIR_VEL          = 0.3f;
constexpr float DOWNSTAIR_DIST         = 1.5f;
constexpr float STAIR_ROTATE_SPEED     = 0.5f;
constexpr float STAIR_ROTATE_ANGLE     = 0.785398f;
constexpr float FLAT_PITCH_THRESHOLD   = 0.087f;
constexpr float FLAT_ROLL_THRESHOLD    = 0.087f;
constexpr int   STABLE_FRAMES_REQUIRED = 10;

// ---- YOLO 参数 ----
constexpr float YOLO_CONF_THRESHOLD = 0.5f;
constexpr float YOLO_NMS_THRESHOLD  = 0.5f;
constexpr int   YOLO_INPUT_SIZE     = 640;

/// Go2 足迹多边形
const std::vector<FootprintPoint> GO2_FOOTPRINT = {
    { 0.38f,  0.20f, 0.0f},
    { 0.38f, -0.20f, 0.0f},
    {-0.35f, -0.20f, 0.0f},
    {-0.35f,  0.20f, 0.0f},
};

// ==========================================================================
// 颜色检测枚举
// ==========================================================================
enum class TargetColor { RED, BLUE };

/// 巡线转弯类型
enum class WalkTurnType {
    STRAIGHT, LEFT_CURVE, RIGHT_CURVE,
    LEFT_RIGHT_ANGLE, RIGHT_RIGHT_ANGLE, CROSS
};

// ==========================================================================
// 全局控制变量
// ==========================================================================
static std::atomic<bool> g_automationRunning{true};

// ==========================================================================
// 辅助函数
// ==========================================================================

/// 当前时间（ROS2）
inline rclcpp::Time now() { return rclcpp::Clock().now(); }

/**
 * @brief 让机器人前进指定步数
 * @param steps 步数
 */
static void moveForwardSteps(int steps)
{
    float duration = steps * STEP_DISTANCE / MOVE_SPEED;
    std::cout << "[MOVE] 前进 " << steps << " 步 (约 " << (steps * STEP_DISTANCE)
              << "m)，速度 " << MOVE_SPEED << " m/s，预计 " << duration << "s" << std::endl;
    go2_motion_move(MOVE_SPEED, 0.0f, 0.0f);

    auto start = std::chrono::steady_clock::now();
    while (true) {
        auto elapsed = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= duration) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    go2_motion_stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "[MOVE] 前进完成" << std::endl;
}

/**
 * @brief 逆时针旋转指定弧度
 * @param rad 旋转弧度（正 = 逆时针）
 */
static void rotate(float rad)
{
    float duration = std::fabs(rad) / ROTATE_SPEED;
    float dir = (rad > 0) ? 1.0f : -1.0f;
    std::cout << "[ROTATE] " << (rad > 0 ? "逆时针" : "顺时针")
              << "旋转 " << (std::fabs(rad) * 180.0 / M_PI) << "°" << std::endl;

    go2_motion_move(0.0f, 0.0f, dir * ROTATE_SPEED);

    auto start = std::chrono::steady_clock::now();
    while (true) {
        auto elapsed = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= duration) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    go2_motion_stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "[ROTATE] 旋转完成" << std::endl;
}

/**
 * @brief 检测图像中指定颜色的面积占比
 * @param bgr 输入 BGR 图像
 * @param target 目标颜色
 * @return float 颜色区域占比 (0~1)
 */
static float detectColorAreaRatio(const cv::Mat& bgr, TargetColor target)
{
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    if (target == TargetColor::RED) {
        cv::Mat mask1, mask2;
        cv::inRange(hsv,
            cv::Scalar(RED_HUE_LOW1, COLOR_SAT_MIN, COLOR_VAL_MIN),
            cv::Scalar(RED_HUE_HIGH1, 255, 255), mask1);
        cv::inRange(hsv,
            cv::Scalar(RED_HUE_LOW2, COLOR_SAT_MIN, COLOR_VAL_MIN),
            cv::Scalar(RED_HUE_HIGH2, 255, 255), mask2);
        cv::bitwise_or(mask1, mask2, mask);
    } else {
        cv::inRange(hsv,
            cv::Scalar(BLUE_HUE_LOW, COLOR_SAT_MIN, COLOR_VAL_MIN),
            cv::Scalar(BLUE_HUE_HIGH, 255, 255), mask);
    }

    int totalPixels = bgr.rows * bgr.cols;
    int colorPixels = cv::countNonZero(mask);
    return static_cast<float>(colorPixels) / static_cast<float>(totalPixels);
}

// ==========================================================================
// 巡线图像处理函数
// ==========================================================================

/**
 * @brief V 通道反二值化
 */
static cv::Mat vChannelInverseBinary(const cv::Mat& bgrFrame, int threshold)
{
    cv::Mat hsv;
    cv::cvtColor(bgrFrame, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    cv::Mat vChannel = channels[2];
    int t = std::clamp(threshold, 0, 255);
    cv::Mat result;
    cv::threshold(vChannel, result, t, 255, cv::THRESH_BINARY_INV);
    return result;
}

/**
 * @brief 形态学操作
 */
static cv::Mat applyMorphology(const cv::Mat& binary, int erodeIter, int dilateIter, int kernelSize)
{
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    cv::Mat eroded, dilated;
    cv::erode(binary, eroded, kernel, cv::Point(-1, -1), erodeIter);
    cv::dilate(eroded, dilated, kernel, cv::Point(-1, -1), dilateIter);
    return dilated;
}

/**
 * @brief 列投影法求白色区域质心
 */
static float findColumnProjectionCenter(const cv::Mat& binary)
{
    if (binary.empty()) return -1.0f;
    cv::Mat colSum;
    cv::reduce(binary, colSum, 0, cv::REDUCE_SUM, CV_32F);
    double totalMass = cv::sum(colSum)[0];
    if (totalMass < 10.0) return -1.0f;
    float weightedSum = 0.0f;
    for (int c = 0; c < colSum.cols; ++c) {
        weightedSum += colSum.at<float>(0, c) * static_cast<float>(c);
    }
    return weightedSum / static_cast<float>(totalMass);
}

/**
 * @brief 上半部分转弯类型分析
 */
static WalkTurnType analyzeUpperHalf(const cv::Mat& upperBinary, float lowerMidX,
                                     int curveThreshold, int sharpThreshold, int numStrips)
{
    if (upperBinary.empty() || lowerMidX < 0) return WalkTurnType::STRAIGHT;
    int rows = upperBinary.rows;
    int cols = upperBinary.cols;
    int stripHeight = std::max(1, rows / numStrips);

    std::vector<float> stripCenters;
    for (int i = 0; i < numStrips; ++i) {
        int yStart = i * stripHeight;
        int yEnd   = std::min((i + 1) * stripHeight, rows);
        cv::Rect stripRect(0, yStart, cols, yEnd - yStart);
        cv::Mat strip = upperBinary(stripRect);
        float cx = findColumnProjectionCenter(strip);
        if (cx >= 0) stripCenters.push_back(cx);
    }
    if (stripCenters.size() < 2) return WalkTurnType::STRAIGHT;

    float upperAvgX = 0.0f;
    for (float cx : stripCenters) upperAvgX += cx;
    upperAvgX /= static_cast<float>(stripCenters.size());
    float offset = upperAvgX - lowerMidX;

    cv::Mat colSum;
    cv::reduce(upperBinary, colSum, 0, cv::REDUCE_SUM, CV_32F);
    int whiteColumns = 0;
    for (int c = 0; c < colSum.cols; ++c) {
        if (colSum.at<float>(0, c) > 1.0f) whiteColumns++;
    }
    if (whiteColumns > CROSS_MIN_WIDTH && std::abs(offset) < sharpThreshold) {
        return WalkTurnType::CROSS;
    }

    float absOff = std::abs(offset);
    if (absOff < curveThreshold) return WalkTurnType::STRAIGHT;
    if (absOff > sharpThreshold) {
        return (offset < 0) ? WalkTurnType::LEFT_RIGHT_ANGLE : WalkTurnType::RIGHT_RIGHT_ANGLE;
    }
    return (offset < 0) ? WalkTurnType::LEFT_CURVE : WalkTurnType::RIGHT_CURVE;
}

// ==========================================================================
// 阶段 1: 关闭避障并前进 move1 步
// ==========================================================================

/**
 * @brief 阶段1：关闭避障模式并前进若干步
 */
static bool stageCloseObstaclesAndMove(int steps)
{
    std::cout << "\n+================================================+\n";
    std::cout <<   "|  阶段 1: 关闭避障并前进 " << steps << " 步                        |\n";
    std::cout <<   "+================================================+\n";

    unitree::robot::go2::ObstaclesAvoidClient oaClient;
    oaClient.Init();
    oaClient.SetTimeout(5.0f);

    std::cout << "[OBS] 正在关闭避障模式..." << std::endl;
    oaClient.SwitchSet(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    bool enabled = true;
    oaClient.SwitchGet(enabled);
    std::cout << "[OBS] 避障状态: " << (enabled ? "开启" : "已关闭") << std::endl;

    go2_motion_stand_up();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    moveForwardSteps(steps);
    return true;
}

// ==========================================================================
// 阶段 2: PCL 横棒检测 + 前跳越障
// ==========================================================================

/// 横棒检测结果
struct StickResult {
    bool   detected = false;
    float  distance = 0.0f;
    float  centerY  = 0.0f;
    float  centerZ  = 0.0f;
    int    inlierCount = 0;
};

/**
 * @brief PCL 横棒检测函数（RANSAC 直线拟合）
 * @param cloud 输入点云
 * @return StickResult 检测结果
 */
static StickResult detectStick(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    StickResult result;

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PassThrough<pcl::PointXYZ> passX;
        passX.setInputCloud(cloud);
        passX.setFilterFieldName("x");
        passX.setFilterLimits(ROI_X_MIN, ROI_X_MAX);
        passX.filter(*tmp);
        pcl::PassThrough<pcl::PointXYZ> passY;
        passY.setInputCloud(tmp);
        passY.setFilterFieldName("y");
        passY.setFilterLimits(-ROI_Y_RANGE, ROI_Y_RANGE);
        passY.filter(*filtered);
        pcl::PassThrough<pcl::PointXYZ> passZ;
        passZ.setInputCloud(filtered);
        passZ.setFilterFieldName("z");
        passZ.setFilterLimits(ROI_Z_MIN, ROI_Z_MAX);
        passZ.filter(*filtered);
    }
    if (filtered->size() < static_cast<size_t>(MIN_LINE_INLIERS)) return result;

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(filtered);
    voxel.setLeafSize(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
    voxel.filter(*downsampled);
    if (downsampled->size() < static_cast<size_t>(MIN_LINE_INLIERS)) return result;

    pcl::PointCloud<pcl::PointXYZ>::Ptr clean(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(downsampled);
    sor.setMeanK(10);
    sor.setStddevMulThresh(1.5);
    sor.filter(*clean);
    if (clean->size() < static_cast<size_t>(MIN_LINE_INLIERS)) return result;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(RANSAC_THRESHOLD);
    seg.setMaxIterations(RANSAC_MAX_ITER);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    seg.setInputCloud(clean);
    seg.segment(*inliers, *coeff);
    if (inliers->indices.empty() || static_cast<int>(inliers->indices.size()) < MIN_LINE_INLIERS)
        return result;

    float dz_norm = std::fabs(coeff->values[5]) /
        std::sqrt(coeff->values[3]*coeff->values[3] + coeff->values[4]*coeff->values[4] + coeff->values[5]*coeff->values[5]);
    if (dz_norm > LINE_HORIZONTAL_Z) return result;

    float sumX = 0, sumY = 0, sumZ = 0, minX = std::numeric_limits<float>::max();
    for (int idx : inliers->indices) {
        const auto& pt = clean->points[idx];
        sumX += pt.x; sumY += pt.y; sumZ += pt.z;
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

/// 跳跃阶段 ROS2 节点（自包含状态）
class JumpStageNode : public rclcpp::Node {
public:
    enum class State { SEARCHING, APPROACHING, JUMPING, DONE };

    JumpStageNode() : Node("_stage2_jump") {}

    void init() {
        cloudSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/utlidar/cloud_base", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                if (!running_) return;
                State state;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    state = state_;
                }
                if (state != State::SEARCHING && state != State::APPROACHING) return;

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::fromROSMsg(*msg, *cloud);
                if (cloud->empty()) return;

                StickResult result = detectStick(cloud);
                std::lock_guard<std::mutex> lock(mutex_);
                lastStick_ = result;
                if (result.detected && state_ == State::SEARCHING) {
                    state_ = State::APPROACHING;
                    std::cout << "[JUMP] 检测到横棒! 距离=" << result.distance
                              << "m 偏移=" << result.centerY << "m 高度=" << result.centerZ << "m" << std::endl;
                }
            });

        controlTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), [this]() { controlLoop(); });
    }

    bool isDone() const { return done_; }
    void stop() { running_ = false; }

private:
    void controlLoop() {
        if (!running_) return;
        State state;
        StickResult stick;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state = state_;
            stick = lastStick_;
        }

        switch (state) {
        case State::SEARCHING: break;
        case State::APPROACHING:
            if (!stick.detected) {
                go2_motion_move(0, 0, 0);
                break;
            }
            if (stick.distance <= JUMP_DISTANCE) {
                go2_motion_stop();
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                std::cout << "[JUMP] 到达起跳距离! 执行前跳!" << std::endl;
                go2_motion_front_jump();
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                go2_motion_stop();
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    state_ = State::DONE;
                }
                done_ = true;
                break;
            }
            {
                float ratio = (stick.distance - JUMP_DISTANCE) / (DISTANCE_SLOWDOWN - JUMP_DISTANCE);
                ratio = std::clamp(ratio, 0.0f, 1.0f);
                float vx = APPROACH_SPEED_MIN + (APPROACH_SPEED - APPROACH_SPEED_MIN) * ratio;
                float vyaw = std::clamp(-JUMP_ANGULAR_KP * stick.centerY, -JUMP_MAX_ANGULAR, JUMP_MAX_ANGULAR);
                go2_motion_move(vx, 0.0f, vyaw);
            }
            break;
        case State::JUMPING:
        case State::DONE:
            go2_motion_stop();
            break;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSub_;
    rclcpp::TimerBase::SharedPtr controlTimer_;

    std::mutex mutex_;
    State state_ = State::SEARCHING;
    StickResult lastStick_;
    std::atomic<bool> running_{true};
    std::atomic<bool> done_{false};
};

/**
 * @brief 阶段2：横棒检测与跳跃
 */
static bool stageStickJump()
{
    std::cout << "\n+================================================+\n";
    std::cout <<   "|  阶段 2: PCL 横棒检测 + 前跳越障               |\n";
    std::cout <<   "+================================================+\n";

    go2_motion_stand_up();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto node = std::make_shared<JumpStageNode>();
    node->init();

    std::cout << "[JUMP] 等待横棒检测..." << std::endl;

    while (rclcpp::ok() && !node->isDone() && g_automationRunning) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    node->stop();
    go2_motion_stop();
    std::cout << "[JUMP] 横棒越障完成!" << std::endl;
    return node->isDone();
}

// ==========================================================================
// 阶段 3: 巡线前进 move2 步
// ==========================================================================

/**
 * @brief 阶段3：巡线前进若干步
 * @param rsPipe RealSense pipeline 引用
 * @param steps 步数
 */
static bool stageWalkingForSteps(rs2::pipeline& rsPipe, int steps)
{
    std::cout << "\n+================================================+\n";
    std::cout <<   "|  阶段 3: 巡线前进 " << steps << " 步                           |\n";
    std::cout <<   "+================================================+\n";

    float targetDistance = steps * STEP_DISTANCE;
    std::cout << "[WALK] 目标距离: " << targetDistance << "m" << std::endl;

    unitree::robot::go2::SportClient sportClient;
    sportClient.SetTimeout(10.0f);
    sportClient.Init();

    float movedDistance = 0.0f;
    bool walkingDone = false;
    int frameCount = 0;

    auto walkStartTime = std::chrono::steady_clock::now();
    auto lastTime = walkStartTime;

    while (!walkingDone) {
        rs2::frameset frames;
        try {
            frames = rsPipe.wait_for_frames(100);
        } catch (...) {
            std::cerr << "[WALK] 获取帧失败，重试..." << std::endl;
            continue;
        }
        rs2::frame colorFrame = frames.get_color_frame();
        if (!colorFrame) continue;

        cv::Mat bgrFrame(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3,
                         const_cast<void*>(colorFrame.get_data()), cv::Mat::AUTO_STEP);
        ++frameCount;

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;

        // 图像处理
        cv::Mat binary = vChannelInverseBinary(bgrFrame, WALK_BINARY_THRESH);
        cv::Mat binaryMorphed = applyMorphology(binary, WALK_ERODE_ITER, WALK_DILATE_ITER, WALK_MORPH_KERNEL);

        int splitRow = WALK_SPLIT_ROW;
        cv::Rect lowerROI(0, splitRow, IMG_WIDTH, IMG_HEIGHT - splitRow);
        cv::Rect upperROI(0, 0, IMG_WIDTH, splitRow);
        cv::Mat lowerBinary = binaryMorphed(lowerROI);
        cv::Mat upperBinary = binaryMorphed(upperROI);

        float lowerMidX = findColumnProjectionCenter(lowerBinary);
        float errorPx   = 0.0f;
        WalkTurnType turnType = WalkTurnType::STRAIGHT;

        if (lowerMidX >= 0) {
            errorPx  = lowerMidX - (IMG_WIDTH / 2.0f);
            turnType = analyzeUpperHalf(upperBinary, lowerMidX,
                                        WALK_CURVE_THRESH, WALK_SHARP_THRESH, WALK_UPPER_STRIPS);

            float vx = WALK_FORWARD_SPEED;
            float vyaw = 0.0f;

            switch (turnType) {
            case WalkTurnType::STRAIGHT:
                vyaw = std::clamp(WALK_ANGULAR_KP * errorPx, -WALK_MAX_ANGULAR * 0.3f, WALK_MAX_ANGULAR * 0.3f);
                break;
            case WalkTurnType::LEFT_CURVE:
            case WalkTurnType::RIGHT_CURVE:
                vx = WALK_TURN_SPEED;
                vyaw = std::clamp(WALK_ANGULAR_KP * errorPx * 1.5f, -WALK_MAX_ANGULAR, WALK_MAX_ANGULAR);
                break;
            case WalkTurnType::LEFT_RIGHT_ANGLE:
            case WalkTurnType::RIGHT_RIGHT_ANGLE: {
                sportClient.StopMove();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                float dir = (turnType == WalkTurnType::LEFT_RIGHT_ANGLE) ? 1.0f : -1.0f;
                std::cout << "[WALK] 直角转弯: " << (dir > 0 ? "左转" : "右转") << std::endl;
                auto tStart = std::chrono::steady_clock::now();
                while (std::chrono::duration<float>(std::chrono::steady_clock::now() - tStart).count() < RIGHT_ANGLE_DURATION) {
                    sportClient.Move(0.0f, 0.0f, dir * RIGHT_ANGLE_SPEED);
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                sportClient.StopMove();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                std::cout << "[WALK] 直角转弯完成" << std::endl;
                vx = 0.0f;
                break;
            }
            case WalkTurnType::CROSS:
                vx = WALK_FORWARD_SPEED;
                vyaw = std::clamp(WALK_ANGULAR_KP * errorPx, -WALK_MAX_ANGULAR * 0.2f, WALK_MAX_ANGULAR * 0.2f);
                break;
            }
            sportClient.Move(vx, 0.0f, vyaw);

            // 估算移动距离
            movedDistance += vx * dt;
        }

        // 检查是否完成
        if (movedDistance >= targetDistance) {
            walkingDone = true;
        }

        // 显示画面
        cv::Mat display = bgrFrame.clone();
        cv::line(display, cv::Point(0, splitRow), cv::Point(display.cols - 1, splitRow),
                 cv::Scalar(255, 255, 0), 2);
        std::string statusText = "Walk Steps: " + std::to_string(movedDistance).substr(0, 5)
                               + "/" + std::to_string(targetDistance).substr(0, 5) + "m";
        cv::putText(display, statusText, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Stage3: Walking", display);
        cv::imshow("Binary", binaryMorphed);

        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) {
            sportClient.StopMove();
            return false;
        }
    }

    sportClient.StopMove();
    cv::destroyWindow("Stage3: Walking");
    cv::destroyWindow("Binary");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "[WALK] 巡线前进完成 (实际 " << movedDistance << "m)" << std::endl;
    return true;
}

// ==========================================================================
// 阶段 4: Nav2 自主导航
// ==========================================================================

/// 路径点结构
struct Waypoint { std::string frame; double x; double y; double yaw; };

/**
 * @brief 解析 route.yaml 文件
 */
static std::vector<Waypoint> parseRouteYaml(const std::string& filepath)
{
    std::vector<Waypoint> waypoints;
    std::ifstream checkFile(filepath);
    if (!checkFile.good()) { std::cerr << "[NAV] 文件不存在: " << filepath << std::endl; return waypoints; }
    checkFile.close();
    try {
        YAML::Node config = YAML::LoadFile(filepath);
        YAML::Node wpNode = config["waypoints"];
        if (!wpNode || !wpNode.IsSequence()) { std::cerr << "[NAV] waypoints 字段不存在" << std::endl; return waypoints; }
        for (const auto& wp : wpNode) {
            Waypoint w;
            w.frame = wp["frame"] ? wp["frame"].as<std::string>() : "map";
            w.x     = wp["x"].as<double>();
            w.y     = wp["y"].as<double>();
            w.yaw   = wp["yaw"].as<double>();
            waypoints.push_back(w);
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "[NAV] YAML 解析失败: " << e.what() << std::endl;
    }
    return waypoints;
}

/**
 * @brief 阶段4：自主导航
 */
static bool stageNavigation()
{
    std::cout << "\n+================================================+\n";
    std::cout <<   "|  阶段 4: Nav2 自主导航                          |\n";
    std::cout <<   "+================================================+\n";

    auto waypoints = parseRouteYaml(ROUTE_FILE);
    if (waypoints.empty()) {
        std::cerr << "[NAV] 路径点为空，跳过导航阶段" << std::endl;
        return false;
    }
    std::cout << "[NAV] 读取 " << waypoints.size() << " 个路径点" << std::endl;

    class NavNode : public rclcpp::Node {
    public:
        NavNode(const std::vector<Waypoint>& waypoints)
            : Node("_stage4_nav"), waypoints_(waypoints) {}

        bool isAllDone() const { return allDone_; }

        void init() {
            // DWB 配置
            DWBConfig dwbConfig;
            dwbConfig.footprint = GO2_FOOTPRINT;
            dwbConfig.maxLinearVel = NAV_FORWARD_SPEED_MAX;
            dwbConfig.maxAngularVel = NAV_ANGULAR_SPEED_MAX;
            dwbConfig.minAngularVel = -NAV_ANGULAR_SPEED_MAX;
            dwbConfig.robotRadius = 0.35f;
            dwbConfig.goalTolerance = 0.3f;
            dwbConfig.lookaheadIndex = 5;
            dwbPlanner_.setConfig(dwbConfig);

            cloudPub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/utlidar/cloud", 10);
            scanPub_  = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
            odomPub_  = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

            planSub_ = create_subscription<nav_msgs::msg::Path>(
                "/plan", rclcpp::QoS(10).transient_local(),
                [this](const nav_msgs::msg::Path::SharedPtr msg) {
                    std::vector<PathPoint> plan;
                    plan.reserve(msg->poses.size());
                    for (const auto& p : msg->poses) {
                        PathPoint pt;
                        pt.x = static_cast<float>(p.pose.position.x);
                        pt.y = static_cast<float>(p.pose.position.y);
                        pt.dx = pt.dy = 0;
                        plan.push_back(pt);
                    }
                    std::lock_guard<std::mutex> lock(planMutex_);
                    currentPlan_ = std::move(plan);
                });

            costmapSub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/local_costmap/costmap", 10,
                [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(costmapMutex_);
                    currentCostmap_ = *msg;
                });

            tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            navToPoseClient_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
            followWpClient_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(this, "follow_waypoints");

            publishTimer_ = create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / PUBLISH_RATE_HZ)),
                std::bind(&NavNode::publishSensorData, this));

            dwbTimer_ = create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / DWB_CONTROL_RATE_HZ)),
                std::bind(&NavNode::dwbControlLoop, this));
        }

        void startNav() {
            if (followWpClient_->wait_for_action_server(std::chrono::seconds(2))) {
                std::cout << "[NAV] FollowWaypoints 模式" << std::endl;
                sendFollowWaypoints();
            } else {
                std::cout << "[NAV] NavigateToPose 逐个模式" << std::endl;
                wpIdx_ = 0;
                sendNextWaypoint();
            }
        }

        void stopNav() {
            allDone_ = false;
            cancelAllGoals();
            go2_motion_stop();
        }

    private:
        void sendNextWaypoint() {
            if (wpIdx_ >= waypoints_.size()) { allDone_ = true; return; }
            if (!navToPoseClient_->wait_for_action_server(std::chrono::seconds(3))) return;

            const auto& wp = waypoints_[wpIdx_];
            auto goal = nav2_msgs::action::NavigateToPose::Goal();
            goal.pose.header.frame_id = wp.frame;
            goal.pose.header.stamp = now();
            goal.pose.pose.position.x = wp.x;
            goal.pose.pose.position.y = wp.y;
            goal.pose.pose.orientation.z = std::sin(wp.yaw / 2.0);
            goal.pose.pose.orientation.w = std::cos(wp.yaw / 2.0);

            std::cout << "[NAV] 目标 [" << (wpIdx_+1) << "/" << waypoints_.size()
                      << "]: x=" << wp.x << " y=" << wp.y << std::endl;

            auto opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            opts.result_callback = [this](auto result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    std::cout << "[NAV] 路径点 [" << (wpIdx_+1) << "/" << waypoints_.size() << "] 到达" << std::endl;
                    wpIdx_++;
                    sendNextWaypoint();
                } else {
                    std::cout << "[NAV] 路径点失败" << std::endl;
                    allDone_ = true;
                }
            };
            auto future = navToPoseClient_->async_send_goal(goal, opts);
            {
                std::lock_guard<std::mutex> lock(goalMutex_);
                navGoalHandle_ = future.get();
            }
        }

        void sendFollowWaypoints() {
            auto goal = nav2_msgs::action::FollowWaypoints::Goal();
            for (const auto& wp : waypoints_) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = wp.frame;
                pose.header.stamp = now();
                pose.pose.position.x = wp.x;
                pose.pose.position.y = wp.y;
                pose.pose.orientation.z = std::sin(wp.yaw / 2.0);
                pose.pose.orientation.w = std::cos(wp.yaw / 2.0);
                goal.poses.push_back(pose);
            }
            std::cout << "[NAV] 发送 " << waypoints_.size() << " 个路径点" << std::endl;
            auto opts = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
            opts.feedback_callback = [this](auto, auto feedback) {
                std::cout << "[NAV] 前往 " << (feedback->current_waypoint + 1) << "/" << waypoints_.size() << std::endl;
            };
            opts.result_callback = [this](auto result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    std::cout << "[NAV] 全部路径点完成!" << std::endl;
                }
                allDone_ = true;
            };
            auto future = followWpClient_->async_send_goal(goal, opts);
            {
                std::lock_guard<std::mutex> lock(goalMutex_);
                followGoalHandle_ = future.get();
            }
        }

        void cancelAllGoals() {
            std::lock_guard<std::mutex> lock(goalMutex_);
            if (navGoalHandle_) { navToPoseClient_->async_cancel_goal(navGoalHandle_); navGoalHandle_.reset(); }
            if (followGoalHandle_) { followWpClient_->async_cancel_goal(followGoalHandle_); followGoalHandle_.reset(); }
        }

        void publishSensorData() {
            auto stamp = now();
            float ox=0, oy=0, oyaw=0, ovx=0, ovyaw=0;
            if (go2_motion_get_odom(&ox, &oy, &oyaw, &ovx, &ovyaw)) {
                {
                    std::lock_guard<std::mutex> lock(odomMutex_);
                    odomX_=ox; odomY_=oy; odomYaw_=oyaw; odomVx_=ovx; odomVyaw_=ovyaw;
                }
                auto omsg = std::make_unique<nav_msgs::msg::Odometry>();
                omsg->header.stamp = stamp;
                omsg->header.frame_id = "odom";
                omsg->child_frame_id = "base_link";
                omsg->pose.pose.position.x = ox; omsg->pose.pose.position.y = oy;
                omsg->pose.pose.orientation.z = std::sin(oyaw/2.0);
                omsg->pose.pose.orientation.w = std::cos(oyaw/2.0);
                omsg->twist.twist.linear.x = ovx;
                omsg->twist.twist.angular.z = ovyaw;
                odomPub_->publish(std::move(omsg));

                auto tf = geometry_msgs::msg::TransformStamped();
                tf.header.stamp = stamp;
                tf.header.frame_id = "odom";
                tf.child_frame_id = "base_link";
                tf.transform.translation.x = ox; tf.transform.translation.y = oy;
                tf.transform.rotation.z = std::sin(oyaw/2.0);
                tf.transform.rotation.w = std::cos(oyaw/2.0);
                tfBroadcaster_->sendTransform(tf);

                auto ltf = geometry_msgs::msg::TransformStamped();
                ltf.header.stamp = stamp;
                ltf.header.frame_id = "base_link";
                ltf.child_frame_id = "utlidar_lidar";
                ltf.transform.translation.z = 0.4;
                ltf.transform.rotation.w = 1.0;
                tfBroadcaster_->sendTransform(ltf);
            }

            constexpr uint32_t MAX_PTS = 50000;
            static float buf[MAX_PTS * 4];
            uint32_t pc=0, w=0, h=0;
            if (go2_motion_get_lidar(buf, MAX_PTS, &pc, &w, &h)) {
                auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
                cloud->header.stamp = stamp;
                cloud->header.frame_id = "utlidar_lidar";
                cloud->height=h; cloud->width=w; cloud->is_bigendian=false;
                cloud->point_step=16; cloud->row_step=16*w; cloud->is_dense=true;
                sensor_msgs::msg::PointField fx,fy,fz,fi;
                fx.name="x"; fx.offset=0; fx.datatype=7; fx.count=1;
                fy.name="y"; fy.offset=4; fy.datatype=7; fy.count=1;
                fz.name="z"; fz.offset=8; fz.datatype=7; fz.count=1;
                fi.name="intensity"; fi.offset=12; fi.datatype=7; fi.count=1;
                cloud->fields={fx,fy,fz,fi};
                cloud->data.assign(reinterpret_cast<uint8_t*>(buf), reinterpret_cast<uint8_t*>(buf)+pc*16);
                cloudPub_->publish(std::move(cloud));

                auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
                scan->header.stamp=stamp; scan->header.frame_id="utlidar_lidar";
                scan->angle_min=LIDAR_ANGLE_MIN; scan->angle_max=LIDAR_ANGLE_MAX;
                scan->angle_increment=(LIDAR_ANGLE_MAX-LIDAR_ANGLE_MIN)/LASER_SCAN_SAMPLES;
                scan->range_min=LIDAR_RANGE_MIN; scan->range_max=LIDAR_RANGE_MAX;
                scan->ranges.assign(LASER_SCAN_SAMPLES, std::numeric_limits<float>::infinity());
                for (uint32_t i=0; i<pc; ++i) {
                    float px=buf[i*4], py=buf[i*4+1], pz=buf[i*4+2];
                    if (std::fabs(pz)>LIDAR_Z_THRESHOLD) continue;
                    float r = std::sqrt(px*px+py*py);
                    if (r<LIDAR_RANGE_MIN || r>LIDAR_RANGE_MAX) continue;
                    float a = std::atan2(py, px);
                    int idx = static_cast<int>((a-LIDAR_ANGLE_MIN)/scan->angle_increment+0.5);
                    if (idx<0 || idx>=LASER_SCAN_SAMPLES) continue;
                    if (r < scan->ranges[idx]) scan->ranges[idx] = r;
                }
                scanPub_->publish(std::move(scan));
            }
        }

        void dwbControlLoop() {
            float ox, oy, oyaw, ovx, ovyaw;
            {
                std::lock_guard<std::mutex> lock(odomMutex_);
                ox=odomX_; oy=odomY_; oyaw=odomYaw_; ovx=odomVx_; ovyaw=odomVyaw_;
            }
            std::vector<PathPoint> plan;
            { std::lock_guard<std::mutex> lock(planMutex_); plan = currentPlan_; }
            nav_msgs::msg::OccupancyGrid costmap;
            { std::lock_guard<std::mutex> lock(costmapMutex_); costmap = currentCostmap_; }
            if (plan.empty() || costmap.data.empty()) { go2_motion_move(0,0,0); return; }
            Pose2D pose; pose.x=ox; pose.y=oy; pose.yaw=oyaw;
            float cvx=0, cvyaw=0;
            bool ok = dwbPlanner_.computeVelocity(pose, ovx, ovyaw, plan,
                costmap.data.data(), static_cast<int>(costmap.info.width),
                static_cast<int>(costmap.info.height),
                static_cast<float>(costmap.info.resolution),
                static_cast<float>(costmap.info.origin.position.x),
                static_cast<float>(costmap.info.origin.position.y), cvx, cvyaw);
            if (ok) {
                go2_motion_move(std::clamp(cvx, -NAV_FORWARD_SPEED_MAX, NAV_FORWARD_SPEED_MAX),
                                0.0f,
                                std::clamp(cvyaw, -NAV_ANGULAR_SPEED_MAX, NAV_ANGULAR_SPEED_MAX));
            } else {
                go2_motion_move(0,0,0);
            }
        }

        std::vector<Waypoint> waypoints_;
        size_t wpIdx_ = 0;
        DWBPlanner dwbPlanner_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scanPub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
        rclcpp::TimerBase::SharedPtr publishTimer_, dwbTimer_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navToPoseClient_;
        rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr followWpClient_;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planSub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmapSub_;

        mutable std::mutex odomMutex_, planMutex_, costmapMutex_, goalMutex_;
        float odomX_=0, odomY_=0, odomYaw_=0, odomVx_=0, odomVyaw_=0;
        std::vector<PathPoint> currentPlan_;
        nav_msgs::msg::OccupancyGrid currentCostmap_;
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navGoalHandle_;
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr followGoalHandle_;
        std::atomic<bool> allDone_{false};
    };

    go2_motion_stand_up();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    auto node = std::make_shared<NavNode>(waypoints);
    node->init();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    node->startNav();

    std::cout << "[NAV] 导航进行中..." << std::endl;

    while (rclcpp::ok() && !node->isAllDone() && g_automationRunning) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    node->stopNav();
    go2_motion_stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "[NAV] 导航阶段完成!" << std::endl;
    return node->isAllDone();
}

// ==========================================================================
// 阶段 5: 上下楼梯
// ==========================================================================

/**
 * @brief 阶段5：上下楼梯
 */
static bool stageWalkStair()
{
    std::cout << "\n+================================================+\n";
    std::cout <<   "|  阶段 5: 上下楼梯                               |\n";
    std::cout <<   "+================================================+\n";

    enum class StairState { INIT, GOING_UP, ROTATING, GOING_DOWN, DONE };

    class StairNode : public rclcpp::Node {
    public:
        StairNode() : Node("_stage5_stair") {}
        void init() {
            timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&StairNode::loop, this));
        }
        bool isDone() const { return done_; }
        void stop() { running_ = false; }
    private:
        void loop() {
            if (!running_) return;
            switch (state_) {
            case StairState::INIT:
                go2_motion_stand_up();
                std::cout << "[STAIR] 站立中..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(2));
                go2_motion_static_walk();
                std::cout << "[STAIR] 爬楼梯模式已启用" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                go2_motion_move(UPSTAIR_VEL, 0.0f, 0.0f);
                std::cout << "[STAIR] 开始上楼梯" << std::endl;
                state_ = StairState::GOING_UP;
                break;
            case StairState::GOING_UP: {
                float roll=0, pitch=0, yaw=0;
                if (!go2_motion_get_imu(&roll, &pitch, &yaw)) return;
                bool isFlat = (std::fabs(roll) < FLAT_ROLL_THRESHOLD && std::fabs(pitch) < FLAT_PITCH_THRESHOLD);
                if (!isFlat) { wasClimbing_ = true; stableCnt_ = 0; }
                if (isFlat && wasClimbing_) {
                    stableCnt_++;
                    if (stableCnt_ >= STABLE_FRAMES_REQUIRED) {
                        go2_motion_stop();
                        std::cout << "[STAIR] 到达平地! 准备旋转45°" << std::endl;
                        state_ = StairState::ROTATING;
                        rotStart_ = this->now();
                    }
                }
                break;
            }
            case StairState::ROTATING: {
                double elapsed = (this->now() - rotStart_).seconds();
                double dur = STAIR_ROTATE_ANGLE / STAIR_ROTATE_SPEED;
                if (elapsed < dur) { go2_motion_move(0,0,STAIR_ROTATE_SPEED); }
                else {
                    go2_motion_stop();
                    std::cout << "[STAIR] 旋转完成，开始下楼梯" << std::endl;
                    go2_motion_move(DOWNSTAIR_VEL, 0, 0);
                    state_ = StairState::GOING_DOWN;
                    downStart_ = this->now();
                }
                break;
            }
            case StairState::GOING_DOWN: {
                double elapsed = (this->now() - downStart_).seconds();
                if (DOWNSTAIR_VEL * elapsed < DOWNSTAIR_DIST) break;
                go2_motion_stop();
                std::cout << "[STAIR] 下楼梯完成" << std::endl;
                go2_motion_free_walk();
                std::cout << "[STAIR] 已切换行走模式" << std::endl;
                state_ = StairState::DONE;
                done_ = true;
                break;
            }
            case StairState::DONE: break;
            }
        }
        StairState state_ = StairState::INIT;
        bool wasClimbing_ = false;
        int stableCnt_ = 0;
        rclcpp::Time rotStart_, downStart_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::atomic<bool> running_{true};
        std::atomic<bool> done_{false};
    };

    auto node = std::make_shared<StairNode>();
    node->init();

    while (rclcpp::ok() && !node->isDone() && g_automationRunning) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    node->stop();
    go2_motion_stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "[STAIR] 上下楼梯完成!" << std::endl;
    return node->isDone();
}

// ==========================================================================
// 阶段 6: 巡线行走直到红色区域
// ==========================================================================

/**
 * @brief 阶段6：巡线行走直到检测到红色区域
 * @param rsPipe RealSense pipeline 引用
 */
static bool stageWalkingUntilColor(rs2::pipeline& rsPipe, TargetColor color,
                                    unitree::robot::go2::SportClient& sportClient)
{
    const char* colorName = (color == TargetColor::RED) ? "红色" : "蓝色";
    std::cout << "\n+================================================+\n";
    std::cout <<   "|  巡线行走至" << colorName << "区域                              |\n";
    std::cout <<   "+================================================+\n";

    bool colorFound = false;
    int frameCount = 0;
    int consecutiveColorFrames = 0;
    constexpr int COLOR_STABLE_FRAMES = 10;

    while (!colorFound) {
        rs2::frameset frames;
        try { frames = rsPipe.wait_for_frames(100); }
        catch (...) { continue; }
        rs2::frame colorFrame = frames.get_color_frame();
        if (!colorFrame) continue;

        cv::Mat bgrFrame(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3,
                         const_cast<void*>(colorFrame.get_data()), cv::Mat::AUTO_STEP);
        ++frameCount;

        cv::Mat binary = vChannelInverseBinary(bgrFrame, WALK_BINARY_THRESH);
        cv::Mat binaryMorphed = applyMorphology(binary, WALK_ERODE_ITER, WALK_DILATE_ITER, WALK_MORPH_KERNEL);

        int splitRow = WALK_SPLIT_ROW;
        cv::Rect lowerROI(0, splitRow, IMG_WIDTH, IMG_HEIGHT - splitRow);
        cv::Rect upperROI(0, 0, IMG_WIDTH, splitRow);
        cv::Mat lowerBinary = binaryMorphed(lowerROI);
        cv::Mat upperBinary = binaryMorphed(upperROI);

        float lowerMidX = findColumnProjectionCenter(lowerBinary);
        float errorPx = 0.0f;

        if (lowerMidX >= 0) {
            errorPx  = lowerMidX - (IMG_WIDTH / 2.0f);
            WalkTurnType turnType = analyzeUpperHalf(upperBinary, lowerMidX,
                WALK_CURVE_THRESH, WALK_SHARP_THRESH, WALK_UPPER_STRIPS);

            float vx = WALK_FORWARD_SPEED;
            float vyaw = std::clamp(WALK_ANGULAR_KP * errorPx, -WALK_MAX_ANGULAR * 0.3f, WALK_MAX_ANGULAR * 0.3f);

            switch (turnType) {
            case WalkTurnType::RIGHT_RIGHT_ANGLE:
            case WalkTurnType::LEFT_RIGHT_ANGLE: {
                sportClient.StopMove();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                float dir = (turnType == WalkTurnType::LEFT_RIGHT_ANGLE) ? 1.0f : -1.0f;
                auto tStart = std::chrono::steady_clock::now();
                while (std::chrono::duration<float>(std::chrono::steady_clock::now() - tStart).count() < RIGHT_ANGLE_DURATION) {
                    sportClient.Move(0.0f, 0.0f, dir * RIGHT_ANGLE_SPEED);
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                sportClient.StopMove();
                vx = 0.0f; vyaw = 0.0f;
                break;
            }
            default: break;
            }
            if (vx > 0 || vyaw != 0) sportClient.Move(vx, 0.0f, vyaw);
        }

        float colorRatio = detectColorAreaRatio(bgrFrame, color);
        if (colorRatio >= COLOR_AREA_RATIO) {
            consecutiveColorFrames++;
            if (consecutiveColorFrames >= COLOR_STABLE_FRAMES) {
                sportClient.StopMove();
                std::cout << "[COLOR] 检测到" << colorName << "区域! (占比 "
                          << (colorRatio * 100) << "%)" << std::endl;
                colorFound = true;
            }
        } else {
            consecutiveColorFrames = 0;
        }

        cv::Mat display = bgrFrame.clone();
        cv::line(display, cv::Point(0, splitRow), cv::Point(display.cols - 1, splitRow),
                 cv::Scalar(255, 255, 0), 2);
        std::string status = "Searching " + std::string(colorName)
                           + " [" + std::to_string(consecutiveColorFrames) + "/" + std::to_string(COLOR_STABLE_FRAMES) + "]"
                           + " ratio=" + std::to_string(colorRatio).substr(0, 5);
        cv::putText(display, status, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Walking to " + std::string(colorName), display);
        cv::imshow("Binary", binaryMorphed);

        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) {
            sportClient.StopMove();
            return false;
        }
    }

    cv::destroyWindow("Walking to " + std::string(colorName));
    cv::destroyWindow("Binary");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true;
}

// ==========================================================================
// 阶段 7: YOLO 动作识别
// ==========================================================================

/**
 * @brief 阶段7：YOLO 动作识别与执行
 *
 * 使用 Go2Action 封装类进行摄像头采集、YOLO 检测与动作分发。
 * 检测到有效目标并执行动作后自动退出。
 */
static bool stageAction()
{
    std::cout << "\n+================================================+\n";
    std::cout <<   "|  阶段 7: YOLO 动作识别与执行                    |\n";
    std::cout <<   "+================================================+\n";

    static std::vector<std::string> classNames = {"stretch", "hello", "light", "one", "two"};
    Go2Action action(YOLO_MODEL, classNames, YOLO_CONF_THRESHOLD, cv::Size(YOLO_INPUT_SIZE, YOLO_INPUT_SIZE));

    if (!action.Initialize()) {
        std::cerr << "[ACTION] Go2Action 初始化失败" << std::endl;
        return false;
    }

    std::cout << "[ACTION] YOLO 动作识别开始，等待识别目标..." << std::endl;

    bool actionCompleted = false;
    bool actionDetected = false;
    int frameCount = 0;
    int noDetectionFrames = 0;
    constexpr int MAX_NO_DETECTION = 300;
    constexpr int MAX_TOTAL_FRAMES = 600;

    cv::namedWindow("Stage7: Action", cv::WINDOW_NORMAL);
    cv::resizeWindow("Stage7: Action", 960, 720);

    // 创建独立的 YOLO 检测器用于获取检测结果
    YOLODetector detector(YOLO_MODEL, classNames, cv::Size(YOLO_INPUT_SIZE, YOLO_INPUT_SIZE));
    if (!detector.initialize()) {
        std::cerr << "[ACTION] YOLO 检测器初始化失败" << std::endl;
        cv::destroyWindow("Stage7: Action");
        return false;
    }

    while (!actionCompleted) {
        cv::Mat frame = action.GetFrame();
        if (frame.empty()) {
            if ((char)cv::waitKey(1) == 'q') break;
            continue;
        }
        ++frameCount;

        auto detections = detector.detect(frame, YOLO_CONF_THRESHOLD, YOLO_NMS_THRESHOLD);

        if (!detections.empty()) {
            noDetectionFrames = 0;
            std::cout << "[ACTION] Frame " << frameCount << " 检测到: ";
            for (size_t i = 0; i < detections.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << detections[i].class_name;
            }
            std::cout << std::endl;
            action.Dispatch(detections);
            actionDetected = true;
            actionCompleted = true;
        } else {
            noDetectionFrames++;
        }

        YOLODetector::drawDetections(frame, detections, true);
        std::string info = "Action Detection - Frame: " + std::to_string(frameCount);
        if (actionDetected) info += " [DETECTED]";
        cv::putText(frame, info, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        cv::imshow("Stage7: Action", frame);

        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) break;

        if (noDetectionFrames > MAX_NO_DETECTION) {
            std::cout << "[ACTION] 超时未检测到目标，跳过此阶段" << std::endl;
            break;
        }
        if (frameCount > MAX_TOTAL_FRAMES) {
            std::cout << "[ACTION] 达到最大帧数限制，跳过此阶段" << std::endl;
            break;
        }
    }

    action.WaitDone();
    cv::destroyWindow("Stage7: Action");
    std::cout << "[ACTION] 动作识别阶段完成!" << std::endl;
    return actionCompleted;
}

// ==========================================================================
// 阶段 8: 逆时针旋转 45°
// ==========================================================================

static bool stageRotateCCW45()
{
    std::cout << "\n+================================================+\n";
    std::cout <<   "|  阶段 8: 逆时针旋转 45°                        |\n";
    std::cout <<   "+================================================+\n";
    rotate(ROTATE_ANGLE_45);
    return true;
}

// ==========================================================================
// 阶段 9: 巡线行走直到蓝色区域，左转45°停止
// ==========================================================================

/**
 * @brief 阶段9：巡线至蓝色区域停止并左转45°
 * @param rsPipe RealSense pipeline 引用
 */
static bool stageWalkingUntilBlueAndTurn(rs2::pipeline& rsPipe)
{
    std::cout << "\n+================================================+\n";
    std::cout <<   "|  阶段 9: 巡线至蓝色区域 → 左转45°停止           |\n";
    std::cout <<   "+================================================+\n";

    unitree::robot::go2::SportClient sportClient;
    sportClient.SetTimeout(10.0f);
    sportClient.Init();

    // 先巡线走到蓝色区域
    bool foundBlue = stageWalkingUntilColor(rsPipe, TargetColor::BLUE, sportClient);
    if (!foundBlue) return false;

    // 停止并左转 45°
    std::cout << "[BLUE] 检测到蓝色区域，停止并左转45°" << std::endl;
    sportClient.StopMove();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 左转 = 逆时针 = 正角速度
    rotate(ROTATE_ANGLE_45);

    std::cout << "[BLUE] 程序结束! 机器人已停在蓝色区域并左转45°" << std::endl;
    return true;
}

// ==========================================================================
// 主函数
// ==========================================================================

/**
 * @brief Go2 全流程自动化程序入口
 * @param argc 参数个数
 * @param argv 参数列表，argv[1] 为网络接口名称（可选，默认 eth0）
 * @return int 0 正常退出，-1 异常退出
 */
int main(int argc, char** argv)
{
    // ---- 解析命令行参数 ----
    std::string netInterface = NET_INTERFACE;
    if (argc > 1) {
        netInterface = argv[1];
    }
    std::cout << "网络接口: " << netInterface << std::endl;
    std::cout << "move1 (避障后步数): " << MOVE1_STEPS << std::endl;
    std::cout << "move2 (巡线步数):   " << MOVE2_STEPS << std::endl;
    std::cout << "导航文件: " << ROUTE_FILE << std::endl;

    // ======================================================================
    // 全局初始化
    // ======================================================================
    std::cout << "\n========== 全局初始化 ==========" << std::endl;

    // 1. 初始化 DDS 通信通道
    std::cout << "[INIT] 初始化 DDS 通道..." << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);
    std::cout << "[INIT] DDS 通道已初始化" << std::endl;

    // 2. 初始化运动桥接
    std::cout << "[INIT] 初始化 Go2 运动桥接..." << std::endl;
    if (!go2_motion_init(netInterface.c_str())) {
        std::cerr << "[ERROR] 运动桥接初始化失败!" << std::endl;
        return -1;
    }
    std::cout << "[INIT] 运动桥接就绪" << std::endl;

    // 3. 初始化 ROS2
    std::cout << "[INIT] 初始化 ROS2..." << std::endl;
    rclcpp::init(argc, argv);
    std::cout << "[INIT] ROS2 就绪" << std::endl;

    // 4. 初始化 RealSense 相机
    std::cout << "[INIT] 初始化 RealSense 相机..." << std::endl;
    rs2::pipeline rsPipe;
    rs2::config rsCfg;
    rsCfg.enable_stream(RS2_STREAM_COLOR, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_BGR8, IMG_FPS);
    try {
        rsPipe.start(rsCfg);
        std::cout << "[INIT] RealSense 就绪" << std::endl;
    } catch (const rs2::error& e) {
        std::cerr << "[ERROR] RealSense 启动失败: " << e.what() << std::endl;
        rclcpp::shutdown();
        return -1;
    }

    // ======================================================================
    // 自动化任务序列
    // ======================================================================
    std::cout << "\n========== 自动化任务开始 ==========" << std::endl;
    std::cout << "流程: 避障关闭→跳跃→巡线→导航→楼梯→红色→动作→旋转→蓝色→结束\n" << std::endl;

    bool success = true;

    // ---- 阶段 1: 关闭避障并前进 move1 步 ----
    if (success) {
        success = stageCloseObstaclesAndMove(MOVE1_STEPS);
    }

    // ---- 阶段 2: PCL 横棒检测 + 前跳 ----
    if (success) {
        success = stageStickJump();
    }

    // ---- 阶段 3: 巡线前进 move2 步 ----
    if (success) {
        success = stageWalkingForSteps(rsPipe, MOVE2_STEPS);
    }

    // ---- 阶段 4: Nav2 自主导航 ----
    if (success) {
        success = stageNavigation();
    }

    // ---- 阶段 5: 上下楼梯 ----
    if (success) {
        success = stageWalkStair();
    }

    // ---- 阶段 6: 巡线行走至红色区域 + 逆时针旋转45° ----
    if (success) {
        // 初始化 SportClient 用于巡线阶段
        unitree::robot::go2::SportClient walkSportClient;
        walkSportClient.SetTimeout(10.0f);
        walkSportClient.Init();

        success = stageWalkingUntilColor(rsPipe, TargetColor::RED, walkSportClient);
        if (success) {
            walkSportClient.StopMove();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "[RED] 检测到红色区域，逆时针旋转45°" << std::endl;
            rotate(ROTATE_ANGLE_45);
        }
    }

    // ---- 阶段 7: YOLO 动作识别与执行 ----
    if (success) {
        success = stageAction();
    }

    // ---- 阶段 8: 逆时针旋转45° ----
    if (success) {
        success = stageRotateCCW45();
    }

    // ---- 阶段 9: 巡线至蓝色区域 + 左转45°停止 ----
    if (success) {
        success = stageWalkingUntilBlueAndTurn(rsPipe);
    }

    // ======================================================================
    // 清理
    // ======================================================================
    std::cout << "\n========== 清理 ==========" << std::endl;

    go2_motion_stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    go2_motion_stand_down();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    try { rsPipe.stop(); } catch (...) {}
    cv::destroyAllWindows();

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "自动化任务 " << (success ? "全部完成!" : "部分完成(有阶段失败)") << std::endl;
    std::cout << "========================================" << std::endl;

    return success ? 0 : -1;
}
