/**
 * @file go2_motion_bridge.cpp
 * @brief Go2 运动控制与传感器数据桥接实现，封装 Unitree SDK2 的 DDS 通信
 *
 * @par 编译说明
 *       本文件不得包含任何 ROS2 头文件，仅链接 Unitree SDK2 DDS 库，
 *       以避免与 ROS2 版本的 CycloneDDS 头文件产生冲突。
 */
#include "go2_motion_bridge.hpp"

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

#include <mutex>
#include <cstring>
#include <thread>
#include <chrono>

#define TOPIC_HIGHSTATE      "rt/sportmodestate"
#define TOPIC_LIDAR_CLOUD    "rt/utlidar/cloud"

/// @brief 全局互斥锁
static std::mutex g_mutex;

/// @brief 运动客户端
static unitree::robot::go2::SportClient* g_sportClient = nullptr;

/// @brief 最新运动状态
static unitree_go::msg::dds_::SportModeState_ g_latestState;

/// @brief 最新点云原始数据字节
static std::vector<uint8_t> g_latestCloudData;
static uint32_t g_cloudWidth  = 0;
static uint32_t g_cloudHeight = 0;
static uint32_t g_cloudStep   = 0;

/// @brief 初始位姿偏移
static float g_px0  = 0.0f;
static float g_py0  = 0.0f;
static float g_yaw0 = 0.0f;

/// @brief DDS 通道订阅器
static unitree::robot::ChannelSubscriberPtr<
    sensor_msgs::msg::dds_::PointCloud2_> g_lidarSuber;
static unitree::robot::ChannelSubscriberPtr<
    unitree_go::msg::dds_::SportModeState_> g_stateSuber;

/**
 * @brief DDS 雷达点云回调
 * @param message DDS PointCloud2_ 消息指针
 */
static void lidarHandler(const void* message)
{
    auto ddsCloud = static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(message);
    if (!ddsCloud) return;

    std::lock_guard<std::mutex> lock(g_mutex);
    g_latestCloudData.assign(
        ddsCloud->data().begin(), ddsCloud->data().end());
    g_cloudWidth  = ddsCloud->width();
    g_cloudHeight = ddsCloud->height();
    g_cloudStep   = ddsCloud->point_step();
}

/**
 * @brief DDS 运动状态回调
 * @param message DDS SportModeState_ 消息指针
 */
static void stateHandler(const void* message)
{
    auto ddsState = static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);
    if (!ddsState) return;

    std::lock_guard<std::mutex> lock(g_mutex);
    g_latestState = *ddsState;
}

bool go2_motion_init(const char* netInterface)
{
    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);

        g_sportClient = new unitree::robot::go2::SportClient();
        g_sportClient->SetTimeout(10.0f);
        g_sportClient->Init();

        g_lidarSuber.reset(new unitree::robot::ChannelSubscriber<
            sensor_msgs::msg::dds_::PointCloud2_>(TOPIC_LIDAR_CLOUD));
        g_lidarSuber->InitChannel(lidarHandler, 1);

        g_stateSuber.reset(new unitree::robot::ChannelSubscriber<
            unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        g_stateSuber->InitChannel(stateHandler, 1);

        // 等待初始状态稳定
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_px0  = g_latestState.position()[0];
            g_py0  = g_latestState.position()[1];
            g_yaw0 = g_latestState.imu_state().rpy()[2];
        }

        return true;
    } catch (...) {
        return false;
    }
}

void go2_motion_stand_up()
{
    if (g_sportClient) g_sportClient->StandUp();
}

void go2_motion_stand_down()
{
    if (g_sportClient) g_sportClient->StandDown();
}

void go2_motion_stop()
{
    if (g_sportClient) g_sportClient->StopMove();
}

void go2_motion_move(float vx, float vy, float vyaw)
{
    if (g_sportClient) g_sportClient->Move(vx, vy, vyaw);
}

bool go2_motion_get_odom(float* x, float* y, float* yaw, float* vx, float* vyaw)
{
    if (!x || !y || !yaw || !vx || !vyaw) return false;

    std::lock_guard<std::mutex> lock(g_mutex);
    *x    = g_latestState.position()[0] - g_px0;
    *y    = g_latestState.position()[1] - g_py0;
    *yaw  = g_latestState.imu_state().rpy()[2] - g_yaw0;
    *vx   = g_latestState.velocity()[0];
    *vyaw = g_latestState.yaw_speed();
    return true;
}

bool go2_motion_get_lidar(float* dataOut, uint32_t maxPoints,
                          uint32_t* pointCountOut,
                          uint32_t* widthOut, uint32_t* heightOut)
{
    if (!dataOut || !pointCountOut || !widthOut || !heightOut) return false;

    std::lock_guard<std::mutex> lock(g_mutex);

    uint32_t totalPoints = g_cloudWidth * g_cloudHeight;
    uint32_t copyPoints  = (totalPoints < maxPoints) ? totalPoints : maxPoints;

    if (copyPoints == 0) {
        *pointCountOut = 0;
        *widthOut  = 0;
        *heightOut = 0;
        return false;
    }

    uint32_t ptStep = (g_cloudStep > 0) ? g_cloudStep : 16;
    for (uint32_t i = 0; i < copyPoints; ++i) {
        uint32_t srcBase = i * ptStep;
        float* dst       = dataOut + i * 4; // 4 floats per point
        if (srcBase + ptStep <= g_latestCloudData.size()) {
            std::memcpy(&dst[0], &g_latestCloudData[srcBase], ptStep > 16 ? 16 : ptStep);
        }
    }

    *pointCountOut = copyPoints;
    *widthOut  = g_cloudWidth;
    *heightOut = g_cloudHeight;
    return true;
}
