/**
 * @file go2_motion_bridge.hpp
 * @brief Go2 运动控制与传感器数据桥接接口，提供纯 C 风格 API，封装 Unitree SDK2 的 DDS 通信
 *
 * @par 使用说明
 *       本接口用于在编译时不引入 Unitree SDK2 DDS 头文件的情况下，
 *       从 ROS2 节点中访问 Go2 的运动控制和传感器数据。
 *       实际实现位于 go2_motion_bridge.cpp，该文件单独编译且仅链接 Unitree SDK2。
 */
#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 Go2 运动桥接模块
 * @param netInterface 网络接口名称（如 "eth0"）
 * @return true 初始化成功
 */
bool go2_motion_init(const char* netInterface);

/**
 * @brief 让机器人站立
 */
void go2_motion_stand_up();

/**
 * @brief 让机器人趴下
 */
void go2_motion_stand_down();

/**
 * @brief 停止机器人运动
 */
void go2_motion_stop();

/**
 * @brief 驱动机器人以指定速度移动
 * @param vx 前进速度 (m/s)
 * @param vy 侧向速度 (m/s)
 * @param vyaw 旋转角速度 (rad/s)
 */
void go2_motion_move(float vx, float vy, float vyaw);

/**
 * @brief 获取最新里程计数据（线程安全）
 * @param x 输出参数：x 坐标 (m)
 * @param y 输出参数：y 坐标 (m)
 * @param yaw 输出参数：偏航角 (rad)
 * @param vx 输出参数：前进速度 (m/s)
 * @param vyaw 输出参数：旋转角速度 (rad/s)
 * @return true 获取成功
 */
bool go2_motion_get_odom(float* x, float* y, float* yaw, float* vx, float* vyaw);

/**
 * @brief 获取最新 LiDAR 点云数据（线程安全）
 *
 * 点云格式：每个点 16 字节，布局为：
 *   offset 0: x (float)
 *   offset 4: y (float)
 *   offset 8: z (float)
 *   offset 12: intensity (float)
 *
 * @param dataOut 输出缓冲区（调用者分配，至少 maxPoints * 16 字节）
 * @param maxPoints 缓冲区能容纳的最大点数
 * @param pointCountOut 输出参数：实际点数
 * @param widthOut 输出参数：点云宽度
 * @param heightOut 输出参数：点云高度
 * @return true 获取成功
 */
bool go2_motion_get_lidar(float* dataOut, uint32_t maxPoints,
                          uint32_t* pointCountOut,
                          uint32_t* widthOut, uint32_t* heightOut);

#ifdef __cplusplus
}
#endif
