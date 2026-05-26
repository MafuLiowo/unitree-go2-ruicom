/**
 * @file DWBPlanner.hpp
 * @brief DWB（Dynamic Window Based）局部规划器，基于速度采样与轨迹评分的局部避障算法
 *
 * @par 功能说明
 *       在速度空间内采样多组 (vx, vyaw) 速度对，
 *       模拟各速度对对应的运动轨迹，并基于代价函数综合评分，
 *       选择最优速度指令输出。
 */
#pragma once

#include "NavFnPlanner.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>

/**
 * @brief 足迹多边形顶点（相对于机器人中心）
 */
struct FootprintPoint
{
    float x = 0.4f;
    float y = 0.2f;
    float z = 0.2f;
};

/**
 * @brief DWB 局部规划器配置参数
 */
struct DWBConfig
{
    float maxLinearVel    = 0.30f;  ///< 最大线速度 (m/s)
    float minLinearVel    = 0.0f;   ///< 最小线速度 (m/s)
    float maxAngularVel   = 1.0f;   ///< 最大角速度 (rad/s)
    float minAngularVel   = -1.0f;  ///< 最负角速度 (rad/s)
    float linearStep      = 0.05f;  ///< 线速度采样步长 (m/s)
    float angularStep     = 0.2f;   ///< 角速度采样步长 (rad/s)
    float simTime         = 1.5f;   ///< 前向模拟时间 (s)
    float simDt           = 0.1f;   ///< 模拟时间步长 (s)
    float goalWeight      = 2.0f;   ///< 目标趋近权重
    float pathAlignWeight = 1.5f;   ///< 路径对齐权重
    float obstacleWeight  = 3.0f;   ///< 障碍物避让权重
    float speedWeight     = 0.5f;   ///< 速度偏好权重
    float robotRadius     = 0.35f;  ///< 机器人碰撞半径 (m)，footprint 为空时作为圆形足迹半径
    std::vector<FootprintPoint> footprint; ///< 机器人足迹多边形（相对于baselink中心，逆时针），非空时取代 robotRadius
    float goalTolerance   = 0.2f;   ///< 目标到达容差 (m)
    int   lookaheadIndex  = 5;      ///< 路径前瞻索引偏移
};

/**
 * @brief DWB 局部规划器类
 *
 * 基于动态窗口法（DWB）的局部路径规划器，
 * 在速度空间采样并评分轨迹以选择最优 cmd_vel。
 */
class DWBPlanner
{
public:
    /**
     * @brief 默认构造函数
     */
    DWBPlanner();

    /**
     * @brief 带配置的构造函数
     * @param config DWB 配置参数
     */
    explicit DWBPlanner(const DWBConfig& config);

    /**
     * @brief 析构函数
     */
    ~DWBPlanner();

    /**
     * @brief 设置配置参数
     * @param config DWB 配置参数
     */
    void setConfig(const DWBConfig& config);

    /**
     * @brief 设置机器人足迹多边形
     * @param footprint 多边形顶点（相对于 baselink 中心，逆时针顺序）
     */
    void setFootprint(const std::vector<FootprintPoint>& footprint);

    /**
     * @brief 计算最优速度指令
     * @param currentPose 当前机器人位姿
     * @param currentVx 当前线速度 (m/s)
     * @param currentVyaw 当前角速度 (rad/s)
     * @param globalPath 全局规划路径
     * @param costmapData 局部代价地图数据（0-100：空闲，-1：未知），按行优先
     * @param costmapWidth 局部代价地图宽度
     * @param costmapHeight 局部代价地图高度
     * @param costmapResolution 代价地图分辨率 (m/pixel)
     * @param costmapOriginX 代价地图原点 x (m)
     * @param costmapOriginY 代价地图原点 y (m)
     * @param cmdVx 输出参数：最优线速度 (m/s)
     * @param cmdVyaw 输出参数：最优角速度 (rad/s)
     * @return true 计算成功
     */
    bool computeVelocity(const Pose2D& currentPose,
                         float currentVx, float currentVyaw,
                         const std::vector<PathPoint>& globalPath,
                         const int8_t* costmapData,
                         int costmapWidth, int costmapHeight,
                         float costmapResolution,
                         float costmapOriginX, float costmapOriginY,
                         float& cmdVx, float& cmdVyaw);

    /**
     * @brief 判断是否已到达目标
     * @param currentPose 当前位姿
     * @param globalPath 全局路径
     * @return true 已到达
     */
    bool isGoalReached(const Pose2D& currentPose,
                       const std::vector<PathPoint>& globalPath) const;

    /**
     * @brief 获取目标在全局路径中的索引
     * @param globalPath 全局路径
     * @param pose 需要判定的位姿
     * @return int 路径中最近点的索引
     */
    int findClosestPathIndex(const std::vector<PathPoint>& globalPath,
                             const Pose2D& pose) const;

private:
    /**
     * @brief 根据速度和当前位姿模拟前向轨迹
     * @param vx 线速度 (m/s)
     * @param vyaw 角速度 (rad/s)
     * @param startPose 起始位姿
     * @param simTime 模拟时长 (s)
     * @param simDt 模拟步长 (s)
     * @param trajectory 输出参数：模拟轨迹点
     */
    void simulateTrajectory(float vx, float vyaw, const Pose2D& startPose,
                            float simTime, float simDt,
                            std::vector<Pose2D>& trajectory) const;

    /**
     * @brief 对轨迹进行综合评分
     * @param trajectory 模拟轨迹
     * @param globalPath 全局路径
     * @param closestPathIdx 当前在路径上的最近索引
     * @param costmapData 代价地图数据
     * @param costmapWidth 代价地图宽度
     * @param costmapHeight 代价地图高度
     * @param costmapResolution 分辨率
     * @param costmapOriginX 原点 x
     * @param costmapOriginY 原点 y
     * @return float 综合评分（越高越好）
     */
    float scoreTrajectory(const std::vector<Pose2D>& trajectory,
                          const std::vector<PathPoint>& globalPath,
                          int closestPathIdx,
                          const int8_t* costmapData,
                          int costmapWidth, int costmapHeight,
                          float costmapResolution,
                          float costmapOriginX, float costmapOriginY) const;

    /**
     * @brief 将足迹多边形从机器人坐标系变换到世界坐标系
     * @param pose 机器人当前位姿
     * @param worldFootprint 输出参数：世界坐标系下的足迹顶点
     */
    void transformFootprintToWorld(const Pose2D& pose,
                                   std::vector<FootprintPoint>& worldFootprint) const;

    /**
     * @brief 计算机器人足迹到最近障碍物的距离（考虑足迹多边形）
     * @param pose 机器人位姿
     * @param costmapData 代价地图数据
     * @param costmapWidth 宽度
     * @param costmapHeight 高度
     * @param resolution 分辨率
     * @param originX 原点 x
     * @param originY 原点 y
     * @return float 最近距离 (m)，若无障碍物返回极大值
     */
    float footprintObstacleDistance(const Pose2D& pose,
                                    const int8_t* costmapData,
                                    int costmapWidth, int costmapHeight,
                                    float resolution,
                                    float originX, float originY) const;

    /**
     * @brief 计算单个点到最近障碍物的距离
     * @param wx 世界 x 坐标
     * @param wy 世界 y 坐标
     * @param costmapData 代价地图数据
     * @param costmapWidth 宽度
     * @param costmapHeight 高度
     * @param resolution 分辨率
     * @param originX 原点 x
     * @param originY 原点 y
     * @return float 最近距离 (m)，若无障碍物返回极大值
     */
    float pointObstacleDistance(float wx, float wy,
                                const int8_t* costmapData,
                                int costmapWidth, int costmapHeight,
                                float resolution,
                                float originX, float originY) const;

    DWBConfig config_;
};
