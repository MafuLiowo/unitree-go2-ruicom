/**
 * @file NavFnPlanner.hpp
 * @brief NavFn 全局规划器，基于导航函数（Navigation Function）的势场传播算法
 *
 * @par 功能说明
 *       使用 Dijkstra 风格的波前传播从目标点向外扩散势场值，
 *       再通过梯度下降法从起始点回溯提取最优路径。
 *       支持 8 连通邻域传播和三次样条插值路径平滑。
 */
#pragma once

#include <vector>
#include <cmath>
#include <queue>
#include <limits>
#include <algorithm>
#include <cstdint>
#include <cstring>

/**
 * @brief 2D 位姿点
 */
struct Pose2D
{
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
};

/**
 * @brief 路径点，继承自 Pose2D 并增加方向向量分量
 */
struct PathPoint
{
    float x  = 0.0f;
    float y  = 0.0f;
    float dx = 0.0f;
    float dy = 0.0f;
};

/**
 * @brief NavFn 全局规划器类
 *
 * 基于导航函数（Navigation Function）的全局路径规划器，
 * 采用 8 连通邻域的波前传播（Dijkstra） + 梯度下降路径提取。
 */
class NavFnPlanner
{
public:
    /**
     * @brief 默认构造函数
     */
    NavFnPlanner();

    /**
     * @brief 析构函数
     */
    ~NavFnPlanner();

    /**
     * @brief 设置规划器参数
     * @param cellCostMin 单元格最小代价（大于 0）
     * @param cellCostMax 单元格最大代价，对应障碍物
     * @param obsCostThreshold 障碍物代价阈值，大于此值视为障碍
     */
    void setParams(float cellCostMin = 1.0f, float cellCostMax = 1000.0f, float obsCostThreshold = 100.0f);

    /**
     * @brief 初始化势场并从占用栅格地图构建代价数组
     * @param mapData 占用栅格数据（0-100：空闲，-1：未知），按行优先存储
     * @param width 地图宽度（栅格数）
     * @param height 地图高度（栅格数）
     * @param resolution 地图分辨率 (m/pixel)
     * @param originX 地图原点 x 坐标 (m)
     * @param originY 地图原点 y 坐标 (m)
     */
    void setCostmap(const int8_t* mapData, int width, int height,
                    float resolution, float originX, float originY);

    /**
     * @brief 计算从起点到目标点的全局路径
     * @param startX 起点 x 坐标 (m)
     * @param startY 起点 y 坐标 (m)
     * @param goalX 目标 x 坐标 (m)
     * @param goalY 目标 y 坐标 (m)
     * @param path 输出参数：规划出的路径点数组（含方向向量）
     * @return true 规划成功
     */
    bool planPath(float startX, float startY, float goalX, float goalY,
                  std::vector<PathPoint>& path);

    /**
     * @brief 获取世界坐标到栅格坐标的映射
     * @param wx 世界 x 坐标 (m)
     * @param wy 世界 y 坐标 (m)
     * @param mx 输出参数：栅格 x 坐标
     * @param my 输出参数：栅格 y 坐标
     * @return true 坐标在地图范围内
     */
    bool worldToMap(float wx, float wy, int& mx, int& my) const;

    /**
     * @brief 获取栅格坐标到世界坐标的映射
     * @param mx 栅格 x 坐标
     * @param my 栅格 y 坐标
     * @param wx 输出参数：世界 x 坐标 (m)
     * @param wy 输出参数：世界 y 坐标 (m)
     */
    void mapToWorld(int mx, int my, float& wx, float& wy) const;

private:
    /// @brief 邻居偏移索引（8 连通）
    static constexpr int NB_OFFSET_COUNT = 8;
    static constexpr int NB_OFFSET_X[NB_OFFSET_COUNT] = {-1, -1, 0, 1, 1,  1,  0, -1};
    static constexpr int NB_OFFSET_Y[NB_OFFSET_COUNT] = { 0,  1, 1, 1, 0, -1, -1, -1};

    /// @brief 势场值的无穷大常量
    static constexpr float POT_HIGH = 1.0e10f;

    /** 势场传播用的优先队列元素 */
    struct PotCell
    {
        int   mx;
        int   my;
        float pot;

        bool operator>(const PotCell& other) const
        {
            return pot > other.pot;
        }
    };

    /**
     * @brief 获取代价数组中指定栅格的代价值
     * @param mx 栅格 x 坐标
     * @param my 栅格 y 坐标
     * @return float 代价值
     */
    float costAt(int mx, int my) const;

    /**
     * @brief 获取势场数组中指定栅格的势场值
     * @param mx 栅格 x 坐标
     * @param my 栅格 y 坐标
     * @return float 势场值
     */
    float potAt(int mx, int my) const;

    /**
     * @brief 设置势场数组中指定栅格的势场值
     * @param mx 栅格 x 坐标
     * @param my 栅格 y 坐标
     * @param val 势场值
     */
    void potSet(int mx, int my, float val);

    /**
     * @brief 更新势场值（若新值更小）
     * @param mx 栅格 x 坐标
     * @param my 栅格 y 坐标
     * @param val 新势场值
     * @return true 值已更新
     */
    bool potUpdate(int mx, int my, float val);

    /**
     * @brief 从目标点进行波前势场传播
     * @param goalMx 目标栅格 x 坐标
     * @param goalMy 目标栅格 y 坐标
     * @param startMx 起点栅格 x 坐标
     * @param startMy 起点栅格 y 坐标
     * @return true 传播成功到达起点
     */
    bool propagatePotential(int goalMx, int goalMy, int startMx, int startMy);

    /**
     * @brief 通过梯度下降从起点回溯到目标点提取路径
     * @param startMx 起点栅格 x 坐标
     * @param startMy 起点栅格 y 坐标
     * @param goalMx 目标栅格 x 坐标
     * @param goalMy 目标栅格 y 坐标
     * @param path 输出参数：栅格坐标路径
     */
    void tracePath(int startMx, int startMy, int goalMx, int goalMy,
                   std::vector<std::pair<int, int>>& path);

    /**
     * @brief 平滑栅格坐标路径并转换为世界坐标
     * @param rawPath 输入的栅格坐标路径
     * @param smoothedPath 输出参数：平滑后的世界坐标路径点
     */
    void smoothPath(const std::vector<std::pair<int, int>>& rawPath,
                    std::vector<PathPoint>& smoothedPath);

    float cellCostMin_       = 1.0f;
    float cellCostMax_       = 1000.0f;
    float obsCostThreshold_  = 100.0f;

    int   mapWidth_    = 0;
    int   mapHeight_   = 0;
    float resolution_  = 0.05f;
    float originX_     = 0.0f;
    float originY_     = 0.0f;

    std::vector<float> costArr_;   ///< 代价数组，行优先
    std::vector<float> potArr_;    ///< 势场数组，行优先
    bool initialized_ = false;
};
