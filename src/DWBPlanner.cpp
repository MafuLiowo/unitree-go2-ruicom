/**
 * @file DWBPlanner.cpp
 * @brief DWB 局部规划器实现，包含速度采样、轨迹模拟和评分逻辑
 */
#include "DWBPlanner.hpp"

#include <cmath>

DWBPlanner::DWBPlanner() {}

DWBPlanner::DWBPlanner(const DWBConfig& config)
    : config_(config) {}

DWBPlanner::~DWBPlanner() {}

void DWBPlanner::setConfig(const DWBConfig& config)
{
    config_ = config;
    // 若未设置 footprint 但有 robotRadius，自动生成圆形足迹
    if (config_.footprint.empty() && config_.robotRadius > 0.0f) {
        const int N = 16; // 16 边形近似圆
        float r = config_.robotRadius;
        for (int i = 0; i < N; ++i) {
            float ang = 2.0f * M_PI * i / N;
            FootprintPoint pt;
            pt.x = r * std::cos(ang);
            pt.y = r * std::sin(ang);
            config_.footprint.push_back(pt);
        }
    }
}

void DWBPlanner::setFootprint(const std::vector<FootprintPoint>& footprint)
{
    config_.footprint = footprint;
}

void DWBPlanner::simulateTrajectory(float vx, float vyaw, const Pose2D& startPose,
                                    float simTime, float simDt,
                                    std::vector<Pose2D>& trajectory) const
{
    trajectory.clear();
    trajectory.push_back(startPose);

    float t = 0.0f;
    Pose2D current = startPose;

    while (t < simTime) {
        t += simDt;
        current.x   += vx * simDt * std::cos(current.yaw);
        current.y   += vx * simDt * std::sin(current.yaw);
        current.yaw += vyaw * simDt;
        trajectory.push_back(current);
    }
}

/**
 * @brief 将足迹多边形从机器人坐标系变换到世界坐标系
 * @param pose 机器人当前位姿
 * @param worldFootprint 输出参数：世界坐标系下的足迹顶点
 */
void DWBPlanner::transformFootprintToWorld(
    const Pose2D& pose,
    std::vector<FootprintPoint>& worldFootprint) const
{
    worldFootprint.clear();
    double cosYaw = std::cos(static_cast<double>(pose.yaw));
    double sinYaw = std::sin(static_cast<double>(pose.yaw));
    for (const auto& pt : config_.footprint) {
        FootprintPoint wpt;
        wpt.x = static_cast<float>(pose.x + pt.x * cosYaw - pt.y * sinYaw);
        wpt.y = static_cast<float>(pose.y + pt.x * sinYaw + pt.y * cosYaw);
        wpt.z = 0.0f;
        worldFootprint.push_back(wpt);
    }
}

/**
 * @brief 计算机器人足迹到最近障碍物的距离
 */
float DWBPlanner::footprintObstacleDistance(
    const Pose2D& pose,
    const int8_t* costmapData,
    int costmapWidth, int costmapHeight,
    float resolution,
    float originX, float originY) const
{
    if (config_.footprint.empty()) {
        // 退化情况：当作单点
        return pointObstacleDistance(pose.x, pose.y, costmapData,
                                     costmapWidth, costmapHeight,
                                     resolution, originX, originY);
    }

    std::vector<FootprintPoint> worldFootprint;
    transformFootprintToWorld(pose, worldFootprint);

    float minDist = 1.0e10f;
    for (const auto& wpt : worldFootprint) {
        float d = pointObstacleDistance(wpt.x, wpt.y, costmapData,
                                        costmapWidth, costmapHeight,
                                        resolution, originX, originY);
        if (d < minDist) minDist = d;
    }
    return minDist;
}

/**
 * @brief 计算单个点到最近障碍物的距离
 */
float DWBPlanner::pointObstacleDistance(float wx, float wy,
                                          const int8_t* costmapData,
                                          int costmapWidth, int costmapHeight,
                                          float resolution,
                                          float originX, float originY) const
{
    // 在机器人半径范围内搜索障碍物
    int searchRadius = static_cast<int>(std::ceil(config_.robotRadius / resolution)) + 1;

    // 世界坐标转为栅格坐标
    int cx = static_cast<int>(std::floor((wx - originX) / resolution));
    int cy = static_cast<int>(std::floor((wy - originY) / resolution));

    float minDist = 1.0e10f;
    bool found = false;

    for (int dy = -searchRadius; dy <= searchRadius; ++dy) {
        for (int dx = -searchRadius; dx <= searchRadius; ++dx) {
            int mx = cx + dx;
            int my = cy + dy;
            if (mx < 0 || mx >= costmapWidth || my < 0 || my >= costmapHeight) {
                continue;
            }
            int val = costmapData[my * costmapWidth + mx];
            if (val < 0) continue; // 未知
            if (val > 65) { // 障碍物
                float wx2 = originX + (static_cast<float>(mx) + 0.5f) * resolution;
                float wy2 = originY + (static_cast<float>(my) + 0.5f) * resolution;
                float dist = std::sqrt((wx - wx2) * (wx - wx2) + (wy - wy2) * (wy - wy2));
                if (dist < minDist) {
                    minDist = dist;
                    found = true;
                }
            }
        }
    }

    return found ? minDist : 1.0e10f;
}

float DWBPlanner::scoreTrajectory(const std::vector<Pose2D>& trajectory,
                                   const std::vector<PathPoint>& globalPath,
                                   int closestPathIdx,
                                   const int8_t* costmapData,
                                   int costmapWidth, int costmapHeight,
                                   float costmapResolution,
                                   float costmapOriginX, float costmapOriginY) const
{
    if (trajectory.empty() || globalPath.empty()) return -1.0f;

    size_t lookahead = std::min(static_cast<size_t>(closestPathIdx + config_.lookaheadIndex),
                                globalPath.size() - 1);

    const Pose2D& endPose = trajectory.back();
    const PathPoint& goalPoint = globalPath[lookahead];
    const PathPoint& targetPoint = globalPath.back();

    // 目标趋近代价：终点到前瞻路径点距离
    float dxGoal = goalPoint.x - endPose.x;
    float dyGoal = goalPoint.y - endPose.y;
    float goalDist = std::sqrt(dxGoal * dxGoal + dyGoal * dyGoal);

    // 目标是终点时直接计算
    float dxTarget = targetPoint.x - endPose.x;
    float dyTarget = targetPoint.y - endPose.y;
    float targetDist = std::sqrt(dxTarget * dxTarget + dyTarget * dyTarget);

    float goalScore = (1.0f / (1.0f + goalDist));
    float targetScore = (1.0f / (1.0f + targetDist));

    // 路径对齐代价：轨迹上各点到路径的最小距离
    float pathAlignCost = 0.0f;
    int sampleCount = 0;
    for (size_t i = 0; i < trajectory.size(); i += 2) { // 每2步采样
        const Pose2D& tp = trajectory[i];
        float minDistToPath = 1.0e10f;
        for (size_t j = 0; j < globalPath.size(); ++j) {
            float dx = globalPath[j].x - tp.x;
            float dy = globalPath[j].y - tp.y;
            float d2 = dx * dx + dy * dy;
            if (d2 < minDistToPath) minDistToPath = d2;
        }
        pathAlignCost += std::sqrt(minDistToPath);
        sampleCount++;
    }
    float pathAlignScore = sampleCount > 0
        ? (1.0f / (1.0f + pathAlignCost / static_cast<float>(sampleCount)))
        : 0.0f;

    // 障碍物避让代价（基于足迹多边形）
    float minObsDist = 1.0e10f;
    for (const auto& tp : trajectory) {
        float obsDist = footprintObstacleDistance(tp,
                                                   costmapData, costmapWidth, costmapHeight,
                                                   costmapResolution, costmapOriginX, costmapOriginY);
        if (obsDist < minObsDist) minObsDist = obsDist;
    }
    // 安全距离：取足迹多边形的最小内接半径
    float inscribedRadius = config_.robotRadius;
    if (!config_.footprint.empty()) {
        inscribedRadius = 1.0e10f;
        for (const auto& pt : config_.footprint) {
            float r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            if (r < inscribedRadius) inscribedRadius = r;
        }
    }
    float safeDist = inscribedRadius * 1.5f;
    float obsScore;
    if (minObsDist < safeDist) {
        obsScore = 0.0f; // 太近，不安全
    } else {
        obsScore = std::min(1.0f, minObsDist / (safeDist * 3.0f));
    }

    // 速度偏好：偏向更高的线速度
    float endSpeed = std::sqrt(
        (endPose.x - trajectory.front().x) * (endPose.x - trajectory.front().x) +
        (endPose.y - trajectory.front().y) * (endPose.y - trajectory.front().y)
    ) / config_.simTime;
    float speedScore = std::min(1.0f, endSpeed / config_.maxLinearVel);

    // 综合评分
    float totalScore = config_.goalWeight * (goalScore * 0.5f + targetScore * 0.5f)
                     + config_.pathAlignWeight * pathAlignScore
                     + config_.obstacleWeight * obsScore
                     + config_.speedWeight * speedScore;

    return totalScore;
}

bool DWBPlanner::computeVelocity(const Pose2D& currentPose,
                                  float /*currentVx*/, float /*currentVyaw*/,
                                  const std::vector<PathPoint>& globalPath,
                                  const int8_t* costmapData,
                                  int costmapWidth, int costmapHeight,
                                  float costmapResolution,
                                  float costmapOriginX, float costmapOriginY,
                                  float& cmdVx, float& cmdVyaw)
{
    if (globalPath.empty()) {
        cmdVx   = 0.0f;
        cmdVyaw = 0.0f;
        return true;
    }

    // 找到路径上离机器人最近的点
    int closestIdx = findClosestPathIndex(globalPath, currentPose);

    // 速度采样
    float bestScore = -1.0f;
    float bestVx    = 0.0f;
    float bestVyaw  = 0.0f;

    for (float vx = config_.minLinearVel; vx <= config_.maxLinearVel; vx += config_.linearStep) {
        for (float vyaw = config_.minAngularVel; vyaw <= config_.maxAngularVel; vyaw += config_.angularStep) {
            if (vx < 0.001f && std::abs(vyaw) < 0.001f) continue;

            std::vector<Pose2D> trajectory;
            simulateTrajectory(vx, vyaw, currentPose,
                               config_.simTime, config_.simDt, trajectory);

            float score = scoreTrajectory(trajectory, globalPath, closestIdx,
                                          costmapData, costmapWidth, costmapHeight,
                                          costmapResolution, costmapOriginX, costmapOriginY);

            if (score > bestScore) {
                bestScore = score;
                bestVx    = vx;
                bestVyaw  = vyaw;
            }
        }
    }

    if (bestScore < 0.0f) {
        cmdVx   = 0.0f;
        cmdVyaw = 0.0f;
    } else {
        cmdVx   = bestVx;
        cmdVyaw = bestVyaw;
    }

    return true;
}

bool DWBPlanner::isGoalReached(const Pose2D& currentPose,
                                const std::vector<PathPoint>& globalPath) const
{
    if (globalPath.empty()) return true;

    const PathPoint& goal = globalPath.back();
    float dx = goal.x - currentPose.x;
    float dy = goal.y - currentPose.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    return dist < config_.goalTolerance;
}

int DWBPlanner::findClosestPathIndex(const std::vector<PathPoint>& globalPath,
                                      const Pose2D& pose) const
{
    int bestIdx = 0;
    float bestDist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < globalPath.size(); ++i) {
        float dx = globalPath[i].x - pose.x;
        float dy = globalPath[i].y - pose.y;
        float d2 = dx * dx + dy * dy;
        if (d2 < bestDist) {
            bestDist = d2;
            bestIdx  = static_cast<int>(i);
        }
    }
    return bestIdx;
}
