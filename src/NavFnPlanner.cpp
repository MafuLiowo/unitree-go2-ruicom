/**
 * @file NavFnPlanner.cpp
 * @brief NavFn 全局规划器实现，包含势场传播和梯度下降路径提取
 */
#include "NavFnPlanner.hpp"

#include <cmath>
#include <cstring>

constexpr int NavFnPlanner::NB_OFFSET_X[];
constexpr int NavFnPlanner::NB_OFFSET_Y[];

NavFnPlanner::NavFnPlanner() {}

NavFnPlanner::~NavFnPlanner() {}

void NavFnPlanner::setParams(float cellCostMin, float cellCostMax, float obsCostThreshold)
{
    cellCostMin_      = cellCostMin;
    cellCostMax_      = cellCostMax;
    obsCostThreshold_ = obsCostThreshold;
}

void NavFnPlanner::setCostmap(const int8_t* mapData, int width, int height,
                              float resolution, float originX, float originY)
{
    mapWidth_   = width;
    mapHeight_  = height;
    resolution_ = resolution;
    originX_    = originX;
    originY_    = originY;

    int totalCells = width * height;
    costArr_.resize(static_cast<size_t>(totalCells));
    potArr_.resize(static_cast<size_t>(totalCells));

    for (int i = 0; i < totalCells; ++i) {
        int8_t occ = mapData[i];
        float cost;
        if (occ < 0) {
            // 未知区域：中等代价，允许通过但不鼓励
            cost = cellCostMax_ * 0.3f;
        } else if (occ > 75) {
            // 障碍物：高代价
            cost = cellCostMax_;
        } else {
            // 空闲区域：代价与占据程度成比例
            float ratio = static_cast<float>(occ) / 100.0f;
            cost = cellCostMin_ + ratio * (obsCostThreshold_ - cellCostMin_);
        }
        costArr_[i] = cost;
        potArr_[i]  = POT_HIGH;
    }

    initialized_ = true;
}

bool NavFnPlanner::worldToMap(float wx, float wy, int& mx, int& my) const
{
    mx = static_cast<int>(std::floor((wx - originX_) / resolution_));
    my = static_cast<int>(std::floor((wy - originY_) / resolution_));
    return (mx >= 0 && mx < mapWidth_ && my >= 0 && my < mapHeight_);
}

void NavFnPlanner::mapToWorld(int mx, int my, float& wx, float& wy) const
{
    wx = originX_ + (static_cast<float>(mx) + 0.5f) * resolution_;
    wy = originY_ + (static_cast<float>(my) + 0.5f) * resolution_;
}

float NavFnPlanner::costAt(int mx, int my) const
{
    if (mx < 0 || mx >= mapWidth_ || my < 0 || my >= mapHeight_) {
        return cellCostMax_;
    }
    return costArr_[static_cast<size_t>(my * mapWidth_ + mx)];
}

float NavFnPlanner::potAt(int mx, int my) const
{
    if (mx < 0 || mx >= mapWidth_ || my < 0 || my >= mapHeight_) {
        return POT_HIGH;
    }
    return potArr_[static_cast<size_t>(my * mapWidth_ + mx)];
}

void NavFnPlanner::potSet(int mx, int my, float val)
{
    if (mx >= 0 && mx < mapWidth_ && my >= 0 && my < mapHeight_) {
        potArr_[static_cast<size_t>(my * mapWidth_ + mx)] = val;
    }
}

bool NavFnPlanner::potUpdate(int mx, int my, float val)
{
    if (mx < 0 || mx >= mapWidth_ || my < 0 || my >= mapHeight_) {
        return false;
    }
    size_t idx = static_cast<size_t>(my * mapWidth_ + mx);
    if (val < potArr_[idx]) {
        potArr_[idx] = val;
        return true;
    }
    return false;
}

bool NavFnPlanner::propagatePotential(int goalMx, int goalMy, int startMx, int startMy)
{
    // 重置所有势场值为无穷大
    std::fill(potArr_.begin(), potArr_.end(), POT_HIGH);

    // 从目标点开始传播
    std::priority_queue<PotCell, std::vector<PotCell>, std::greater<PotCell>> pq;
    potSet(goalMx, goalMy, 0.0f);
    pq.push({goalMx, goalMy, 0.0f});

    bool startReached = false;

    while (!pq.empty()) {
        PotCell current = pq.top();
        pq.pop();

        // 若弹出的势场值比已记录的更大，说明是过期项，跳过
        if (current.pot > potAt(current.mx, current.my) + 0.001f) {
            continue;
        }

        // 检查是否已到达起点
        if (current.mx == startMx && current.my == startMy) {
            startReached = true;
        }

        // 8 连通邻域传播
        for (int i = 0; i < NB_OFFSET_COUNT; ++i) {
            int nx = current.mx + NB_OFFSET_X[i];
            int ny = current.my + NB_OFFSET_Y[i];

            if (nx < 0 || nx >= mapWidth_ || ny < 0 || ny >= mapHeight_) {
                continue;
            }

            float cellCost = costAt(nx, ny);
            if (cellCost >= obsCostThreshold_) {
                continue; // 障碍物，不传播
            }

            // 计算邻居到当前点的距离（对角线更远）
            float dist = (NB_OFFSET_X[i] != 0 && NB_OFFSET_Y[i] != 0)
                         ? 1.41421356f : 1.0f;

            float newPot = current.pot + dist * cellCost;

            if (potUpdate(nx, ny, newPot)) {
                pq.push({nx, ny, newPot});
            }
        }
    }

    return startReached;
}

void NavFnPlanner::tracePath(int startMx, int startMy, int goalMx, int goalMy,
                             std::vector<std::pair<int, int>>& path)
{
    int cx = startMx;
    int cy = startMy;

    path.clear();
    path.emplace_back(cx, cy);

    int maxSteps = mapWidth_ * mapHeight_;
    int steps = 0;

    while (steps < maxSteps) {
        steps++;

        if (cx == goalMx && cy == goalMy) {
            break;
        }

        int bestNx = cx, bestNy = cy;
        float bestPot = potAt(cx, cy);
        bool improved = false;

        for (int i = 0; i < NB_OFFSET_COUNT; ++i) {
            int nx = cx + NB_OFFSET_X[i];
            int ny = cy + NB_OFFSET_Y[i];

            float npot = potAt(nx, ny);
            if (npot < bestPot - 0.0001f) {
                bestPot   = npot;
                bestNx    = nx;
                bestNy    = ny;
                improved  = true;
            }
        }

        if (!improved) {
            break; // 陷入局部极小值或已到达
        }

        cx = bestNx;
        cy = bestNy;
        path.emplace_back(cx, cy);
    }
}

void NavFnPlanner::smoothPath(const std::vector<std::pair<int, int>>& rawPath,
                              std::vector<PathPoint>& smoothedPath)
{
    smoothedPath.clear();
    if (rawPath.size() < 2) {
        for (const auto& p : rawPath) {
            PathPoint pp;
            mapToWorld(p.first, p.second, pp.x, pp.y);
            pp.dx = 0.0f;
            pp.dy = 0.0f;
            smoothedPath.push_back(pp);
        }
        return;
    }

    // 使用三次样条平滑的思路：对每段路径做窗口平均平滑
    int windowSize = 3;
    size_t n = rawPath.size();

    // 先将所有点转为世界坐标
    std::vector<PathPoint> worldPoints;
    worldPoints.reserve(n);
    for (const auto& p : rawPath) {
        PathPoint pp;
        mapToWorld(p.first, p.second, pp.x, pp.y);
        worldPoints.push_back(pp);
    }

    // 移动平均平滑
    smoothedPath.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        int start = static_cast<int>(i) - windowSize / 2;
        int end   = static_cast<int>(i) + windowSize / 2;
        if (start < 0) { start = 0; }
        if (end >= static_cast<int>(n)) { end = static_cast<int>(n) - 1; }

        float sumX = 0.0f, sumY = 0.0f;
        int count = 0;
        for (int j = start; j <= end; ++j) {
            sumX += worldPoints[static_cast<size_t>(j)].x;
            sumY += worldPoints[static_cast<size_t>(j)].y;
            count++;
        }

        PathPoint pp;
        pp.x = sumX / static_cast<float>(count);
        pp.y = sumY / static_cast<float>(count);
        smoothedPath.push_back(pp);
    }

    // 设置方向向量
    for (size_t i = 0; i < smoothedPath.size(); ++i) {
        size_t next = std::min(i + 1, smoothedPath.size() - 1);
        float dx = smoothedPath[next].x - smoothedPath[i].x;
        float dy = smoothedPath[next].y - smoothedPath[i].y;
        float len = std::sqrt(dx * dx + dy * dy);
        if (len > 1e-6f) {
            smoothedPath[i].dx = dx / len;
            smoothedPath[i].dy = dy / len;
        } else {
            smoothedPath[i].dx = 0.0f;
            smoothedPath[i].dy = 0.0f;
        }
    }
}

bool NavFnPlanner::planPath(float startX, float startY, float goalX, float goalY,
                            std::vector<PathPoint>& path)
{
    if (!initialized_) return false;

    int startMx = 0, startMy = 0, goalMx = 0, goalMy = 0;

    if (!worldToMap(startX, startY, startMx, startMy)) return false;
    if (!worldToMap(goalX, goalY, goalMx, goalMy)) return false;

    // 若起点或目标点在障碍物上，尝试向外搜索最近空闲栅格
    if (costAt(startMx, startMy) >= obsCostThreshold_) {
        bool found = false;
        for (int r = 1; r < 10 && !found; ++r) {
            for (int dx = -r; dx <= r && !found; ++dx) {
                for (int dy = -r; dy <= r && !found; ++dy) {
                    int nx = startMx + dx;
                    int ny = startMy + dy;
                    if (costAt(nx, ny) < obsCostThreshold_) {
                        startMx = nx;
                        startMy = ny;
                        found = true;
                    }
                }
            }
        }
        if (!found) return false;
    }

    if (costAt(goalMx, goalMy) >= obsCostThreshold_) {
        bool found = false;
        for (int r = 1; r < 10 && !found; ++r) {
            for (int dx = -r; dx <= r && !found; ++dx) {
                for (int dy = -r; dy <= r && !found; ++dy) {
                    int nx = goalMx + dx;
                    int ny = goalMy + dy;
                    if (costAt(nx, ny) < obsCostThreshold_) {
                        goalMx = nx;
                        goalMy = ny;
                        found = true;
                    }
                }
            }
        }
        if (!found) return false;
    }

    // 势场传播
    if (!propagatePotential(goalMx, goalMy, startMx, startMy)) {
        return false; // 起点不可达
    }

    // 梯度下降提取路径
    std::vector<std::pair<int, int>> rawPath;
    tracePath(startMx, startMy, goalMx, goalMy, rawPath);

    if (rawPath.size() < 2) return false;

    // 平滑路径
    smoothPath(rawPath, path);

    return !path.empty();
}
