# Go2 自主导航模块（NavFn + DWB + AMCL）

## 概述

`go2_navigation.cpp` 是 Go2 机器狗的自主导航程序，集成以下组件：

- **AMCL（自适应蒙特卡洛定位）**：通过 ROS2 `nav2_amcl` 节点订阅 `/amcl_pose` 话题获取机器人地图坐标定位。
- **NavFn（导航函数全局规划器）**：基于势场传播算法，从目标点向外扩散波前，再通过梯度下降提取全局最优路径。
- **DWB（动态窗口局部规划器）**：在速度空间采样多组速度指令，通过轨迹模拟与代价函数评分选择实时最优速度。
- **Go2 运动桥接**：通过 `go2_motion_bridge` C API 直接控制机器人运动并获取传感器数据。

## 系统架构

```
┌─────────────────┐     /amcl_pose     ┌──────────────────┐
│   AMCL 节点      │ ─────────────────>│  go2_navigation   │
│  (nav2_amcl)     │                    │   (本程序)        │
└─────────────────┘                    │                  │
                                        │  NavFnPlanner     │── 全局路径规划
┌─────────────────┐     /map           │  DWBPlanner       │── 局部速度生成
│  map_server      │ ─────────────────>│                  │──── /cmd_vel
│  (nav2_map_server)│                   │  go2_motion_bridge│── 传感器获取
└─────────────────┘                    └────────┬─────────┘
                                                │ DDS 通信
                                         ┌──────▼──────────┐
                                         │   Go2 机器人     │
                                         └─────────────────┘
```

## 启动方式

### 前提条件

1. 已编译本仓库程序
2. 已 source ROS2 Foxy 环境
3. 已准备好地图文件（`.yaml` + `.pgm`）
4. `nav2_amcl` 和 `nav2_map_server` 已安装

### 步骤

在 3 个终端中分别执行：

**终端 1：启动 map_server 提供静态地图**

```bash
source /opt/ros/foxy/setup.bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/path/to/your_map.yaml
```

**终端 2：启动 AMCL 定位**

```bash
source /opt/ros/foxy/setup.bash
ros2 run nav2_amcl amcl --ros-args \
  --params-file /path/to/amcl_params.yaml
```

AMCL 参数示例 (`amcl_params.yaml`)：

```yaml
amcl:
  ros__parameters:
    use_sim_time: false
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 20.0
    laser_min_range: 0.05
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "differential"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
```

**终端 3：启动 Go2 导航程序（本程序）**

```bash
source /opt/ros/foxy/setup.bash
cd /path/to/ai-unitree-go2-ruicom/build
./go2_navigation eth0 --ros-args -p footprint:="[[0.25,0.15],[0.25,-0.15],[-0.25,-0.15],[-0.25,0.15]]"
```

若未指定 `footprint` 参数，默认使用 `robot_radius=0.35` 的圆形足迹。

本程序会自动：
1. 初始化 Go2 运动桥接（发布 `/scan`、`/odom`、TF）
2. 订阅 `/amcl_pose` 获取定位
3. 订阅 `/map` 获取地图
4. 等待用户输入导航目标

## 控制命令

程序启动后在终端输入以下命令（按回车确认）：

| 命令 | 功能 |
|------|------|
| `g` / `1` | 导航到前方 2m 处 (2, 0, 0°) |
| `2` | 导航到左方 2m 处 (0, 2, 90°) |
| `3` | 导航到右前方 (2, 2, 45°) |
| `p` | 打印当前导航状态（位姿、地图、路径信息） |
| `s` | 停止导航 |
| `q` | 退出程序（机器人趴下） |

## 核心算法

### NavFn 全局规划器（NavFnPlanner）

基于 Navigation Function 的全局路径规划算法，分为两个阶段：

1. **势场传播（Propagation）**
   - 从目标栅格出发，使用 Dijkstra 优先队列向 8 连通邻域传播势场值
   - 每个栅格的势场值 = 邻居势场值 + 距离 × 栅格代价值
   - 障碍物栅格（代价 ≥ 阈值）不参与传播
   - 持续传播直到起点栅格被访问

2. **梯度下降路径提取（Trace Back）**
   - 从起点栅格出发，每次选择 8 连通邻域中势场值最小的方向移动
   - 重复直到到达目标栅格或陷入局部极小值

3. **路径平滑**
   - 使用移动窗口平均平滑栅格坐标路径
   - 输出每个路径点的世界坐标和方向向量

参数可通过 `setParams()` 调整：
- `cellCostMin`：空闲单元格最小代价（默认 1.0）
- `cellCostMax`：障碍物最大代价（默认 1000.0）
- `obsCostThreshold`：障碍物阈值（默认 100.0）

### DWB 局部规划器（DWBPlanner）

基于 Dynamic Window Approach 的局部路径规划算法：

1. **速度采样**
   - 在线速度 `[minLinearVel, maxLinearVel]` 范围以 `linearStep` 步长采样
   - 在角速度 `[minAngularVel, maxAngularVel]` 范围以 `angularStep` 步长采样
   - 排除 (0, 0) 采样点

2. **轨迹模拟**
   - 对每组速度 `(vx, vyaw)`，以 `simDt` 为步长前向模拟 `simTime` 秒
   - 使用简化的差分驱动运动模型：`x += vx * dt * cos(yaw)`, `y += vx * dt * sin(yaw)`, `yaw += vyaw * dt`

3. **轨迹评分**
   - **目标趋近代价**（goalWeight）：轨迹终点到前瞻路径点距离越近分数越高
   - **路径对齐代价**（pathAlignWeight）：轨迹上采样点到全局路径的最小距离越小越好
   - **障碍物避让代价**（obstacleWeight）：轨迹点到最近障碍物距离（机器人半径内为 0）
   - **速度偏好代价**（speedWeight）：偏向更高的线速度

4. **最优速度选择**：选择综合评分最高的速度对输出

配置参数通过 `DWBConfig` 结构体设置：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `maxLinearVel` | 0.30 m/s | 最大线速度 |
| `minLinearVel` | 0.00 m/s | 最小线速度 |
| `maxAngularVel` | 1.00 rad/s | 最大角速度 |
| `minAngularVel` | -1.00 rad/s | 最负角速度 |
| `linearStep` | 0.05 m/s | 线速度采样步长 |
| `angularStep` | 0.10 rad/s | 角速度采样步长 |
| `simTime` | 1.50 s | 前向模拟时长 |
| `simDt` | 0.10 s | 模拟时间步长 |
| `robotRadius` | 0.35 m | 机器人碰撞半径（`footprint` 为空时作为圆形足迹半径） |
| `footprint` | 空 | 机器人足迹多边形顶点（相对于 baselink 中心，逆时针），非空时取代 `robotRadius` |
| `goalTolerance` | 0.30 m | 目标到达容差 |

## 机器人足迹（Footprint）配置

本程序集成 Nav2 的 `nav2_costmap_2d` 库，支持通过 ROS2 参数定义多边形足迹。

### 多边形足迹

在启动时传入 `--ros-args -p` 参数：

```bash
# 矩形足迹：Go2 机身约 0.5m × 0.3m
./go2_navigation eth0 --ros-args -p \
    footprint:="[[0.25,0.15],[0.25,-0.15],[-0.25,-0.15],[-0.25,0.15]]"
```

格式为标准 Nav2 footprint 参数：`[[x1,y1],[x2,y2],...]`，顶点为相对于 baselink 中心的坐标（米），逆时针排列。

### 圆形足迹

使用 `robot_radius` 参数（默认 0.35m），此时不设置 `footprint` 即可：

```bash
./go2_navigation eth0 --ros-args -p robot_radius:=0.35
```

若不传任何参数，默认使用 `robot_radius=0.35` 的圆形足迹（16 边形近似）。

### 碰撞检测逻辑

1. DWB 对每个采样速度模拟轨迹
2. 对轨迹上的每位姿，调用 `transformFootprintToWorld()` 将足迹多边形变换到世界坐标系
3. 对每个足迹顶点，查询局部代价地图中到最近障碍物的距离
4. 取所有顶点中的最小距离作为该位姿的障碍物距离
5. 障碍物评分低于安全阈值时，该轨迹被否决

## 模块接口

### NavFnPlanner

```cpp
#include "NavFnPlanner.hpp"

NavFnPlanner planner;
planner.setParams(1.0f, 1000.0f, 100.0f);
planner.setCostmap(mapData, width, height, resolution, originX, originY);

std::vector<PathPoint> path;
planner.planPath(startX, startY, goalX, goalY, path);
```

### DWBPlanner

```cpp
#include "DWBPlanner.hpp"

DWBConfig config;
config.maxLinearVel = 0.3f;
// ... 其他参数

DWBPlanner planner(config);

float cmdVx, cmdVyaw;
planner.computeVelocity(currentPose, currentVx, currentVyaw,
                        globalPath, costmapData, width, height,
                        resolution, originX, originY,
                        cmdVx, cmdVyaw);
```

### MapLoader

```cpp
#include "MapLoader.hpp"

MapLoader loader;
loader.loadFromYaml("/path/to/map.yaml");

const auto& data = loader.getMapData();
const auto& meta = loader.getMetaData();
int mx, my;
loader.worldToMap(1.5, 2.0, mx, my);
```
