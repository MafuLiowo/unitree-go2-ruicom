# Go2 自主导航模块（Nav2 FollowWaypoints / NavigateToPose）

## 概述

`go2_navigation.cpp` 是 Go2 机器狗的自主导航程序，桥接 Go2 传感器数据到 ROS2 导航栈，
并通过 Nav2 动作接口发送路径点进行自主导航。

- **传感器桥接**：将 Go2 的雷达点云、里程计数据转换为标准 ROS2 消息（`/utlidar/cloud`、`/scan`、`/odom`），广播 TF 变换。
- **运动转发**：订阅 `/cmd_vel` 话题，将 Nav2 规划的速度指令转发到 Go2 运动桥接。
- **路径点发送**：启动时自动读取 `route.yaml`，通过 Nav2 动作服务器发送导航目标。
  - 优先使用 **FollowWaypoints** 一次性发送全部路径点（需 `bt_navigator` 中的 waypoint_follower 插件）。
  - 不可用时回退到 **NavigateToPose** 逐个发送。

## 系统架构

```
┌──────────────┐     /amcl_pose     ┌──────────────────┐     navigate_to_pose      ┌──────────────────┐
│   AMCL 节点   │ ─────────────────>│                   │<──────────────────────────│   Nav2 导航栈     │
│  (nav2_amcl)  │                   │  go2_navigation   │     follow_waypoints       │ bt_navigator +   │
└──────────────┘                   │   (本程序)        │<──────────────────────────│ planner_server + │
                                    │                   │                           │ controller_server│
┌──────────────┐     /map           │  传感器桥接       │──── /scan /odom /tf ────>│                  │
│  map_server   │ ─────────────────>│  路径点发送       │<──── /cmd_vel ───────────│                  │
│(nav2_map_srv) │                   │                   │                           └──────────────────┘
└──────────────┘                   └────────┬─────────┘
                                            │ DDS 通信
                                     ┌──────▼──────────┐
                                     │   Go2 机器人     │
                                     └─────────────────┘
```

## route.yaml 格式

```yaml
waypoints:
    - frame: map
      x: 1.49987
      y: 0.743541
      yaw: 1.99759

    - frame: map
      x: 1.27954
      y: 1.31652
      yaw: -2.74095
```

- `frame`: 坐标帧 ID，通常为 `map`
- `x` / `y`: 地图坐标系的坐标 (m)
- `yaw`: 目标偏航角 (rad)

## 启动方式

### 前提条件

1. 已编译本仓库程序
2. 已 source ROS2 Foxy 环境
3. 已准备好地图文件（`.yaml` + `.pgm`）
4. 已准备好 `route.yaml` 路径点文件
5. Nav2 导航栈、AMCL、map_server 已安装

### 步骤

在 3 个终端中分别执行：

**终端 1：启动 Nav2 导航栈**

```bash
source /opt/ros/foxy/setup.bash

# 启动 map_server
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/path/to/map.yaml

# 启动 AMCL
ros2 run nav2_amcl amcl --ros-args --params-file /path/to/amcl_params.yaml

# 启动 Nav2 导航栈（planner + controller + bt_navigator）
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=/path/to/nav2_params.yaml \
    map:=/path/to/map.yaml
```

**终端 2：启动 Go2 导航程序（本程序）**

```bash
source /opt/ros/foxy/setup.bash
cd /path/to/ai-unitree-go2-ruicom/build
./go2_navigation eth0
```

或指定 route.yaml 路径：

```bash
./go2_navigation eth0 ../src/route.yaml
```

程序启动后自动：
1. 初始化 Go2 运动桥接，发布传感器数据和 TF
2. 读取 `route.yaml` 解析路径点
3. 尝试连接 FollowWaypoints 动作服务器，发送全部路径点
4. 若 FollowWaypoints 不可用，回退到 NavigateToPose 逐个发送

## 控制命令

程序启动后在终端输入以下命令（按回车确认）：

| 命令 | 功能 |
|------|------|
| `r` | 重新发送所有路径点（先停止当前导航） |
| `s` | 停止导航 |
| `q` | 退出程序（机器人趴下） |

## 导航模式

### FollowWaypoints 模式（优先）

1. 一次性将所有路径点发送到 Nav2 的 `follow_waypoints` 动作服务器
2. Nav2 的 waypoint_follower BT 节点自动按序导航
3. 实时反馈当前正在前往的路径点索引
4. 结果返回未到达的路径点索引列表

### NavigateToPose 模式（回退）

1. 逐个发送路径点到 Nav2 的 `navigate_to_pose` 动作服务器
2. 当前路径点完成（SUCCEEDED）后自动发送下一个
3. 任一路径点失败（ABORTED）则停止导航

## 依赖项

- `rclcpp` / `rclcpp_action`：ROS2 客户端库
- `nav2_msgs`：Nav2 动作/消息定义
- `sensor_msgs` / `nav_msgs` / `geometry_msgs`：ROS2 标准消息
- `tf2_ros`：TF 坐标变换
- `yaml-cpp`：YAML 文件解析
- `go2_motion_bridge`：Go2 运动控制与传感器数据桥接
- `unitree_sdk2`：宇树 SDK2（运动桥接所需）

## 与 go2_slam.cpp 的区别

| 特性 | go2_slam.cpp | go2_navigation.cpp |
|------|-------------|--------------------|
| 建图支持 | 是（SLAM 地图显示） | 否 |
| 导航方式 | 手动输入目标，单次 NavigateToPose | 自动读取 route.yaml 批量发送 |
| FollowWaypoints | 不支持 | 支持（优先） |
| 用途 | SLAM 建图 + 调试导航 | 批量路径点自主导航 |
