# Go2 SLAM 建图与 Nav2 导航

## 概述

`go2_slam` 是 Go2 机器人与 ROS2 (Nav2 + slam_toolbox) 导航栈之间的桥接节点。它将 Go2 的雷达点云、里程计数据转换为标准 ROS2 消息，通过 TF 广播坐标变换，并接收 Nav2 规划的速度指令驱动 Go2 运动。

## 编译

```bash
source /opt/ros/foxy/setup.bash   # 或 humble / jazzy
cd build && cmake .. && make go2_slam
```

## 使用方法

### 1. 启动程序

```bash
./go2_slam <network_interface>
# 示例:
./go2_slam eth0
```

### 2. 并行启动 ROS2 节点（需在其它终端）

```bash
# 终端 A：启动 SLAM 工具箱
ros2 launch slam_toolbox online_async_launch.py

# 终端 B：启动 Nav2 导航栈
ros2 launch nav2_bringup navigation_launch.py
```

### 3. 交互命令

在 `go2_slam` 运行终端的 `> ` 提示符下输入命令后按回车：

| 命令 | 功能 |
|------|------|
| `m` | 开始建图 (SLAM)，遥控机器人移动构建地图 |
| `v` | 显示当前地图（ASCII 缩略图、尺寸、分辨率、覆盖率） |
| `s` | 打印保存地图命令（需在另一终端手动执行） |
| `g` / `1` | 发送导航目标 (x=2, y=0, yaw=0) —— 向前直行 2 米 |
| `2` | 发送导航目标 (x=0, y=2, yaw=π/2) —— 向左平移 2 米 |
| `3` | 发送导航目标 (x=2, y=2, yaw=π/4) —— 右前方约 2.8 米 |
| `q` | 停止运动 → 趴下 → 退出程序 |

### 4. 导航目标自定义

如需自定义导航目标，修改代码中 `navigateToPose(x, y, yaw)` 的调用即可，参数单位均为米和弧度。

## 实时地图显示

按下 `m` 开始建图后，程序会：
- 每 **3 秒** 自动打印 ASCII 缩略图（仅在地图有更新时）
- 随时按 `v` 手动查看当前地图
- 显示信息包含：地图序号、尺寸（宽 × 高）、分辨率、覆盖率

ASCII 图例：
- `.` = 空闲区域
- `+` = 部分占据
- `#` = 障碍物
- ` ` (空格) = 未知区域

## 运动控制参数

| 参数 | 值 | 含义 |
|------|-----|------|
| NAV_FORWARD_SPEED_MAX | 0.30 m/s | 最大前进速度 |
| NAV_ANGULAR_SPEED_MAX | 1.00 rad/s | 最大转向角速度 |

## 依赖

- ROS2 (foxy/humble/jazzy)
- rclcpp, sensor_msgs, nav_msgs, geometry_msgs, tf2_ros, nav2_msgs
- Unitree SDK2 + ddsc/ddscxx
- go2_motion_bridge (本项目的 C API 封装)

## 运行流程

```
初始化运动桥接 → ROS2 节点创建 → 机器人站立 → 等待命令
    ↓
[m] 开始建图 → 发布传感器数据 → slam_toolbox 构建地图 → 自动/手动显示地图
    ↓
[g/1/2/3] 发送导航目标 → Nav2 规划路径 → /cmd_vel 驱动 Go2 → 到达目标
    ↓
[q] 退出 → 停止运动 → 趴下 → 关闭
```
