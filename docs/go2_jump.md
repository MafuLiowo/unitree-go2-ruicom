# Go2 横棒检测与越障模块（PCL 点云 + FrontJump）

## 概述

`go2_jump.cpp` 是 Go2 机器狗的横棒检测与越障程序，使用 **PCL 点云库** 处理 ROS2 `/utlidar/cloud_base` 话题的点云数据，
识别前方水平横棒（如栏杆、木棒），自动行走至棒前，并在合适距离触发 **前跳（FrontJump）** 越障。

- **点云获取**：订阅 Go2 机器人通过 ROS2 发布的 `/utlidar/cloud_base` 点云话题（标准 `sensor_msgs/PointCloud2` 格式）。
- **横棒识别**：使用 PCL 的 PassThrough 滤波器、VoxelGrid 降采样、统计离群点去除和 RANSAC 直线分割算法识别水平横棒。
- **运动控制**：自动接近横棒（带横向偏移修正和距离减速），到达起跳距离后执行宇树 SDK 前跳动作。
- **状态机**：SEARCHING → APPROACHING → JUMPING → DONE 四状态自动流转。

## 系统架构

```
┌──────────────┐   /utlidar/cloud_base    ┌──────────────────┐
│  Go2 机器人   │ ────────────────────────>│  go2_jump 节点    │
│              │  (PointCloud2)            │                  │
└──────┬───────┘                           │  PCL 横棒检测     │
       │ DDS 通信                          │  状态机控制       │
       │ <── go2_motion_bridge ───────────│  FrontJump 指令   │
       │                                   └──────────────────┘
```

## 横棒检测算法

### 处理流程

1. **ROI 滤波（PassThrough）**：截取机器人前方感兴趣区域
   - X（前方）：0.2 ~ 3.0 m
   - Y（左右）：±2.0 m
   - Z（高度）：0.05 ~ 0.50 m

2. **降采样（VoxelGrid）**：体素大小 0.02 m，减少计算量

3. **离群点去除（StatisticalOutlierRemoval）**：过滤噪声点

4. **RANSAC 直线分割**：拟合空间直线，检查是否近似水平
   - 距离阈值：0.03 m
   - 最少内点数：15
   - 水平判定：直线方向 Z 分量归一化后 < 0.15

5. **位置计算**：从内点中提取横棒最近距离、Y 偏移和 Z 高度

### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ROI_X_MIN` | 0.2 m | 最近检测距离 |
| `ROI_X_MAX` | 3.0 m | 最远检测距离 |
| `ROI_Z_MIN` | 0.05 m | 横棒最低高度 |
| `ROI_Z_MAX` | 0.50 m | 横棒最高高度 |
| `VOXEL_LEAF_SIZE` | 0.02 m | 体素降采样尺寸 |
| `RANSAC_THRESHOLD` | 0.03 m | RANSAC 拟合距离阈值 |
| `MIN_LINE_INLIERS` | 15 | 有效横棒最少内点数 |
| `LINE_HORIZONTAL_Z` | 0.15 | 水平方向 Z 阈值 |

## 运动控制参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `JUMP_DISTANCE` | 0.40 m | 起跳距离，距横棒此距离时触发前跳 |
| `APPROACH_SPEED` | 0.20 m/s | 最大接近速度 |
| `APPROACH_SPEED_MIN` | 0.08 m/s | 最小接近速度 |
| `DISTANCE_SLOWDOWN` | 1.0 m | 减速起始距离，距离小于此值时线性降速 |
| `ANGULAR_KP` | 0.6 | 角速度比例系数 (rad/s per m) |
| `MAX_ANGULAR_SPEED` | 0.8 rad/s | 最大角速度 |

### 速度控制公式

- 前进速度：`vx = v_min + (v_max - v_min) * (dist - jump_dist) / (slowdown_dist - jump_dist)`（线性插值）
- 角速度：`vyaw = -KP * centerY`（比例控制，限幅 ±0.8 rad/s）

## 启动方式

### 前提条件

1. 已编译本仓库程序
2. 已 source ROS2 Foxy 环境
3. Go2 机器人已连接并通电
4. Go2 机器人已启动 `utlidar` 点云发布（通常已随 Go2 系统启动）

### 步骤

```bash
source /opt/ros/foxy/setup.bash
cd /path/to/ai-unitree-go2-ruicom/build
./go2_jump eth0
```

程序启动后自动：
1. 初始化 Go2 运动桥接（DDS 通信）
2. 机器人站立
3. 创建 ROS2 节点并订阅 `/utlidar/cloud_base`
4. 进入 SEARCHING 状态等待横棒检测

## 控制命令

程序启动后在终端输入以下命令：

| 命令 | 功能 |
|------|------|
| `q` | 退出程序（机器人趴下） |
| `Space` | 暂停/恢复运动 |

## 状态机说明

| 状态 | 说明 | 触发条件 |
|------|------|----------|
| SEARCHING | 搜索横棒，机器人原地待命 | 程序启动，或横棒信号丢失 |
| APPROACHING | 已检测到横棒，正在接近 | 检测到有效横棒 |
| JUMPING | 到达起跳距离，执行前跳 | `distance <= JUMP_DISTANCE` |
| DONE | 越障完成，保持不动 | 前跳指令已发出 |

## 依赖项

- `rclcpp`：ROS2 客户端库
- `sensor_msgs`：ROS2 点云消息
- `pcl_conversions`：ROS2 ↔ PCL 数据转换
- `PCL 1.10`：点云处理库（filters, segmentation, common）
- `go2_motion_bridge`：Go2 运动控制与传感器数据桥接
- `unitree_sdk2`：宇树 SDK2（运动桥接所需）
