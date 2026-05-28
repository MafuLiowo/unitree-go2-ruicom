# Go2 上下楼梯模块使用说明

`go2_walk_stair` 用于实现 Go2 机器狗的自主上下楼梯功能。程序通过运动桥接模块订阅 IMU 姿态数据（等价于 ROS2 `/lowstate` 节点），实时检测机器狗的姿态变化，完成爬楼梯、检测平地、旋转、下楼梯的完整流程。

## 使用示例

```bash
# 基本用法，指定网络接口
./go2_walk_stair eth0
```

## 执行流程

```
爬楼梯模式(StaticWalk) → 前进上楼梯(速度 upstairVel)
    ↓ IMU 实时检测
到达平地(Pitch/Roll 回零) → 逆时针旋转45°
    ↓
下楼梯(前进距离 downstair) → 切换行走模式(FreeWalk) → 程序结束
```

## 可配置参数

| 参数变量 | 默认值 | 说明 |
|----------|--------|------|
| `UPSTAIR_VEL` | `0.3f` | 上楼梯前进速度 (m/s)，可在代码中修改 |
| `DOWNSTAIR_VEL` | `0.3f` | 下楼梯前进速度 (m/s)，可在代码中修改 |
| `DOWNSTAIR_DIST` | `1.5f` | 下楼梯前进距离 (m) |
| `ROTATE_SPEED` | `0.5f` | 旋转角速度 (rad/s) |
| `ROTATE_ANGLE` | `π/4` | 旋转角度 (45°) |
| `FLAT_PITCH_THRESHOLD` | `0.087f` | 水平俯仰角阈值 (rad)，约5° |
| `FLAT_ROLL_THRESHOLD` | `0.087f` | 水平横滚角阈值 (rad)，约5° |
| `STABLE_FRAMES_REQUIRED` | `10` | 判断为水平的连续稳定帧数 |

## IMU 数据来源

IMU 姿态数据通过 `go2_motion_bridge` 模块获取，该模块订阅 DDS 运动状态话题 `rt/sportmodestate`，其中的 `imu_state` 字段与 ROS2 `/lowstate` 节点的 `imu_state` 等价，均包含：

- `rpy[0]`：横滚角 Roll (rad)
- `rpy[1]`：俯仰角 Pitch (rad)
- `rpy[2]`：偏航角 Yaw (rad)

## 状态机说明

程序采用五状态状态机控制：

1. **INIT**：站立 → 调用 `StaticWalk()` 启用爬楼梯步态 → 开始前进
2. **GOING_UP**：持续前进，实时读取 IMU 的 Roll/Pitch 角判断机器人是否水平。仅当机器人曾经历不平坦阶段后再恢复水平，才判定为到达平地
3. **ROTATING**：停止移动，以固定角速度逆时针旋转45°
4. **GOING_DOWN**：前进下楼梯，通过计时计算行进距离，达到 `DOWNSTAIR_DIST` 后停止
5. **DONE**：调用 `FreeWalk()` 切换为行走模式，程序结束

## 运动桥接 API

本程序依赖 `go2_motion_bridge` 模块提供的以下新增 API：

```c
// 获取 IMU 姿态数据
bool go2_motion_get_imu(float* roll, float* pitch, float* yaw);

// 切换到静态行走步态（爬楼梯模式）
void go2_motion_static_walk();

// 切换到自由行走步态（正常行走模式）
void go2_motion_free_walk();
```

## 控制说明

| 按键 | 功能 |
|------|------|
| `q` | 退出程序 |
