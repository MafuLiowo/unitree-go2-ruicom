/**
 * @file go2_walk_stair.cpp
 * @brief Go2 机器人上下楼梯程序，通过 IMU 姿态检测平地并自动切换运动模式
 *
 * @par 使用说明
 *       go2_walk_stair <network_interface>
 *       示例: ./go2_walk_stair eth0
 *
 *       说明: 本程序通过 go2_motion_bridge 订阅运动状态中的 IMU 数据（等价于
 *             ROS2 /lowstate 节点），检测机器人姿态是否水平。
 *             流程: 爬楼梯模式 → 前进上楼梯 → 检测平地 → 逆时针旋转45° →
 *                   前进下楼梯 → 切换行走模式 → 程序结束。
 *
 *       控制: [q] 退出程序
 */
#include <rclcpp/rclcpp.hpp>
#include "go2_motion_bridge.hpp"

#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>

#include <unistd.h>
#include <sys/select.h>

// ==========================================================================
// 可配置参数
// ==========================================================================
constexpr float UPSTAIR_VEL            = 0.3f;       ///< 上楼梯前进速度 (m/s)
constexpr float DOWNSTAIR_VEL          = 0.3f;       ///< 下楼梯前进速度 (m/s)
constexpr float DOWNSTAIR_DIST         = 1.5f;       ///< 下楼梯前进距离 (m)
constexpr float ROTATE_SPEED           = 0.5f;       ///< 旋转角速度 (rad/s)
constexpr float ROTATE_ANGLE           = 0.785398f;  ///< 旋转角度 (rad)，45° = π/4
constexpr float FLAT_PITCH_THRESHOLD   = 0.087f;     ///< 水平俯仰角阈值 (rad)，约5°
constexpr float FLAT_ROLL_THRESHOLD    = 0.087f;     ///< 水平横滚角阈值 (rad)，约5°
constexpr int   STABLE_FRAMES_REQUIRED = 10;         ///< 连续稳定帧数判断为水平

/// π 常量（C++17 兼容替代 M_PI）
constexpr double PI = 3.14159265358979323846;

// ==========================================================================
// 状态机枚举
// ==========================================================================
enum class StairState {
    INIT,        ///< 初始化：站立、切换爬楼梯模式
    GOING_UP,    ///< 上楼梯中：向前移动，检测 IMU 是否水平
    ROTATING,    ///< 旋转中：逆时针旋转45°
    GOING_DOWN,  ///< 下楼梯中：向前移动指定距离
    DONE         ///< 完成：切换行走模式，结束
};

// ==========================================================================
// 全局控制变量
// ==========================================================================
static std::atomic<bool> g_running{true};

/**
 * @brief Go2 上下楼梯节点类
 *
 * 通过 go2_motion_bridge 读取 IMU 姿态数据（等价于订阅 ROS2 /lowstate），
 * 使用状态机控制机器人完成上楼梯 → 旋转 → 下楼梯 → 切换行走模式的完整流程。
 */
class Go2WalkStairNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数，初始化状态机和定时器
     */
    Go2WalkStairNode()
        : Node("go2_walk_stair_node")
    {
        // ---- 控制定时器 (20Hz) ----
        controlTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Go2WalkStairNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Go2 上下楼梯节点已就绪");
    }

private:
    /**
     * @brief 控制循环：根据状态机驱动机器人运动
     */
    void controlLoop()
    {
        switch (state_) {
        case StairState::INIT:
            handleInit();
            break;
        case StairState::GOING_UP:
            handleGoingUp();
            break;
        case StairState::ROTATING:
            handleRotating();
            break;
        case StairState::GOING_DOWN:
            handleGoingDown();
            break;
        case StairState::DONE:
            handleDone();
            break;
        }
    }

    /**
     * @brief 初始化阶段：站立、启用爬楼梯步态、开始前进
     */
    void handleInit()
    {
        go2_motion_stand_up();
        RCLCPP_INFO(this->get_logger(), "机器人站立中...");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        go2_motion_static_walk();
        RCLCPP_INFO(this->get_logger(), "已启用爬楼梯模式 (StaticWalk)");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        go2_motion_move(UPSTAIR_VEL, 0.0f, 0.0f);
        RCLCPP_INFO(this->get_logger(),
            "开始上楼梯: 前进速度 = %.2f m/s, Pitch阈值 = %.3f rad, Roll阈值 = %.3f rad",
            UPSTAIR_VEL, FLAT_PITCH_THRESHOLD, FLAT_ROLL_THRESHOLD);

        state_ = StairState::GOING_UP;
    }

    /**
     * @brief 上楼梯阶段：向前移动，实时检测 IMU 姿态判断是否到达平地
     *
     * 检测策略：
     *   1. 连续记录 IMU 俯仰/横滚角
     *   2. 若夹角曾显著偏离水平（说明正在爬楼梯），之后恢复水平则判定到达平地
     *   3. 需要连续若干帧稳定在水平阈值内才认为可靠
     */
    void handleGoingUp()
    {
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        if (!go2_motion_get_imu(&roll, &pitch, &yaw)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(),
                *this->get_clock(), 1000,
                "无法获取 IMU 数据，等待中...");
            return;
        }

        float absRoll  = std::fabs(roll);
        float absPitch = std::fabs(pitch);

        // 判断当前帧是否水平
        bool isFlat = (absRoll < FLAT_ROLL_THRESHOLD &&
                       absPitch < FLAT_PITCH_THRESHOLD);

        if (!isFlat) {
            wasClimbing_ = true;
            stableCount_ = 0;
        }

        if (isFlat && wasClimbing_) {
            stableCount_++;
            RCLCPP_INFO_THROTTLE(this->get_logger(),
                *this->get_clock(), 500,
                "检测到水平位置: roll=%.3f pitch=%.3f (稳定计数 %d/%d)",
                roll, pitch, stableCount_, STABLE_FRAMES_REQUIRED);
        }

        // 若曾爬过楼梯且连续水平帧数达到阈值，认为到达平地
        if (isFlat && wasClimbing_ && stableCount_ >= STABLE_FRAMES_REQUIRED) {
            go2_motion_stop();
            RCLCPP_INFO(this->get_logger(),
                "到达平地！roll=%.3f pitch=%.3f, 准备旋转45°", roll, pitch);

            state_ = StairState::ROTATING;
            rotateStartTime_ = this->now();
        }
    }

    /**
     * @brief 旋转阶段：逆时针旋转45°
     *
     * 使用 go2_motion_move 的 vyaw 参数控制角速度，
     * 通过计时控制旋转持续时间以达成45°旋转。
     */
    void handleRotating()
    {
        rclcpp::Time now = this->now();
        double elapsed = (now - rotateStartTime_).seconds();
        double rotateDuration = ROTATE_ANGLE / ROTATE_SPEED;

        if (elapsed < rotateDuration) {
            go2_motion_move(0.0f, 0.0f, ROTATE_SPEED);
        } else {
            go2_motion_stop();
            RCLCPP_INFO(this->get_logger(),
                "旋转完成 (%.1f°), 开始下楼梯: 速度=%.2f m/s, 距离=%.2f m",
                ROTATE_ANGLE * 180.0 / PI, DOWNSTAIR_VEL, DOWNSTAIR_DIST);

            go2_motion_move(DOWNSTAIR_VEL, 0.0f, 0.0f);
            state_ = StairState::GOING_DOWN;
            downStartTime_ = now;
        }
    }

    /**
     * @brief 下楼梯阶段：向前移动指定距离
     *
     * 通过计时控制前进距离：distance = speed * time
     */
    void handleGoingDown()
    {
        rclcpp::Time now = this->now();
        double elapsed = (now - downStartTime_).seconds();
        double distance = DOWNSTAIR_VEL * elapsed;

        if (distance < DOWNSTAIR_DIST) {
            // 继续前进
            RCLCPP_INFO_THROTTLE(this->get_logger(),
                *this->get_clock(), 500,
                "下楼梯中: 已前进 %.2f/%.2f m", distance, DOWNSTAIR_DIST);
        } else {
            go2_motion_stop();
            RCLCPP_INFO(this->get_logger(),
                "下楼梯完成 (前进 %.2f m)", distance);

            go2_motion_free_walk();
            RCLCPP_INFO(this->get_logger(), "已切换为行走模式 (FreeWalk)");

            state_ = StairState::DONE;
        }
    }

    /**
     * @brief 完成阶段：输出完成信息，标记程序结束
     */
    void handleDone()
    {
        RCLCPP_INFO(this->get_logger(),
            "+------------------------------------------------+\n"
            "|  上下楼梯任务完成！                              |\n"
            "|  已切换至行走模式 (FreeWalk)                     |\n"
            "+------------------------------------------------+");
        g_running = false;
    }

    // ---- 状态机成员 ----
    StairState    state_ = StairState::INIT;
    bool          wasClimbing_   = false;
    int           stableCount_   = 0;

    // ---- 计时成员 ----
    rclcpp::Time  rotateStartTime_;
    rclcpp::Time  downStartTime_;

    // ---- ROS2 接口 ----
    rclcpp::TimerBase::SharedPtr controlTimer_;
};

// ==========================================================================
// 主函数
// ==========================================================================

/**
 * @brief Go2 上下楼梯程序入口
 * @param argc 参数个数
 * @param argv 参数列表，argv[1] 为网络接口名称
 * @return int 0 正常退出，-1 异常退出
 */
int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "用法: " << argv[0] << " <network_interface>" << std::endl;
        std::cout << "示例: " << argv[0] << " eth0" << std::endl;
        return -1;
    }
    std::string netInterface = argv[1];

    // ---- 初始化 Go2 运动桥接 ----
    std::cout << "正在初始化 Go2 运动桥接 (接口: " << netInterface << ")..." << std::endl;
    if (!go2_motion_init(netInterface.c_str())) {
        std::cerr << "运动桥接初始化失败" << std::endl;
        return -1;
    }
    std::cout << "Go2 运动桥接已就绪" << std::endl;

    // ---- 初始化 ROS2 ----
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2WalkStairNode>();

    std::cout << "\n+------------------------------------------------+\n";
    std::cout <<   "|  Go2 上下楼梯程序                                |\n";
    std::cout <<   "+------------------------------------------------+\n";
    std::cout <<   "|  参数:                                          |\n";
    std::cout <<   "|    上楼梯速度: " << UPSTAIR_VEL << " m/s                              |\n";
    std::cout <<   "|    下楼梯速度: " << DOWNSTAIR_VEL << " m/s                             |\n";
    std::cout <<   "|    下楼梯距离: " << DOWNSTAIR_DIST << " m                            |\n";
    std::cout <<   "|    旋转角度:   " << ROTATE_ANGLE * 180.0 / PI << "°                           |\n";
    std::cout <<   "|    Pitch阈值:  " << FLAT_PITCH_THRESHOLD << " rad                      |\n";
    std::cout <<   "|    Roll阈值:   " << FLAT_ROLL_THRESHOLD << " rad                      |\n";
    std::cout <<   "+------------------------------------------------+\n";
    std::cout <<   "|  控制: [q] 退出程序                              |\n";
    std::cout <<   "+------------------------------------------------+\n";
    std::cout << "\nIMU 数据来源: go2_motion_bridge 订阅的 DDS 运动状态" << std::endl;
    std::cout << "（等价于 ROS2 /lowstate 节点的 imu_state 字段）" << std::endl;
    std::cout << "\n启动中..." << std::endl;

    // ---- 主循环 ----
    while (rclcpp::ok() && g_running) {
        rclcpp::spin_some(node);

        // 键盘检测（非阻塞）
        struct timeval tv = {0, 50000};
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        int ret = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
        if (ret > 0 && FD_ISSET(STDIN_FILENO, &fds)) {
            char ch;
            if (read(STDIN_FILENO, &ch, 1) == 1) {
                if (ch == 'q' || ch == 'Q') {
                    std::cout << "\n手动退出程序" << std::endl;
                    g_running = false;
                }
            }
        }
    }

    // ---- 清理 ----
    go2_motion_stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
