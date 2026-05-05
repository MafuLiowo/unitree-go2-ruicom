/**
 * @file Go2SportSwitch.hpp
 * @brief Go2 机器人高层运动控制封装，提供 Stretch、Hello、StopMove 及 FrontJump 等运动指令接口
 *
 * @par 使用说明
 *       - 本模块为库文件，封装 Unitree SDK2 的 SportClient，供其他程序直接调用
 *       - 使用时需先调用 Init() 初始化通信通道，再调用具体运动指令
 *       - 典型用法：
 *         @code
 *         unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
 *         Go2SportSwitch sport;
 *         sport.Stretch();
 *         sport.Hello();
 *         @endcode
 */
#pragma once

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <iostream>

/**
 * @brief Go2 机器人高层运动控制类，封装常用运动指令
 *
 * 本类对 Unitree SDK2 的 SportClient 进行封装，提供语义化的运动控制接口，
 * 方便在上层应用（如图像识别、自动巡检等）中直接调用预设动作。
 */
class Go2SportSwitch
{
public:
    /**
     * @brief 构造函数，初始化运动客户端超时时间并完成初始化
     */
    Go2SportSwitch()
    {
        sport_client_.SetTimeout(10.0f);
        sport_client_.Init();
    }

    /**
     * @brief 析构函数
     */
    ~Go2SportSwitch() = default;

    /**
     * @brief 执行打招呼（Hello）动作
     *
     * 机器狗举起前腿做出招手/打招呼的姿态，常用于展示或与观众互动。
     */
    void Hello()
    {
        std::cout << ">>> 执行动作: Hello (打招呼)" << std::endl;
        sport_client_.Hello();
    }

    /**
     * @brief 执行伸懒腰（Stretch）动作
     *
     * 机器狗四肢伸展，做出类似伸懒腰的姿态。
     */
    void Stretch()
    {
        std::cout << ">>> 执行动作: Stretch (伸懒腰)" << std::endl;
        sport_client_.Stretch();
    }

    /**
     * @brief 停止机器狗当前运动
     *
     * 发送停止指令使机器狗立即停止所有正在进行的运动，恢复到站立状态。
     * 适用于紧急停止或运动切换前的中止操作。
     */
    void StopMove()
    {
        std::cout << ">>> 执行动作: StopMove (停止运动)" << std::endl;
        sport_client_.StopMove();
    }

    /**
     * @brief 执行前跳（FrontJump）动作
     *
     * 机器狗向前方跳跃，适用于越障或跨越小沟壑等场景。
     */
    void FrontJump()
    {
        std::cout << ">>> 执行动作: FrontJump (前跳)" << std::endl;
        sport_client_.FrontJump();
    }


private:
    unitree::robot::go2::SportClient sport_client_; ///< Unitree SDK2 运动客户端实例
};
