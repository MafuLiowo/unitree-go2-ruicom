/**
 * @file Go2LightController.hpp
 * @brief Go2 机器人前灯控制封装，提供前灯闪烁等功能接口
 *
 * @par 使用说明
 *       - 本模块为头文件，封装 Unitree SDK2 的 VuiClient，供其他程序直接调用
 *       - 使用时需先在 main 中调用 ChannelFactory::Init() 初始化通信通道
 *       - 典型用法：
 *         @code
 *         unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
 *         Go2LightController light;
 *         light.Blink(3, 1.0f, 0.7f);  // 闪烁3次，亮度5持续1s，亮度0持续0.7s
 *         @endcode
 */
#pragma once

#include <unitree/robot/go2/vui/vui_client.hpp>
#include <iostream>
#include <unistd.h>

/**
 * @brief Go2 机器人前灯控制类，封装常用灯光指令
 *
 * 本类对 Unitree SDK2 的 VuiClient 进行封装，提供前灯闪烁等灯光控制接口，
 * 方便在上层应用（如状态指示、报警提示等）中直接调用。
 */
class Go2LightController
{
public:
    /**
     * @brief 构造函数，初始化 VuiClient 并设置默认亮度级别
     */
    Go2LightController()
        : brightness_level_(5)
    {
        vui_client_.Init();
    }

    /**
     * @brief 析构函数
     */
    ~Go2LightController() = default;

    /**
     * @brief 设置前灯亮度级别
     * @param level 亮度级别，范围取决于硬件支持（通常 0~10）
     * @return int32_t 0 表示成功，非 0 表示失败
     */
    int32_t SetBrightness(int level)
    {
        int32_t ret = vui_client_.SetBrightness(level);
        if (ret == 0)
        {
            brightness_level_ = level;
        }
        return ret;
    }

    /**
     * @brief 关闭前灯（亮度设为 0）
     * @return int32_t 0 表示成功，非 0 表示失败
     */
    int32_t TurnOff()
    {
        return SetBrightness(0);
    }

    /**
     * @brief 打开前灯（恢复为之前设置的亮度级别）
     * @return int32_t 0 表示成功，非 0 表示失败
     */
    int32_t TurnOn()
    {
        return SetBrightness(brightness_level_);
    }

    /**
     * @brief 前灯闪烁指定次数，直接通过 SetBrightness 调节亮度
     * @param times 闪烁次数
     * @param onDuration 每次灯光持续亮起的时间（秒）
     * @param offInterval 每次灯光熄灭的间隔时间（秒）
     */
    void Blink(int times, float onDuration, float offInterval)
    {
        int onLevel = 5;
        int offLevel = 0;
        std::cout << ">>> 前灯闪烁开始: 共 " << times << " 次，亮度 " << onLevel
                  << " 持续 " << onDuration << "s，亮度 " << offLevel
                  << " 持续 " << offInterval << "s" << std::endl;
        for (int i = 0; i < times; i++)
        {
            std::cout << "第 " << (i + 1) << " 次闪烁: 亮度设为 " << onLevel << std::endl;
            if (SetBrightness(onLevel) == 0)
            {
                brightness_level_ = onLevel;
            }
            usleep(static_cast<useconds_t>(onDuration * 1000000));
            std::cout << "第 " << (i + 1) << " 次闪烁: 亮度设为 " << offLevel << std::endl;
            SetBrightness(offLevel);
            usleep(static_cast<useconds_t>(offInterval * 1000000));
        }
        std::cout << ">>> 前灯闪烁结束" << std::endl;
    }

private:
    unitree::robot::go2::VuiClient vui_client_; ///< Unitree SDK2 VUI 客户端实例
    int brightness_level_;                       ///< 当前亮度级别
};
