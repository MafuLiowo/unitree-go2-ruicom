/**
 * @file go2_light_controller.cpp
 * @brief 灯光控制客户端，通过 SetBrightness 调节 Go2 机器人前灯亮度：亮度 5 持续 1s、亮度 0 持续 0.7s，循环 3 次
 *
 * @par 使用说明
 *       go2_light_controller <network_interface>
 *       示例: ./go2_light_controller eth0
 *       说明: 程序使用 SDK 灯光服务接口 SetBrightness 函数，自动控制前灯亮度循环变化
 */
#include <iostream>
#include <unistd.h>
#include <unitree/robot/channel/channel_factory.hpp>
#include "Go2LightController.hpp"

int main(int argc, char** argv)
{
    std::string netInterface;
    if (argc > 1) {
        netInterface = argv[1];
    } else {
        std::cout << "Usage: " << argv[0] << " <network_interface>" << std::endl;
        std::cout << "Example: " << argv[0] << " enx00e04c36141b" << std::endl;
        return -1;
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);

    Go2LightController light;

    const int kCycles = 3;
    const int kBrightLevel = 5;
    const int kDarkLevel = 0;
    const float kOnDurationSec = 1.0f;
    const float kOffDurationSec = 0.7f;

    std::cout << ">>> 开始循环亮度调节: 共 " << kCycles << " 次"
              << "，亮度 " << kBrightLevel << " 持续 " << kOnDurationSec
              << "s，亮度 " << kDarkLevel << " 持续 " << kOffDurationSec
              << "s" << std::endl;

    for (int i = 0; i < kCycles; i++)
    {
        std::cout << "第 " << (i + 1) << " 次: SetBrightness(" << kBrightLevel << ")" << std::endl;
        light.SetBrightness(kBrightLevel);
        usleep(static_cast<useconds_t>(kOnDurationSec * 1000000));

        std::cout << "第 " << (i + 1) << " 次: SetBrightness(" << kDarkLevel << ")" << std::endl;
        light.SetBrightness(kDarkLevel);
        usleep(static_cast<useconds_t>(kOffDurationSec * 1000000));
    }

    std::cout << ">>> 循环亮度调节结束" << std::endl;

    return 0;
}
