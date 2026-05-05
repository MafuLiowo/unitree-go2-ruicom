/**
 * @file go2_light_controller.cpp
 * @brief 灯光控制客户端，控制 Go2 机器人前灯闪烁，每次亮 0.8s、间隔 0.5s，共闪烁 3 次
 *
 * @par 使用说明
 *       go2_light_controller <network_interface>
 *       示例: ./go2_light_controller eth0
 *       说明: 程序自动控制前灯闪烁 3 次，可通过修改 Go2LightController 接口参数调整
 */
#include <iostream>
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
    light.Blink(3, 0.8f, 0.5f);

    return 0;
}
