/**
 * @file go2_sport_switch.cpp
 * @brief Go2 机器人高层运动控制交互程序，支持 Stretch、Hello、StopMove、FrontJump 及 WalkStair 指令
 *
 * @par 使用说明
 *       go2_sport_switch <network_interface>
 *       示例: ./go2_sport_switch eth0
 *       进入交互菜单后选择对应编号执行动作:
 *           [1] Hello   (打招呼)
 *           [2] Stretch (伸懒腰)
 *           [3] StopMove (停止运动)
 *           [4] FrontJump (前跳)
 *           [0] 退出
 */
#include <iostream>
#include <limits>
#include "Go2SportSwitch.hpp"

int main(int argc, char **argv)
{
    // 解析命令行参数，获取网络接口名称
    std::string netInterface;
    if (argc < 2)
    {
        std::cout << "用法: " << argv[0] << " <network_interface>" << std::endl;
        std::cout << "示例: " << argv[0] << " eth0" << std::endl;
        return -1;
    }
    netInterface = argv[1];

    // 初始化 Unitree SDK2 通信通道
    if (!netInterface.empty()) {
        unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);
    } else {
        unitree::robot::ChannelFactory::Instance()->Init(0);
    }

    // 创建运动控制处理器实例
    Go2SportSwitch sport;
    std::cout << "Go2 Sport Switch Client 已就绪" << std::endl;
    std::cout << "-------------------------------" << std::endl;
    std::cout << "指令菜单:" << std::endl;
    std::cout << "  1: Hello    (打招呼)" << std::endl;
    std::cout << "  2: Stretch  (伸懒腰)" << std::endl;
    std::cout << "  3: StopMove (停止运动)" << std::endl;
    std::cout << "  4: FrontJump(前跳)" << std::endl;
    std::cout << "  0: 退出" << std::endl;

    // 交互式命令循环
    int input;
    while (true)
    {
        std::cout << "\n请输入指令编号: ";
        if (!(std::cin >> input)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "输入无效，请输入数字。" << std::endl;
            continue;
        }

        switch (input)
        {
        case 0:
            std::cout << "退出程序..." << std::endl;
            return 0;
        case 1:
            sport.Hello();
            break;
        case 2:
            sport.Stretch();
            break;
        case 3:
            sport.StopMove();
            break;
        case 4:
            sport.FrontJump();
            break;
        default:
            std::cout << "未知指令: " << input
                      << "，请输入 1-4 或 0 退出。" << std::endl;
            break;
        }
    }

    return 0;
}
