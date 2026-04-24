/**
 * @file go2_light_controller.cpp
 * @brief 灯光控制客户端，控制 Go2 机器人 LED 灯效
 */
#include <iostream>
#include <unitree/robot/go2/vui/vui_client.hpp>
#include <unistd.h>
using namespace std;
int main(int argc, char** argv)
{
    string netInterface;
    if (argc > 1) {
        netInterface = argv[1];
    } else {
        std::cout << "Usage: " << argv[0] << " <network_interface>" << std::endl;
        std::cout << "Example: " << argv[0] << " enx00e04c36141b" << std::endl;
        return -1;
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);
    unitree::robot::go2::VuiClient lc;
    lc.Init();
    usleep(500000); //不知道是否有时间功能
    int level = 5;
    if(lc.SetBrightness(level)==0)
    {
        cout<<"亮度设置成功"<<"当前亮度为"<<level<<endl;
    }
    else
    {
        cout<<"亮度设置失败"<<endl<<"错误码:"<<lc.SetBrightness(level)<<endl;
    }
    usleep(500000);//可以多次for循环上面的程序来使灯光闪烁，达成一定节目效果
        return 0;
}