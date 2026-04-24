/**
 * @file go2_obstacles_avoid.cpp
 * @brief 障碍物规避客户端，配置并启用 Go2 机器人避障功能
 */
#include <iostream>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_api.hpp>
#include <thread>
#include <chrono>
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
    unitree::robot::go2::ObstaclesAvoidClient sc;
    sc.Init();
    sc.SetTimeout(5.0f);
    bool enable=true;
    sc.SwitchSet(enable);//开启避障功能
    sc.UseRemoteCommandFromApi(enable);//抢夺遥控器
    if(sc.SwitchGet(enable)==0)
    {
        cout<<"避障功能已开启"<<endl;
    }
    else
    {
        cout<<"避障功能开启失败"<<endl<<"错误码:"<<sc.SwitchGet(enable)<<endl;
    }
    // while (enable)
    // {
    //     int ret = sc.GetObstaclesAvoidData();
    //     if (ret == 0) {
    //         auto data = sc.GetObstaclesAvoidData();
    //         cout << "障碍物距离: " << data.obstacle_distance << " 米" << endl;
    //         cout << "障碍物角度: " << data.obstacle_angle << " 度" << endl;
    //     }
    //     else {
    //         cout << "获取数据失败，错误码: " << ret << endl;
    //     }
    //     this_thread::sleep_for(chrono::milliseconds(100));
    // }
    //   sc.Move(1.0,0.0,0.0);//以1m的速度前进，自动避障
    sc.MoveToIncrementPosition(1.0, 0.0, 0.0);//走两步，自动避障
    // while (true)
    // {
    //   usleep(1000000);
    //   if(tem<=5)
    //   {
    //     tem++;
    //   }
    //   else
    //   {
    //     sc.UseRemoteCommandFromApi(false);//5秒后关闭
    //     sc.SwitchSet(false);//5秒后关闭
    //   }
    // }
    // 定时关闭避障功能，暂时还不知道有什么用

    return 0;
}