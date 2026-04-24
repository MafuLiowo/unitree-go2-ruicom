/**
 * @file go2_sport_interactive.cpp
 * @brief 交互式运动控制客户端，支持 Stretch 和 Hello 等运动指令
 */
#include <iostream>
#include <limits>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

/**
 * @brief Class to handle interactive sport commands for the Unitree Go2 robot.
 */
class Go2SportHandler
{
public:
    Go2SportHandler()
    {
        // Set the timeout for sport client commands (10 seconds)
        sport_client.SetTimeout(10.0f);
        // Initialize the sport client to prepare for sending commands
        sport_client.Init();
    }

    /**
     * @brief Performs the Hello (打招呼) action.
     */
    void Hello()
    {
        std::cout << "Action: Hello (打招呼)" << std::endl;
        sport_client.Hello();
    }

    /**
     * @brief Performs the Stretch (伸懒腰) action.
     */
    void Stretch()
    {
        std::cout << "Action: Stretch (伸懒腰)" << std::endl;
        sport_client.Stretch();
    }

private:
    unitree::robot::go2::SportClient sport_client;
};

int main(int argc, char **argv)
{
    std::string netInterface;
    // The robot requires a network interface (e.g., eth0, wlan0) to communicate
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        std::cout << "Example: " << argv[0] << " eth0" << std::endl;
        netInterface = argv[1];
        return -1;
    }

    // Initialize the Unitree SDK communication channel on the specified network interface

    if (!netInterface.empty()) {
        unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);
    } else {
        unitree::robot::ChannelFactory::Instance()->Init(0);
    }

    Go2SportHandler handler;

    std::cout << "Go2 Sport Interactive Client" << std::endl;
    std::cout << "----------------------------" << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << "1: Hello (打招呼)" << std::endl;
    std::cout << "2: Stretch (伸懒腰)" << std::endl;
    std::cout << "0: Exit" << std::endl;

    int input;
    while (true)
    {
        std::cout << "\nEnter command (0, 1, or 2): ";
        if (!(std::cin >> input)) {
            // Clear input buffer on error if non-numeric input is provided
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input. Please enter a valid number." << std::endl;
            continue;
        }

        if (input == 0) {
            std::cout << "Exiting..." << std::endl;
            break;
        } else if (input == 1) {
            handler.Hello();
        } else if (input == 2) {
            handler.Stretch();
        } else {
            std::cout << "Unknown command: " << input << ". Please enter 1 for Hello, 2 for Stretch, or 0 to Exit." << std::endl;
        }
    }

    return 0;
}
