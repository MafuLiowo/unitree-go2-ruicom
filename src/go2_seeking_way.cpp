/**
 * @file go2_seeking_way.cpp
 * @brief Go2 机器人视觉寻路系统，基于连通域分析定位白色路径中点并控制前进方向
 *
 * @par 使用说明
 *       ./go2_seeking_way <network_interface>
 *       示例: ./go2_seeking_way eth0
 *       控制: [q/Esc] 退出程序
 *
 * @par 工作原理
 *       - 从Go2摄像头获取实时画面
 *       - 通过 go2_process_images 的连通域分析算法提取高对比度路径（白色=可行路径）：
 *         在近端区域内找出所有连续白色连通域，选取最靠近图像中央者，
 *         以其质心横坐标作为路径中点
 *       - 定位该中点位置，使用比例控制器调整偏航角度持续前进
 *       - 检测90度拐弯，判断拐弯方向，先前进一定距离再旋转到该方向继续前进
 */
#include <unitree/robot/go2/video/video_client.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <opencv2/opencv.hpp>
#include "ImageProcessor.hpp"

#include <iostream>
#include <string>
#include <cmath>

/**
 * @brief 寻路状态机枚举，定义机器人在视觉寻路过程中的各个阶段
 */
enum class SeekState {
    FORWARD,        // 正常前进，沿路径中点调整偏航角
    TURN_DETECTED,  // 检测到转弯点，准备靠近转弯位置
    TURN_APPROACH,  // 预留：靠近转弯点后的过渡状态
    TURN_ROTATING   // 正在旋转 90 度，完成后回到 FORWARD 状态
};

/**
 * @brief Go2 视觉寻路系统主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组，argv[1] 为网络接口名称（如 eth0）
 * @return int 程序退出码，0 表示正常退出，-1 表示参数错误
 */
int main(int argc, char** argv)
{
    std::string netInterface;
    if (argc > 1) {
        netInterface = argv[1];
    } else {
        std::cout << "Usage: " << argv[0] << " <network_interface>" << std::endl;
        std::cout << "Example: " << argv[0] << " eth0" << std::endl;
        return -1;
    }

    // 运动控制与检测参数
    const float forwardSpeed = 0.2f;             // 前进线速度 (m/s)
    const float maxYawSpeed = 0.5f;              // 最大偏航角速度 (rad/s)
    const float kP = 1.8f;                       // 比例控制器增益系数（连通域中点更稳定，可适当提高）
    const float turnApproachDistance = 0.25f;    // 转弯前的前进补偿距离 (m)
    const double turnCooldownSec = 4.0;          // 两次转弯之间的冷却时间 (s)
    const int turnDetectionFrames = 4;           // 连续检测到转弯的帧数阈值

    std::cout << "========================================" << std::endl;
    std::cout << "Go2 Visual Path Seeking System" << std::endl;
    std::cout << "Algorithm: blob-centroid path following" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Network interface: " << netInterface << std::endl;
    std::cout << "Forward speed: " << forwardSpeed << " m/s" << std::endl;
    std::cout << "Max yaw speed: " << maxYawSpeed << " rad/s" << std::endl;
    std::cout << "P gain: " << kP << std::endl;
    std::cout << "Turn approach: " << turnApproachDistance << " m" << std::endl;
    std::cout << "========================================" << std::endl;

    unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);

    // 初始化视频客户端，用于获取 Go2 摄像头实时画面
    unitree::robot::go2::VideoClient videoClient;
    videoClient.SetTimeout(1.0f);
    videoClient.Init();

    // 初始化避障运动控制客户端，启用 API 远程指令模式
    unitree::robot::go2::ObstaclesAvoidClient oaClient;
    oaClient.SetTimeout(10.0f);
    oaClient.Init();
    oaClient.SwitchSet(true);
    oaClient.UseRemoteCommandFromApi(true);

    std::cout << "Clients initialized. Starting main loop." << std::endl;
    std::cout << "Press [q] or [Esc] to exit." << std::endl;

    cv::namedWindow("Go2 Camera", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("High Contrast Analysis", cv::WINDOW_AUTOSIZE);

    std::vector<uint8_t> imageSample;

    SeekState state = SeekState::FORWARD;
    int detectedTurnDirection = 0;
    int turnDetectionCount = 0;
    double lastActionTime = 0.0;

    int frameCount = 0;
    bool userConfirmed = false;

    while (true) {
        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }

        int ret = videoClient.GetImageSample(imageSample);
        if (ret != 0 || imageSample.empty()) {
            continue;
        }

        cv::Mat rawData(imageSample);
        cv::Mat frame = cv::imdecode(rawData, cv::IMREAD_COLOR);
        if (frame.empty()) continue;

        frameCount++;
        // 高对比度二值化 + 连通域路径分析
        cv::Mat highContrast = toHighContrast(frame, 70);
        PathAnalysis analysis = analyzePath(highContrast);

        double currentTime = (double)cv::getTickCount() / cv::getTickFrequency();

        switch (state) {
        case SeekState::FORWARD: {
            // 等待用户确认后才开始运动
            if (!userConfirmed) {
                oaClient.Move(0.0f, 0.0f, 0.0f);
                break;
            }
            if (analysis.pathFound) {
                // 比例控制：根据连通域中点偏移量计算偏航角速度
                float error = 0.5f - analysis.midpointX;
                float yawCmd = kP * error;
                yawCmd = std::max(-maxYawSpeed, std::min(maxYawSpeed, yawCmd));
                oaClient.Move(forwardSpeed, 0.0f, yawCmd);

                // 累积检测转弯帧数，连续满足条件时触发转弯状态切换
                if (analysis.isTurn &&
                    (currentTime - lastActionTime) > turnCooldownSec &&
                    turnDetectionCount < turnDetectionFrames) {
                    turnDetectionCount++;
                    if (turnDetectionCount >= turnDetectionFrames) {
                        detectedTurnDirection = analysis.turnDirection;
                        state = SeekState::TURN_DETECTED;
                        turnDetectionCount = 0;
                        lastActionTime = currentTime;
                        std::cout << "[TURN DETECTED] Direction: "
                                  << (detectedTurnDirection < 0 ? "LEFT" : "RIGHT")
                                  << "  Mid: " << analysis.midpointX << std::endl;
                    }
                } else if (!analysis.isTurn) {
                    turnDetectionCount = 0;
                }
            } else {
                // 未检测到路径时停止运动
                oaClient.Move(0.0f, 0.0f, 0.0f);
                turnDetectionCount = 0;
            }
            break;
        }
        case SeekState::TURN_DETECTED: {
            // 先停止，再以增量方式前进到转弯点附近
            oaClient.Move(0.0f, 0.0f, 0.0f);
            std::cout << "[TURN] Approaching turn point... moving forward "
                      << turnApproachDistance << " m" << std::endl;
            oaClient.MoveToIncrementPosition(turnApproachDistance, 0.0f, 0.0f);
            state = SeekState::TURN_ROTATING;
            break;
        }
        case SeekState::TURN_ROTATING: {
            // 根据检测到的转弯方向旋转约 90 度
            float turnAngle = (detectedTurnDirection > 0) ? -(float)M_PI_2 : (float)M_PI_2;
            std::cout << "[TURN] Rotating " << (detectedTurnDirection > 0 ? "RIGHT" : "LEFT")
                      << " 90 degrees..." << std::endl;
            oaClient.MoveToIncrementPosition(0.0f, 0.0f, turnAngle);
            std::cout << "[TURN] Rotation complete. Resuming path following." << std::endl;
            state = SeekState::FORWARD;
            detectedTurnDirection = 0;
            break;
        }
        }

        // 在原图上绘制路径中点指示线与状态信息
        cv::Mat rawDisplay = frame.clone();
        if (analysis.pathFound) {
            int midX = (int)(analysis.midpointX * (float)rawDisplay.cols);
            cv::line(rawDisplay, cv::Point(midX, 0), cv::Point(midX, rawDisplay.rows),
                     cv::Scalar(0, 255, 0), 2);

            cv::putText(rawDisplay, "PATH (blob centroid)", cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            char buf[64];
            snprintf(buf, sizeof(buf), "Mid: %.3f  White: %.2f%%",
                     analysis.midpointX, analysis.whiteRatio * 100.0f);
            cv::putText(rawDisplay, buf, cv::Point(10, 55),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

            // 显示各区域白色占比
            char regionBuf[96];
            snprintf(regionBuf, sizeof(regionBuf), "L:%.2f%% C:%.2f%% R:%.2f%%",
                     analysis.leftWhiteRatio * 100.0f,
                     analysis.centerWhiteRatio * 100.0f,
                     analysis.rightWhiteRatio * 100.0f);
            cv::putText(rawDisplay, regionBuf, cv::Point(10, 80),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        } else {
            cv::putText(rawDisplay, "NO PATH", cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }

        const char* stateStr = "";
        switch (state) {
        case SeekState::FORWARD:       stateStr = "FORWARD"; break;
        case SeekState::TURN_DETECTED: stateStr = "TURN_DETECTED"; break;
        case SeekState::TURN_APPROACH: stateStr = "TURN_APPROACH"; break;
        case SeekState::TURN_ROTATING: stateStr = "TURN_ROTATING"; break;
        }
        cv::putText(rawDisplay, stateStr, cv::Point(10, 110),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

        char frameBuf[32];
        snprintf(frameBuf, sizeof(frameBuf), "Frame: %d", frameCount);
        cv::putText(rawDisplay, frameBuf, cv::Point(rawDisplay.cols - 150, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        cv::imshow("Go2 Camera", rawDisplay);
        cv::imshow("High Contrast Analysis", analysis.displayImage);

        // 首次检测到路径时暂停，等待用户确认后再开始运动
        if (!userConfirmed && analysis.pathFound) {
            cv::waitKey(1);
            std::cout << "\n[CONFIRM] Path detected at frame " << frameCount << "!" << std::endl;
            std::cout << "[CONFIRM] Blob midpoint: " << analysis.midpointX
                      << ", white ratio: " << (analysis.whiteRatio * 100.0f) << "%" << std::endl;
            std::cout << "[CONFIRM] Start walking? (y/n): ";
            std::string input;
            std::getline(std::cin, input);
            if (input == "y" || input == "Y") {
                userConfirmed = true;
                std::cout << "[CONFIRM] Starting path following." << std::endl;
            } else {
                std::cout << "[CONFIRM] User declined. Exiting." << std::endl;
                break;
            }
        }

        if (frameCount % 100 == 0) {
            std::cout << "Processed " << frameCount << " frames, state: "
                      << stateStr
                      << (analysis.pathFound ? ", mid: " + std::to_string(analysis.midpointX) : "")
                      << std::endl;
        }
    }

    oaClient.Move(0.0f, 0.0f, 0.0f);
    cv::destroyAllWindows();
    std::cout << "\nProgram terminated. Total frames: " << frameCount << std::endl;
    return 0;
}
