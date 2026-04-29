/**
 * @file go2_seeking_way.cpp
 * @brief Go2 机器人视觉寻路系统，基于高对比度图像分析白色路径中点并控制前进方向
 *
 * @par 使用说明
 *       ./go2_seeking_way <network_interface>
 *       示例: ./go2_seeking_way eth0
 *       控制: [q/Esc] 退出程序
 *
 * @par 工作原理
 *       - 从Go2摄像头获取实时画面
 *       - 通过go2_process_images的图像处理逻辑提取高对比度路径（白色=可行路径）
 *       - 定位横向白色部分的中点，朝着中点方向调整偏航角度持续前进
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

enum class SeekState {
    FORWARD,
    TURN_DETECTED,
    TURN_APPROACH,
    TURN_ROTATING
};

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

    const float forwardSpeed = 0.2f;
    const float maxYawSpeed = 0.5f;
    const float kP = 1.6f;
    const float turnApproachDistance = 0.25f;
    const double turnCooldownSec = 4.0;
    const int turnDetectionFrames = 4;

    std::cout << "========================================" << std::endl;
    std::cout << "Go2 Visual Path Seeking System" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Network interface: " << netInterface << std::endl;
    std::cout << "Forward speed: " << forwardSpeed << " m/s" << std::endl;
    std::cout << "Max yaw speed: " << maxYawSpeed << " rad/s" << std::endl;
    std::cout << "Turn approach: " << turnApproachDistance << " m" << std::endl;
    std::cout << "========================================" << std::endl;

    unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);

    unitree::robot::go2::VideoClient videoClient;
    videoClient.SetTimeout(1.0f);
    videoClient.Init();

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
        cv::Mat highContrast = toHighContrast(frame, 70);
        PathAnalysis analysis = analyzePath(highContrast);

        double currentTime = (double)cv::getTickCount() / cv::getTickFrequency();

        switch (state) {
        case SeekState::FORWARD: {
            if (!userConfirmed) {
                oaClient.Move(0.0f, 0.0f, 0.0f);
                break;
            }
            if (analysis.pathFound) {
                float error = 0.5f - analysis.midpointX;
                float yawCmd = kP * error;
                yawCmd = std::max(-maxYawSpeed, std::min(maxYawSpeed, yawCmd));
                oaClient.Move(forwardSpeed, 0.0f, yawCmd);

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
                                  << (detectedTurnDirection < 0 ? "LEFT" : "RIGHT") << std::endl;
                    }
                } else if (!analysis.isTurn) {
                    turnDetectionCount = 0;
                }
            } else {
                oaClient.Move(0.0f, 0.0f, 0.0f);
                turnDetectionCount = 0;
            }
            break;
        }
        case SeekState::TURN_DETECTED: {
            oaClient.Move(0.0f, 0.0f, 0.0f);
            std::cout << "[TURN] Approaching turn point... moving forward "
                      << turnApproachDistance << " m" << std::endl;
            oaClient.MoveToIncrementPosition(turnApproachDistance, 0.0f, 0.0f);
            state = SeekState::TURN_ROTATING;
            break;
        }
        case SeekState::TURN_ROTATING: {
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

        cv::Mat rawDisplay = frame.clone();
        if (analysis.pathFound) {
            int midX = (int)(analysis.midpointX * (float)rawDisplay.cols);
            cv::line(rawDisplay, cv::Point(midX, 0), cv::Point(midX, rawDisplay.rows),
                     cv::Scalar(0, 255, 0), 2);

            cv::putText(rawDisplay, "PATH FOUND", cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            char buf[64];
            snprintf(buf, sizeof(buf), "Mid: %.2f", analysis.midpointX);
            cv::putText(rawDisplay, buf, cv::Point(10, 55),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
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
        cv::putText(rawDisplay, stateStr, cv::Point(10, 80),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

        char frameBuf[32];
        snprintf(frameBuf, sizeof(frameBuf), "Frame: %d", frameCount);
        cv::putText(rawDisplay, frameBuf, cv::Point(rawDisplay.cols - 150, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        cv::imshow("Go2 Camera", rawDisplay);
        cv::imshow("High Contrast Analysis", analysis.displayImage);

        if (!userConfirmed && analysis.pathFound) {
            cv::waitKey(1);
            std::cout << "\n[CONFIRM] Path detected at frame " << frameCount << "!" << std::endl;
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
                      << stateStr << std::endl;
        }
    }

    oaClient.Move(0.0f, 0.0f, 0.0f);
    cv::destroyAllWindows();
    std::cout << "\nProgram terminated. Total frames: " << frameCount << std::endl;
    return 0;
}
