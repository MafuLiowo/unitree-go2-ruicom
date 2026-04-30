/**
 * @file go2_process_images_main.cpp
 * @brief Go2 机器人图像处理独立显示程序，实时分析路径中点并显示在双窗口中
 *
 * @par 使用说明
 *       ./go2_process_images <network_interface>
 *       示例: ./go2_process_images eth0
 *       控制: [q/Esc] 退出程序
 */
#include <unitree/robot/go2/video/video_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <opencv2/opencv.hpp>
#include "ImageProcessor.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <cstdint>

/**
 * @brief Go2 图像处理显示程序主函数
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

    std::cout << "========================================" << std::endl;
    std::cout << "Go2 Image Processing Display" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Network interface: " << netInterface << std::endl;
    std::cout << "Press [q] or [Esc] to exit." << std::endl;

    unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);

    unitree::robot::go2::VideoClient videoClient;
    videoClient.SetTimeout(1.0f);
    videoClient.Init();

    cv::namedWindow("Go2 Camera", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("High Contrast Analysis", cv::WINDOW_AUTOSIZE);

    std::vector<uint8_t> imageSample;
    int frameCount = 0;

    while (true) {
        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) break;

        int ret = videoClient.GetImageSample(imageSample);
        if (ret != 0 || imageSample.empty()) continue;

        cv::Mat rawData(imageSample);
        cv::Mat frame = cv::imdecode(rawData, cv::IMREAD_COLOR);
        if (frame.empty()) continue;

        frameCount++;

        cv::Mat highContrast = toHighContrast(frame, 70);
        PathAnalysis analysis = analyzePath(highContrast);

        cv::Mat rawDisplay = frame.clone();
        if (analysis.pathFound && !analysis.displayImage.empty()) {
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

        char frameBuf[32];
        snprintf(frameBuf, sizeof(frameBuf), "Frame: %d", frameCount);
        cv::putText(rawDisplay, frameBuf, cv::Point(rawDisplay.cols - 150, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        cv::imshow("Go2 Camera", rawDisplay);
        cv::imshow("High Contrast Analysis", analysis.displayImage);

        if (frameCount % 100 == 0) {
            std::cout << "Processed " << frameCount << " frames" << std::endl;
        }
    }

    cv::destroyAllWindows();
    std::cout << "\nProgram terminated. Total frames: " << frameCount << std::endl;
    return 0;
}
