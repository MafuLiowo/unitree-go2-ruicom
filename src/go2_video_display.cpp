/**
 * @file go2_video_display.cpp
 * @brief Go2 机器人摄像头实时画面显示，在窗口中弹出当前机器人摄像头内容
 *
 * @par 使用说明
 *       go2_video_display <network_interface>
 *       示例: ./go2_video_display eth0
 *       控制: [s] 保存当前帧  [q/Esc] 退出程序
 */
#include <unitree/robot/go2/video/video_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <ctime>
#include <iomanip>
#include <sstream>

/**
 * @brief 获取当前时间戳字符串，用于构造保存图片的文件名
 * @return std::string 格式为 "YYYYMMDD_HHMMSS" 的时间戳字符串
 */
std::string getCurrentTimestamp() {
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

/**
 * @brief Go2 摄像头实时画面显示程序主函数
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

    unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);

    unitree::robot::go2::VideoClient video_client;
    video_client.SetTimeout(1.0f);
    video_client.Init();

    std::vector<uint8_t> image_sample;
    int save_counter = 0;

    cv::namedWindow("Go2 Camera", cv::WINDOW_NORMAL);
    std::cout << "控制说明: [s] 保存图片 | [q/Esc] 退出程序" << std::endl;

    while (true) {
        int ret = video_client.GetImageSample(image_sample);

        if (ret == 0 && !image_sample.empty()) {
            cv::Mat rawData(image_sample);
            cv::Mat frame = cv::imdecode(rawData, cv::IMREAD_COLOR);

            if (!frame.empty()) {
                cv::Mat display;
                cv::resize(frame, display, cv::Size(), 3.0, 3.0, cv::INTER_LINEAR);
                cv::imshow("Go2 Camera", display);

                char key = (char)cv::waitKey(1);
                if (key == 'q' || key == 27) {
                    break;
                }
                else if (key == 's' || key == 'S') {
                    // 构造带时间戳的文件名，确保多次保存不会覆盖
                    std::string filename = "go2_display_" + getCurrentTimestamp() + "_" + std::to_string(save_counter++) + ".jpg";

                    if (cv::imwrite(filename, frame)) {
                        std::cout << ">>> 已成功保存图片: " << filename << " (共捕获 " << save_counter << " 张)" << std::endl;
                    } else {
                        std::cerr << "!!! 保存失败: " << filename << std::endl;
                    }
                }
            }
        } else {
            // 未获取到图像时仍可响应退出按键
            if ((char)cv::waitKey(1) == 'q') break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
