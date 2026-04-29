/**
 * @file go2_video_display.cpp
 * @brief Go2 机器人摄像头实时画面显示，在窗口中弹出当前机器人摄像头内容
 *
 * @par 使用说明
 *       go2_video_display <network_interface>
 *       示例: ./go2_video_display eth0
 *       控制: [q/Esc] 退出程序
 */
#include <unitree/robot/go2/video/video_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

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

    cv::namedWindow("Go2 Camera", cv::WINDOW_AUTOSIZE);
    std::cout << "按 [q] 或 [Esc] 退出程序" << std::endl;

    while (true) {
        int ret = video_client.GetImageSample(image_sample);

        if (ret == 0 && !image_sample.empty()) {
            cv::Mat rawData(image_sample);
            cv::Mat frame = cv::imdecode(rawData, cv::IMREAD_COLOR);

            if (!frame.empty()) {
                cv::imshow("Go2 Camera", frame);
            }
        }

        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
