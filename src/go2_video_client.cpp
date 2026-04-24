/**
 * @file go2_video_client.cpp
 * @brief Unitree Go2 机器人视频流客户端，实时拉取并显示相机画面
 */
#include <unitree/robot/go2/video/video_client.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <ctime>
#include <iomanip>
#include <sstream>

std::string getCurrentTimestamp() {
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

int main(int argc, char** argv)
{
    std::string netInterface;
    if (argc > 1) {
        netInterface = argv[1];
    }

    if (!netInterface.empty()) {
        unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);
    } else {
        unitree::robot::ChannelFactory::Instance()->Init(0);
    }

    unitree::robot::go2::VideoClient video_client;
    video_client.SetTimeout(1.0f);
    video_client.Init();

    std::vector<uint8_t> image_sample;
    int ret;
    int save_counter = 0;

    cv::namedWindow("Go2 Real-time Video", cv::WINDOW_AUTOSIZE);
    std::cout << "控制说明: [s] 保存图片 | [q/Esc] 退出程序" << std::endl;

    while (true)
    {
        ret = video_client.GetImageSample(image_sample);

        if (ret == 0 && !image_sample.empty()) {
            cv::Mat rawData(image_sample);
            cv::Mat frame = cv::imdecode(rawData, cv::IMREAD_COLOR);

            if (!frame.empty()) {
                cv::imshow("Go2 Real-time Video", frame);

                char key = (char)cv::waitKey(1);

                if (key == 'q' || key == 27) {
                    break;
                }
                else if (key == 's' || key == 'S') {
                    std::string filename = "go2_capture_" + getCurrentTimestamp() + "_" + std::to_string(save_counter++) + ".jpg";

                    if (cv::imwrite(filename, frame)) {
                        std::cout << ">>> 已成功保存图片: " << filename << " (共捕获 " << save_counter << " 张)" << std::endl;

                        cv::Mat feedback = frame.clone();
                        cv::putText(feedback, "SAVED!", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                        cv::imshow("Go2 Real-time Video", feedback);
                        cv::waitKey(200);
                    } else {
                        std::cerr << "!!! 保存失败: " << filename << std::endl;
                    }
                }
            }
        } else {
            if ((char)cv::waitKey(1) == 'q') break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}