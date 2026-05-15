/**
 * @file go2_photo.cpp
 * @brief Go2 原生相机拍照程序，实时显示摄像头画面，通过终端命令控制
 *
 * @par 使用说明
 *       go2_photo [network_interface]
 *       示例: ./go2_photo              # 默认网口
 *             ./go2_photo eth0         # 指定以太网接口
 *       控制: 终端输入 s 保存图片到 ./photo/ 目录，输入 q 退出
 *       图片格式: IMG_xxx.jpg (自动检测已有编号，不覆盖)
 */
#include <unitree/robot/go2/video/video_client.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <poll.h>
#include <unistd.h>

namespace fs = std::filesystem;

/**
 * @brief 获取 ./photo/ 目录中下一个可用的图片编号
 * @return int 下一个编号（从 1 开始），保证对应文件尚不存在
 */
static int getNextPhotoNumber() {
    fs::create_directories("./photo");

    int max_num = 0;
    for (const auto& entry : fs::directory_iterator("./photo")) {
        std::string stem = entry.path().stem().string();
        std::string ext  = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext != ".jpg") continue;
        if (stem.size() > 4 && stem.substr(0, 4) == "IMG_") {
            std::string num_str = stem.substr(4);
            try {
                int num = std::stoi(num_str);
                if (num > max_num) max_num = num;
            } catch (...) {}
        }
    }
    return max_num + 1;
}

int main(int argc, char** argv) {
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

    std::cout << "Go2 原生相机已连接" << std::endl;
    std::cout << "控制: 终端输入 s 保存图片  q 退出" << std::endl;

    cv::namedWindow("Go2 Camera", cv::WINDOW_AUTOSIZE);

    std::vector<uint8_t> image_sample;
    int save_counter = 0;
    bool save_requested = false;

    while (true) {
        struct pollfd pfd = { STDIN_FILENO, POLLIN, 0 };
        if (poll(&pfd, 1, 0) > 0) {
            std::string line;
            std::getline(std::cin, line);
            if (!line.empty()) {
                char cmd = line[0];
                if (cmd == 'q' || cmd == 'Q') {
                    break;
                } else if (cmd == 's' || cmd == 'S') {
                    save_requested = true;
                }
            }
        }

        int ret = video_client.GetImageSample(image_sample);

        if (ret == 0 && !image_sample.empty()) {
            cv::Mat rawData(image_sample);
            cv::Mat frame = cv::imdecode(rawData, cv::IMREAD_COLOR);

            if (!frame.empty()) {
                cv::imshow("Go2 Camera", frame);
                cv::waitKey(1);

                if (save_requested) {
                    int next_num = getNextPhotoNumber();
                    std::ostringstream oss;
                    oss << "./photo/IMG_" << std::setw(3) << std::setfill('0')
                        << next_num << ".jpg";
                    std::string filename = oss.str();

                    if (cv::imwrite(filename, frame)) {
                        ++save_counter;
                        std::cout << ">>> 已保存: " << filename << std::endl;
                    } else {
                        std::cerr << "!!! 保存失败: " << filename << std::endl;
                    }
                    save_requested = false;
                }
            }
        } else {
            cv::waitKey(1);
        }
    }

    cv::destroyAllWindows();
    std::cout << "程序结束，本次共保存 " << save_counter << " 张图片。" << std::endl;
    return 0;
}
