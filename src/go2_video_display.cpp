/**
 * @file go2_video_display.cpp
 * @brief 基于 Intel RealSense 深度相机的实时画面显示，展示彩色图与深度图
 *
 * @par 使用说明
 *       go2_video_display
 *       示例: ./go2_video_display
 *       控制: [s] 保存当前帧  [q/Esc] 退出程序
 *       说明: 程序自动查找并连接 RealSense 设备，同时显示彩色画面和深度热力图
 */
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <algorithm>

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

int main()
{
    int width = 640;
    int height = 480;
    int fps = 30;

    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);

    rs2::pipeline_profile profile;
    try {
        profile = pipe.start(cfg);
        std::cout << "RealSense 设备已连接" << std::endl;
    } catch (const rs2::error& e) {
        std::cerr << "无法启动 RealSense 设备: " << e.what() << std::endl;
        return -1;
    }

    auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
    float depth_scale = depth_sensor.get_depth_scale();
    std::cout << "深度比例因子: " << depth_scale << std::endl;

    rs2::align align_to_color(RS2_STREAM_COLOR);

    rs2::colorizer color_map;
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.0f);

    const std::string win_color = "RealSense Color";
    const std::string win_depth = "RealSense Depth (Heatmap)";
    const std::string win_combined = "RealSense Combined";

    cv::namedWindow(win_color, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_depth, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_combined, cv::WINDOW_AUTOSIZE);

    std::cout << "控制说明: [s] 保存当前帧 | [q/Esc] 退出程序" << std::endl;

    int save_counter = 0;

    while (true)
    {
        rs2::frameset frames;
        try {
            frames = pipe.wait_for_frames();
        } catch (const rs2::error& e) {
            std::cerr << "获取帧失败: " << e.what() << std::endl;
            break;
        }

        frames = align_to_color.process(frames);

        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();

        if (!color_frame || !depth_frame) {
            continue;
        }

        cv::Mat color_mat(cv::Size(width, height), CV_8UC3,
                          (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        rs2::frame depth_colored = color_map.process(depth_frame);
        cv::Mat depth_mat(cv::Size(width, height), CV_8UC3,
                          (void*)depth_colored.get_data(), cv::Mat::AUTO_STEP);

        cv::Mat combined;
        cv::hconcat(color_mat, depth_mat, combined);

        cv::imshow(win_color, color_mat);
        cv::imshow(win_depth, depth_mat);
        cv::imshow(win_combined, combined);

        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
        else if (key == 's' || key == 'S') {
            std::string timestamp = getCurrentTimestamp();
            std::string color_filename = "realsense_color_" + timestamp + "_" + std::to_string(save_counter) + ".jpg";
            std::string depth_filename = "realsense_depth_" + timestamp + "_" + std::to_string(save_counter) + ".jpg";

            if (cv::imwrite(color_filename, color_mat)) {
                std::cout << ">>> 已保存彩色图: " << color_filename << std::endl;
            } else {
                std::cerr << "!!! 保存彩色图失败: " << color_filename << std::endl;
            }

            if (cv::imwrite(depth_filename, depth_mat)) {
                std::cout << ">>> 已保存深度图: " << depth_filename << std::endl;
            } else {
                std::cerr << "!!! 保存深度图失败: " << depth_filename << std::endl;
            }

            std::cout << "    (共保存 " << ++save_counter << " 组帧)" << std::endl;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
