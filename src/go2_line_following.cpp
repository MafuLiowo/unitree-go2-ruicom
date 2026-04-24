/**
 * @file go2_line_following.cpp
 * @brief 巡线参数调优工具，支持 RealSense 实时视频流和本地图片两种输入源
 */
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <vector>
#include <algorithm>
#include <filesystem>
#include "LineProcessor.hpp"

namespace fs = std::filesystem;

// Global parameters for trackbars
int g_threshold = 60;
int g_hsv_channel = 2; // V channel
int g_roi_y = 300;
int g_roi_h = 100;
int g_erode_iter = 1;
int g_dilate_iter = 2;

void on_trackbar(int, void*) {}

namespace {
    std::vector<std::string> scanImages(const std::string& dirPath)
    {
        std::vector<std::string> images;
        for (const auto& entry : fs::directory_iterator(dirPath)) {
            std::string ext = entry.path().extension().string();
            if (ext == ".jpg" || ext == ".png" || ext == ".bmp") {
                images.push_back(entry.path().string());
            }
        }
        std::sort(images.begin(), images.end());
        return images;
    }
}

int main(int argc, char** argv)
{
    LineProcessor processor;

    // --- 解析命令行参数 ---
    std::vector<std::string> imageList;
    bool useRealsense = true;

    if (argc >= 2) {
        std::string path = argv[1];
        if (fs::is_directory(path)) {
            imageList = scanImages(path);
            if (imageList.empty()) {
                std::cerr << "目录中未找到图片文件" << std::endl;
                return -1;
            }
            useRealsense = false;
            std::cout << "图片模式: 共 " << imageList.size() << " 张，按 空格/回车 切换，q 退出" << std::endl;
        } else if (fs::exists(path)) {
            std::string ext = fs::path(path).extension().string();
            if (ext == ".jpg" || ext == ".png" || ext == ".bmp") {
                imageList.push_back(path);
                useRealsense = false;
                std::cout << "图片模式: 单张图片，按 q 退出" << std::endl;
            } else {
                std::cerr << "不支持的图片格式: " << ext << std::endl;
                return -1;
            }
        } else {
            std::cerr << "路径不存在: " << path << std::endl;
            return -1;
        }
    }

    // --- RealSense 初始化 ---
    rs2::pipeline pipe;
    rs2::config cfg;
    if (useRealsense) {
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        try {
            pipe.start(cfg);
        } catch (const rs2::error& e) {
            std::cerr << "无法启动 RealSense 设备: " << e.what() << std::endl;
            return -1;
        }
    }

    // --- 创建调参窗口 ---
    std::string win_tuning = "Tuning Controls";
    cv::namedWindow(win_tuning, cv::WINDOW_NORMAL);
    cv::createTrackbar("Threshold", win_tuning, &g_threshold, 255, on_trackbar);
    cv::createTrackbar("HSV Channel", win_tuning, &g_hsv_channel, 2, on_trackbar);
    cv::createTrackbar("ROI Y Offset", win_tuning, &g_roi_y, 470, on_trackbar);
    cv::createTrackbar("ROI Height", win_tuning, &g_roi_h, 480, on_trackbar);
    cv::createTrackbar("Erode Iter", win_tuning, &g_erode_iter, 10, on_trackbar);
    cv::createTrackbar("Dilate Iter", win_tuning, &g_dilate_iter, 10, on_trackbar);

    cv::namedWindow("Line Following View", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Binary Mask (ROI)", cv::WINDOW_AUTOSIZE);

    std::cout << "控制说明: [q/Esc] 退出程序";
    if (!useRealsense) std::cout << "  [Space/Enter] 下一张图片";
    std::cout << std::endl;

    // --- 提取共用处理+绘制逻辑为 lambda ---
    auto processAndShow = [&](const cv::Mat& frame, const std::string& overlay) {
        processor.setThreshold(g_threshold);
        processor.setHSVChannel(g_hsv_channel);
        processor.setMorphIterations(g_erode_iter, g_dilate_iter);

        int roi_y = std::clamp(g_roi_y, 0, frame.rows - 1);
        int roi_h = std::clamp(g_roi_h, 1, frame.rows - roi_y);
        processor.setROI(cv::Rect(0, roi_y, frame.cols, roi_h));

        cv::Mat binary_roi = processor.process(frame);
        float cx = processor.getLineCenter(binary_roi);

        cv::Mat display = frame.clone();
        cv::rectangle(display, cv::Rect(0, roi_y, frame.cols, roi_h), cv::Scalar(0, 255, 255), 2);

        if (cx >= 0) {
            cv::circle(display, cv::Point((int)cx, roi_y + roi_h / 2), 5, cv::Scalar(0, 0, 255), -1);
            cv::line(display, cv::Point((int)cx, roi_y), cv::Point((int)cx, roi_y + roi_h), cv::Scalar(0, 255, 0), 2);
            float error = cx - (frame.cols / 2.0f);
            std::string text = "Offset: " + std::to_string((int)error);
            cv::putText(display, text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        } else {
            cv::putText(display, "LINE LOST!", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }

        if (!overlay.empty()) {
            cv::putText(display, overlay, cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        }

        cv::imshow("Line Following View", display);
        if (!binary_roi.empty()) {
            cv::imshow("Binary Mask (ROI)", binary_roi);
        }
    };

    // --- 主循环 ---
    if (useRealsense) {
        while (true)
        {
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::frame color_frame = frames.get_color_frame();
            if (!color_frame) continue;

            cv::Mat frame(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            processAndShow(frame, "");

            char key = (char)cv::waitKey(1);
            if (key == 'q' || key == 27) break;
        }
    } else {
        for (size_t i = 0; i < imageList.size(); i++) {
            cv::Mat frame = cv::imread(imageList[i]);
            if (frame.empty()) {
                std::cerr << "无法读取图片: " << imageList[i] << std::endl;
                continue;
            }

            std::string overlay = fs::path(imageList[i]).filename().string();

            while (true) {
                processAndShow(frame, overlay);

                char key = (char)cv::waitKey(30);
                if (key == ' ' || key == 13) break;   // 下一张
                if (key == 'q' || key == 27) {         // 退出
                    i = imageList.size();
                    break;
                }
                // 其他情况（含调参滑块变化）继续内层循环，重新处理当前图片
            }
        }
    }

    cv::destroyAllWindows();
    return 0;
}
