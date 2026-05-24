/**
 * @file go2_jump.cpp
 * @brief 深度相机横棒检测与前跳程序，通过 RealSense 深度相机持续检测水平横棒，检测到后触发 Go2 前跳
 *
 * @par 使用说明
 *       go2_jump <network_interface>
 *       示例: ./go2_jump eth0
 *       说明: 程序自动连接 RealSense 深度相机，持续分析深度图中是否存在水平横棒（深度值显著异于
 *             背景的横向带状区域），检测到后调用 Go2 FrontJump 前跳一次并退出。
 *       控制: [q/Esc] 手动退出
 */
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "Go2SportSwitch.hpp"

/** @brief 深度图水平分割的条带高度（像素） */
constexpr int STRIP_HEIGHT = 20;

/** @brief 横棒深度与背景深度的最小差异阈值（米） */
constexpr float DEPTH_DIFF_THRESHOLD = 0.3f;

/** @brief 横棒条带的最小宽度占比（有效像素占全宽的比例） */
constexpr float MIN_WIDTH_RATIO = 0.4f;

/** @brief 横棒条带的最大高度（条带数），防止将大面积物体误判为横棒 */
constexpr int MAX_BAR_STRIPS = 4;

/** @brief 检测到横棒后的连续确认帧数，防止误触发 */
constexpr int CONFIRM_FRAMES = 3;

/**
 * @brief 获取当前时间戳字符串
 * @return std::string 格式为 "YYYYMMDD_HHMMSS" 的时间戳
 */
std::string getCurrentTimestamp()
{
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

/**
 * @brief 检测深度图中的水平横棒
 *
 * 将深度图按行等分为若干水平条带，计算每个条带的中位深度值，
 * 再找出与背景深度差异显著且横跨画面大部分宽度的窄条带区域。
 *
 * @param depthMat 16位原始深度图像（CV_16UC1）
 * @param depthScale 深度缩放因子，将原始uint16值转换为米
 * @param barTopRow 输出参数，横棒上边界行号
 * @param barBottomRow 输出参数，横棒下边界行号
 * @return true 检测到水平横棒
 */
bool detectHorizontalBar(const cv::Mat& depthMat, float depthScale,
                         int& barTopRow, int& barBottomRow)
{
    int height = depthMat.rows;
    int width = depthMat.cols;
    int numStrips = height / STRIP_HEIGHT;

    if (numStrips < 4) {
        return false;
    }

    // 计算每个条带的中位深度和有效像素占比
    struct StripInfo {
        float medianDepth = 0.0f;
        float validRatio = 0.0f;   // 有效像素占全宽的比例
        int startRow = 0;
        int endRow = 0;
    };

    std::vector<StripInfo> strips(numStrips);
    std::vector<uint16_t> sampleBuffer;  // 复用缓冲区
    sampleBuffer.reserve(width * STRIP_HEIGHT);

    for (int s = 0; s < numStrips; s++) {
        int r0 = s * STRIP_HEIGHT;
        int r1 = r0 + STRIP_HEIGHT;
        strips[s].startRow = r0;
        strips[s].endRow = r1;

        sampleBuffer.clear();
        for (int r = r0; r < r1; r++) {
            const uint16_t* rowPtr = depthMat.ptr<uint16_t>(r);
            for (int c = 0; c < width; c++) {
                if (rowPtr[c] > 0) {
                    sampleBuffer.push_back(rowPtr[c]);
                }
            }
        }

        strips[s].validRatio = static_cast<float>(sampleBuffer.size())
                               / static_cast<float>(width * STRIP_HEIGHT);

        if (sampleBuffer.size() > width * 0.1f) {
            std::sort(sampleBuffer.begin(), sampleBuffer.end());
            strips[s].medianDepth = static_cast<float>(
                sampleBuffer[sampleBuffer.size() / 2]) * depthScale;
        }
    }

    // 计算背景深度：取有效条带中位深度的中位数（排除极端值）
    std::vector<float> validMedians;
    for (const auto& strip : strips) {
        if (strip.validRatio > 0.3f && strip.medianDepth > 0.1f) {
            validMedians.push_back(strip.medianDepth);
        }
    }

    if (validMedians.size() < 4) {
        return false;
    }

    std::sort(validMedians.begin(), validMedians.end());
    float backgroundDepth = validMedians[validMedians.size() / 2];

    // 扫描条带，寻找与背景深度差异显著的连续窄区域
    int barStart = -1;
    int barEnd = -1;
    bool inBarRegion = false;

    for (int s = 0; s < numStrips; s++) {
        if (strips[s].validRatio < MIN_WIDTH_RATIO) {
            // 有效像素太少，视为无效条带，中断当前候选区域
            if (inBarRegion) {
                barEnd = s - 1;
                break;
            }
            continue;
        }

        bool isAnomaly = std::abs(strips[s].medianDepth - backgroundDepth)
                         > DEPTH_DIFF_THRESHOLD;

        if (isAnomaly && strips[s].medianDepth > 0.1f) {
            if (!inBarRegion) {
                barStart = s;
                inBarRegion = true;
            }
        } else {
            if (inBarRegion) {
                barEnd = s - 1;
                break;
            }
        }
    }

    if (inBarRegion && barEnd < 0) {
        barEnd = numStrips - 1;
    }

    // 检查候选区域是否符合横棒特征
    if (barStart < 0 || barEnd < 0) {
        return false;
    }

    int barStripCount = barEnd - barStart + 1;
    if (barStripCount < 1 || barStripCount > MAX_BAR_STRIPS) {
        return false;
    }

    // 确保候选区域不在画面最顶部或最底部（避免地面/天花板误判）
    if (barStart == 0 || barEnd == numStrips - 1) {
        return false;
    }

    // 确保候选区域上下都有有效的背景条带
    if (barStart > 0 && (strips[barStart - 1].validRatio < MIN_WIDTH_RATIO
        || std::abs(strips[barStart - 1].medianDepth - backgroundDepth) < DEPTH_DIFF_THRESHOLD * 0.5f)) {
        // 上方是背景，合理
    } else if (barStart > 0) {
        return false;
    }

    barTopRow = strips[barStart].startRow;
    barBottomRow = strips[barEnd].endRow;
    return true;
}

int main(int argc, char** argv)
{
    // 解析命令行参数，获取网络接口名称
    std::string netInterface;
    if (argc < 2) {
        std::cout << "用法: " << argv[0] << " <network_interface>" << std::endl;
        std::cout << "示例: " << argv[0] << " eth0" << std::endl;
        return -1;
    }
    netInterface = argv[1];

    // ================================================================
    // 1. 初始化 RealSense 深度相机
    // ================================================================
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
    float depthScale = depth_sensor.get_depth_scale();
    std::cout << "深度比例因子: " << depthScale << std::endl;

    rs2::align align_to_color(RS2_STREAM_COLOR);

    rs2::colorizer color_map;
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.0f);

    // ================================================================
    // 2. 初始化 Unitree SDK2 通信通道与运动控制
    // ================================================================
    if (!netInterface.empty()) {
        unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);
    } else {
        unitree::robot::ChannelFactory::Instance()->Init(0);
    }

    Go2SportSwitch sport;
    std::cout << "Go2 运动控制已就绪 (网络接口: " << netInterface << ")" << std::endl;

    // ================================================================
    // 3. 创建显示窗口
    // ================================================================
    const std::string winColor = "RealSense Color (横棒检测)";
    const std::string winDepth = "RealSense Depth Heatmap";

    cv::namedWindow(winColor, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(winDepth, cv::WINDOW_AUTOSIZE);

    std::cout << "\n========================================" << std::endl;
    std::cout << "横棒检测已启动，等待检测水平横棒..." << std::endl;
    std::cout << "控制: [q/Esc] 手动退出" << std::endl;
    std::cout << "========================================" << std::endl;

    // ================================================================
    // 4. 主循环：深度帧采集 → 横棒检测 → 触发前跳 → 退出
    // ================================================================
    int confirmCount = 0;       // 连续确认计数
    const int maxConfirm = CONFIRM_FRAMES;

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

        rs2::frame colorFrame = frames.get_color_frame();
        rs2::frame depthFrame = frames.get_depth_frame();

        if (!colorFrame || !depthFrame) {
            continue;
        }

        // 构建 OpenCV 矩阵
        cv::Mat colorMat(cv::Size(width, height), CV_8UC3,
                         (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depthRaw(cv::Size(width, height), CV_16UC1,
                         (void*)depthFrame.get_data(), cv::Mat::AUTO_STEP);

        // 深度热力图
        rs2::frame depthColored = color_map.process(depthFrame);
        cv::Mat depthMat(cv::Size(width, height), CV_8UC3,
                         (void*)depthColored.get_data(), cv::Mat::AUTO_STEP);

        // 横棒检测
        int barTop = -1, barBottom = -1;
        bool barDetected = detectHorizontalBar(depthRaw, depthScale,
                                                barTop, barBottom);

        if (barDetected) {
            confirmCount++;
            std::cout << "[检测] 发现候选横棒 (条带行: " << barTop << "~" << barBottom
                      << "), 确认计数: " << confirmCount << "/" << maxConfirm << std::endl;

            // 在彩色图上绘制横棒候选区域
            cv::line(colorMat, cv::Point(0, barTop), cv::Point(width - 1, barTop),
                     cv::Scalar(0, 255, 0), 2);
            cv::line(colorMat, cv::Point(0, barBottom), cv::Point(width - 1, barBottom),
                     cv::Scalar(0, 255, 0), 2);
            cv::rectangle(colorMat, cv::Point(0, barTop),
                          cv::Point(width - 1, barBottom),
                          cv::Scalar(0, 255, 0), 1);
        } else {
            if (confirmCount > 0) {
                std::cout << "[检测] 横棒消失，重置确认计数" << std::endl;
            }
            confirmCount = 0;
        }

        // 显示画面
        cv::imshow(winColor, colorMat);
        cv::imshow(winDepth, depthMat);

        // 确认帧数达标，触发前跳
        if (confirmCount >= maxConfirm) {
            std::cout << "\n========================================" << std::endl;
            std::cout << "!!! 检测到水平横棒，触发前跳 !!!" << std::endl;
            std::cout << "========================================" << std::endl;

            sport.FrontJump();

            std::cout << "前跳完成，程序退出。" << std::endl;
            break;
        }

        // 键盘控制
        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) {
            std::cout << "用户手动退出。" << std::endl;
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
