/**
 * @file go2_process_image.cpp
 * @brief 基于 Intel RealSense 深度相机的图像处理程序，实现 HSV 颜色空间转换、红蓝颜色识别及 V 通道反二值化
 *
 * @par 使用说明
 *       go2_process_image
 *       示例: ./go2_process_image
 *       控制: [q/Esc] 退出程序  [s] 保存当前帧
 *       说明: 程序自动查找并连接 RealSense 设备，实时显示 HSV 分离通道、红蓝掩码及 V 通道反二值化结果
 */
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <ctime>
#include <iomanip>
#include <sstream>

// ============================================================
// 颜色识别阈值 —— 通过窗口滑块实时调节
// 修改以下滑块范围/默认值时，同步修改 createTrackbar 的 max 参数
// ============================================================

/** @brief 红色色调下限（HSV H 通道最小端，范围 0~180） */
int redLowH1  = 0;
/** @brief 红色色调上限（HSV H 通道最小端，范围 0~180） */
int redHighH1 = 10;
/** @brief 红色色调下限（HSV H 通道最大端，用于包裹红色区域，范围 0~180） */
int redLowH2  = 170;
/** @brief 红色色调上限（HSV H 通道最大端，范围 0~180） */
int redHighH2 = 180;

/** @brief 红色饱和度下限（HSV S 通道，范围 0~255） */
int redLowS  = 100;
/** @brief 红色饱和度上限（HSV S 通道，范围 0~255） */
int redHighS = 255;
/** @brief 红色明度下限（HSV V 通道，范围 0~255） */
int redLowV  = 100;
/** @brief 红色明度上限（HSV V 通道，范围 0~255） */
int redHighV = 255;

/** @brief 蓝色色调下限（HSV H 通道，范围 0~180） */
int blueLowH  = 100;
/** @brief 蓝色色调上限（HSV H 通道，范围 0~180） */
int blueHighH = 130;
/** @brief 蓝色饱和度下限（HSV S 通道，范围 0~255） */
int blueLowS  = 100;
/** @brief 蓝色饱和度上限（HSV S 通道，范围 0~255） */
int blueHighS = 255;
/** @brief 蓝色明度下限（HSV V 通道，范围 0~255） */
int blueLowV  = 100;
/** @brief 蓝色明度上限（HSV V 通道，范围 0~255） */
int blueHighV = 255;

/** @brief V 通道反二值化阈值（V 通道灰度值超过此值的变为黑色，低于的变为白色，范围 0~255） */
int vInverseThreshold = 128;

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
 * @brief 对 V 通道进行反二值化处理，黑色区域变白色，亮背景变暗色
 * @param hsvImage 输入的 HSV 图像
 * @param thresholdValue 二值化阈值，高于该值的像素置零（变暗），低于的置为 255（变亮）
 * @return cv::Mat 反二值化后的单通道图像
 */
cv::Mat inverseBinaryV(const cv::Mat& hsvImage, int thresholdValue)
{
    std::vector<cv::Mat> hsvChannels;
    cv::split(hsvImage, hsvChannels);
    cv::Mat vChannel = hsvChannels[2];

    int actualThreshold = std::max(0, std::min(255, thresholdValue));

    cv::Mat result;
    // THRESH_BINARY_INV: 高于阈值的置零（暗色），低于阈值的置 255（白色）——实现反二值化
    cv::threshold(vChannel, result, actualThreshold, 255, cv::THRESH_BINARY_INV);
    return result;
}

/**
 * @brief 通过 HSV 颜色空间检测红色区域
 *
 * @par 检测原理
 *       HSV 色调环中红色位于 0° 和 360° 两端，映射到 OpenCV 的 H 通道（0~180）
 *       分为 [0~10] 和 [170~180] 两段，合并即为完整红色区域。
 *
 * @param hsvImage 输入的 HSV 图像
 * @return cv::Mat 红色区域的二值掩码图像
 */
cv::Mat detectRed(const cv::Mat& hsvImage)
{
    cv::Scalar lower1(redLowH1, redLowS, redLowV);
    cv::Scalar upper1(redHighH1, redHighS, redHighV);
    cv::Scalar lower2(redLowH2, redLowS, redLowV);
    cv::Scalar upper2(redHighH2, redHighS, redHighV);

    cv::Mat mask1, mask2, mask;
    cv::inRange(hsvImage, lower1, upper1, mask1);
    cv::inRange(hsvImage, lower2, upper2, mask2);
    cv::bitwise_or(mask1, mask2, mask);
    return mask;
}

/**
 * @brief 通过 HSV 颜色空间检测蓝色区域
 * @param hsvImage 输入的 HSV 图像
 * @return cv::Mat 蓝色区域的二值掩码图像
 */
cv::Mat detectBlue(const cv::Mat& hsvImage)
{
    cv::Scalar lower(blueLowH, blueLowS, blueLowV);
    cv::Scalar upper(blueHighH, blueHighS, blueHighV);
    cv::Mat mask;
    cv::inRange(hsvImage, lower, upper, mask);
    return mask;
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

    rs2::align align_to_color(RS2_STREAM_COLOR);

    // -------------------------------------------------------
    // 显示窗口
    // -------------------------------------------------------
    const std::string winOriginal   = "Original RGB";
    const std::string winVInverse   = "V Channel Inverse Binary";
    const std::string winRedMask    = "Red Mask";
    const std::string winBlueMask   = "Blue Mask";
    const std::string winCombined   = "Combined (Red+Blue Overlay)";

    cv::namedWindow(winOriginal, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(winVInverse, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(winRedMask, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(winBlueMask, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(winCombined, cv::WINDOW_AUTOSIZE);

    // -------------------------------------------------------
    // 颜色识别阈值调节滑块 —— 修改默认值时同步更新上方全局变量的初值
    // -------------------------------------------------------

    // 红色阈值滑块
    cv::createTrackbar("Red H Low1",  winRedMask, &redLowH1,  180);
    cv::createTrackbar("Red H High1", winRedMask, &redHighH1, 180);
    cv::createTrackbar("Red H Low2",  winRedMask, &redLowH2,  180);
    cv::createTrackbar("Red H High2", winRedMask, &redHighH2, 180);
    cv::createTrackbar("Red S Low",   winRedMask, &redLowS,   255);
    cv::createTrackbar("Red S High",  winRedMask, &redHighS,  255);
    cv::createTrackbar("Red V Low",   winRedMask, &redLowV,   255);
    cv::createTrackbar("Red V High",  winRedMask, &redHighV,  255);

    // 蓝色阈值滑块
    cv::createTrackbar("Blue H Low",  winBlueMask, &blueLowH,  180);
    cv::createTrackbar("Blue H High", winBlueMask, &blueHighH, 180);
    cv::createTrackbar("Blue S Low",  winBlueMask, &blueLowS,  255);
    cv::createTrackbar("Blue S High", winBlueMask, &blueHighS, 255);
    cv::createTrackbar("Blue V Low",  winBlueMask, &blueLowV,  255);
    cv::createTrackbar("Blue V High", winBlueMask, &blueHighV, 255);

    // V 通道反二值化阈值滑块
    cv::createTrackbar("V Inv Thresh", winVInverse, &vInverseThreshold, 255);

    std::cout << "控制说明: [q/Esc] 退出程序 | [s] 保存当前帧" << std::endl;
    std::cout << "滑块说明: 在 Red Mask / Blue Mask 窗口中调节 H/S/V 阈值范围" << std::endl;
    std::cout << "         在 V Channel Inverse Binary 窗口中调节反二值化阈值" << std::endl;

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
        if (!color_frame) continue;

        cv::Mat bgrFrame(cv::Size(width, height), CV_8UC3,
                         (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // ---- 1. BGR → HSV 色彩空间转换 ----
        cv::Mat hsvFrame;
        cv::cvtColor(bgrFrame, hsvFrame, cv::COLOR_BGR2HSV);

        // ---- 2. 分离 HSV 通道（用于单独查看各通道灰度图，可选展示） ----
        std::vector<cv::Mat> hsvChannels;
        cv::split(hsvFrame, hsvChannels);
        cv::Mat hChannel = hsvChannels[0];
        cv::Mat sChannel = hsvChannels[1];
        cv::Mat vChannel = hsvChannels[2];

        // ---- 3. V 通道反二值化：黑色→白色，亮背景→暗色 ----
        cv::Mat vInverseBinary = inverseBinaryV(hsvFrame, vInverseThreshold);

        // ---- 4. 红色检测（HSV 颜色空间） ----
        cv::Mat redMask   = detectRed(hsvFrame);

        // ---- 5. 蓝色检测（HSV 颜色空间） ----
        cv::Mat blueMask  = detectBlue(hsvFrame);

        // ---- 6. 组合显示：在原始图上叠加红/蓝色掩码 ----
        cv::Mat combinedOverlay = bgrFrame.clone();

        // 在原始图像上用半透明红色标记检测到的红色区域
        cv::Mat redOverlay;
        cv::cvtColor(redMask, redOverlay, cv::COLOR_GRAY2BGR);
        redOverlay.setTo(cv::Scalar(0, 0, 255), redMask);  // 红色区域标为 BGR=(0,0,255)
        cv::addWeighted(combinedOverlay, 0.7, redOverlay, 0.3, 0.0, combinedOverlay);

        // 在图像上用半透明蓝色标记检测到的蓝色区域
        cv::Mat blueOverlay;
        cv::cvtColor(blueMask, blueOverlay, cv::COLOR_GRAY2BGR);
        blueOverlay.setTo(cv::Scalar(255, 0, 0), blueMask);  // 蓝色区域标为 BGR=(255,0,0)
        cv::addWeighted(combinedOverlay, 0.7, blueOverlay, 0.3, 0.0, combinedOverlay);

        // ---- 7. 在掩码窗口中添加文字标注 ----
        cv::Mat redDisplay;
        cv::cvtColor(redMask, redDisplay, cv::COLOR_GRAY2BGR);
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "Red: H[%d-%d]+[%d-%d] S[%d-%d] V[%d-%d]",
                     redLowH1, redHighH1, redLowH2, redHighH2,
                     redLowS, redHighS, redLowV, redHighV);
            cv::putText(redDisplay, buf, cv::Point(5, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
        }

        cv::Mat blueDisplay;
        cv::cvtColor(blueMask, blueDisplay, cv::COLOR_GRAY2BGR);
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "Blue: H[%d-%d] S[%d-%d] V[%d-%d]",
                     blueLowH, blueHighH, blueLowS, blueHighS, blueLowV, blueHighV);
            cv::putText(blueDisplay, buf, cv::Point(5, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
        }

        // ---- 8. 显示所有处理结果 ----
        cv::imshow(winOriginal, bgrFrame);
        cv::imshow(winVInverse, vInverseBinary);
        cv::imshow(winRedMask, redDisplay);
        cv::imshow(winBlueMask, blueDisplay);
        cv::imshow(winCombined, combinedOverlay);

        // ---- 9. 键盘控制 ----
        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
        else if (key == 's' || key == 'S') {
            std::string timestamp = getCurrentTimestamp();
            std::string oriFile   = "proc_original_"  + timestamp + "_" + std::to_string(save_counter) + ".jpg";
            std::string invFile   = "proc_vinv_"      + timestamp + "_" + std::to_string(save_counter) + ".jpg";
            std::string redFile   = "proc_red_"       + timestamp + "_" + std::to_string(save_counter) + ".jpg";
            std::string blueFile  = "proc_blue_"      + timestamp + "_" + std::to_string(save_counter) + ".jpg";
            std::string combFile  = "proc_combined_"  + timestamp + "_" + std::to_string(save_counter) + ".jpg";

            cv::imwrite(oriFile,  bgrFrame);
            cv::imwrite(invFile,  vInverseBinary);
            cv::imwrite(redFile,  redMask);
            cv::imwrite(blueFile, blueMask);
            cv::imwrite(combFile, combinedOverlay);

            std::cout << ">>> 已保存帧 (组 " << ++save_counter << ")" << std::endl;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
