/**
 * @file go2_walking.cpp
 * @brief Go2 视觉巡线行走程序，基于 RealSense 实时图像二值化进行路径检测与运动控制
 *
 * @par 使用说明
 *       go2_walking [network_interface]
 *       示例: ./go2_walking eth0                      # 指定网络接口
 *             ./go2_walking                           # 使用默认接口（仅预览，不控制运动）
 *       控制: [q/Esc] 退出  [s] 保存当前帧  [Space] 暂停/恢复行走
 *       说明: 图像下半部分用于基准方向检测（连续白色区域中点），
 *             上半部分用于转弯预测（1/4圆弧弯平滑转弯，直角弯按方向转弯，十字路口直行）
 */
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <iostream>
#include <string>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <thread>
#include <chrono>

// ==========================================================================
// 图像尺寸常量
// ==========================================================================
constexpr int IMG_WIDTH  = 640;
constexpr int IMG_HEIGHT = 480;
constexpr int IMG_FPS    = 30;

// ==========================================================================
// 行走控制参数
// ==========================================================================
constexpr float BASE_FORWARD_SPEED    = 0.30f;   ///< 基础前进速度 (m/s)
constexpr float TURN_FORWARD_SPEED    = 0.15f;   ///< 转弯时前进速度 (m/s)
constexpr float MAX_ANGULAR_SPEED     = 1.20f;   ///< 最大角速度 (rad/s)
constexpr float ANGULAR_KP            = 0.025f;  ///< 角速度比例系数 (rad/s per pixel error)
constexpr float RIGHT_ANGLE_SPEED     = 1.50f;   ///< 直角转弯角速度 (rad/s)
constexpr float RIGHT_ANGLE_DURATION  = 1.05f;   ///< 直角转弯持续时间 (s)（~90度）
constexpr int   CROSS_MIN_WIDTH       = 120;     ///< 十字路口检测最小白色区域宽度（像素）

// ==========================================================================
// 转弯类型枚举
// ==========================================================================
enum class TurnType {
    STRAIGHT,           ///< 直行
    LEFT_CURVE,         ///< 1/4 圆弧左转弯（平滑）
    RIGHT_CURVE,        ///< 1/4 圆弧右转弯（平滑）
    LEFT_RIGHT_ANGLE,   ///< 直角左转弯
    RIGHT_RIGHT_ANGLE,  ///< 直角右转弯
    CROSS               ///< 十字路口
};

// ==========================================================================
// 全局可调参数（通过滑块调节）
// ==========================================================================
int g_binaryThreshold  = 128;   ///< V 通道反二值化阈值
int g_erodeIter        = 1;     ///< 腐蚀迭代次数
int g_dilateIter       = 2;     ///< 膨胀迭代次数
int g_morphKernelSize  = 3;     ///< 形态学核大小
int g_splitRow         = 240;   ///< 上下半部分界线行号（0~480）
int g_upperStrips      = 5;     ///< 上半部分水平条带数
int g_curveThreshold   = 50;    ///< 曲线偏移检测阈值（像素）
int g_sharpThreshold   = 100;   ///< 直角转弯偏移阈值（像素）

/**
 * @brief 滑块回调（空函数，仅用于 trackbar 机制）
 * @param val 滑块值（未使用）
 * @param userdata 用户数据（未使用）
 */
static void onTrackbar(int val, void* userdata) { (void)val; (void)userdata; }

// ==========================================================================
// 图像处理函数
// ==========================================================================

/**
 * @brief 对 BGR 图像进行 V 通道反二值化处理
 * @param bgrFrame 输入的 BGR 彩色图像
 * @param threshold 二值化阈值（V 通道值大于此值的置黑，小于的置白）
 * @return cv::Mat 反二值化后的单通道图像
 */
cv::Mat vChannelInverseBinary(const cv::Mat& bgrFrame, int threshold)
{
    cv::Mat hsv;
    cv::cvtColor(bgrFrame, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    cv::Mat vChannel = channels[2];

    int t = std::clamp(threshold, 0, 255);
    cv::Mat result;
    cv::threshold(vChannel, result, t, 255, cv::THRESH_BINARY_INV);
    return result;
}

/**
 * @brief 对二值图像执行形态学操作（先腐蚀去噪，再膨胀填充）
 * @param binary 输入的二值图像
 * @param erodeIter 腐蚀迭代次数
 * @param dilateIter 膨胀迭代次数
 * @param kernelSize 形态学核大小
 * @return cv::Mat 形态学处理后的图像
 */
cv::Mat applyMorphology(const cv::Mat& binary, int erodeIter, int dilateIter, int kernelSize)
{
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    cv::Mat eroded, dilated;
    cv::erode(binary, eroded, kernel, cv::Point(-1, -1), erodeIter);
    cv::dilate(eroded, dilated, kernel, cv::Point(-1, -1), dilateIter);
    return dilated;
}

// ==========================================================================
// 路径分析函数
// ==========================================================================

/**
 * @brief 通过列投影法计算二值图像中白色区域的质心横坐标
 * @param binary 输入的二值图像（ROI 区域）
 * @return float 白色区域质心的 x 坐标，未检测到线条返回 -1.0f
 */
float findColumnProjectionCenter(const cv::Mat& binary)
{
    if (binary.empty()) return -1.0f;

    cv::Mat colSum;
    cv::reduce(binary, colSum, 0, cv::REDUCE_SUM, CV_32F);

    double totalMass = cv::sum(colSum)[0];
    if (totalMass < 10.0) return -1.0f;

    float weightedSum = 0.0f;
    for (int c = 0; c < colSum.cols; ++c) {
        float val = colSum.at<float>(0, c);
        weightedSum += val * static_cast<float>(c);
    }
    return weightedSum / static_cast<float>(totalMass);
}

/**
 * @brief 分析上半部分图像，检测转弯类型
 * @param upperBinary 上半部分的二值图像
 * @param lowerMidX 下半部分检测到的基准中点 x 坐标
 * @param curveThreshold 曲线偏移检测阈值
 * @param sharpThreshold 直角转弯偏移阈值
 * @param numStrips 水平条带数量
 * @return TurnType 检测到的转弯类型
 */
TurnType analyzeUpperHalf(const cv::Mat& upperBinary, float lowerMidX,
                          int curveThreshold, int sharpThreshold, int numStrips)
{
    if (upperBinary.empty() || lowerMidX < 0) return TurnType::STRAIGHT;

    int rows = upperBinary.rows;
    int cols = upperBinary.cols;
    int stripHeight = std::max(1, rows / numStrips);

    // 收集每条带中白色区域的中点 x 坐标
    std::vector<float> stripCenters;
    for (int i = 0; i < numStrips; ++i) {
        int yStart = i * stripHeight;
        int yEnd   = std::min((i + 1) * stripHeight, rows);
        cv::Rect stripRect(0, yStart, cols, yEnd - yStart);
        cv::Mat strip = upperBinary(stripRect);
        float cx = findColumnProjectionCenter(strip);
        if (cx >= 0) {
            stripCenters.push_back(cx);
        }
    }

    if (stripCenters.size() < 2) return TurnType::STRAIGHT;

    // 计算上半部分的平均偏移方向
    float upperAvgX = 0.0f;
    for (float cx : stripCenters) upperAvgX += cx;
    upperAvgX /= static_cast<float>(stripCenters.size());

    float offset = upperAvgX - lowerMidX;

    // 检测十字路口：上半部分白色区域宽度过大（存在多条路径）
    cv::Mat colSum;
    cv::reduce(upperBinary, colSum, 0, cv::REDUCE_SUM, CV_32F);
    int whiteColumns = 0;
    for (int c = 0; c < colSum.cols; ++c) {
        if (colSum.at<float>(0, c) > 1.0f) whiteColumns++;
    }
    if (whiteColumns > CROSS_MIN_WIDTH && std::abs(offset) < sharpThreshold) {
        return TurnType::CROSS;
    }

    // 判断转弯方向和类型
    float absOff = std::abs(offset);

    if (absOff < curveThreshold) {
        return TurnType::STRAIGHT;
    }

    // 分析偏移的渐变程度：比较顶部和底部条带的中点
    float topCenter  = stripCenters.front();
    float bottomCenter = stripCenters.back();
    float gradient   = bottomCenter - topCenter;

    if (absOff > sharpThreshold) {
        // 偏移量大的直角转弯
        return (offset < 0) ? TurnType::LEFT_RIGHT_ANGLE : TurnType::RIGHT_RIGHT_ANGLE;
    } else {
        // 偏移量适中的平滑弯道（1/4 圆弧）
        return (offset < 0) ? TurnType::LEFT_CURVE : TurnType::RIGHT_CURVE;
    }
}

// ==========================================================================
// 运动控制函数
// ==========================================================================

/**
 * @brief 执行行走控制，根据偏移量和转弯类型计算并发送运动指令
 * @param sportClient 运动客户端引用
 * @param errorPx 当前帧的偏移量（像素，正=右侧偏移，负=左侧偏移）
 * @param turnType 检测到的转弯类型
 * @param baseSpeed 基础前进速度
 * @param turnSpeed 转弯时前进速度
 * @param maxAngular 最大角速度
 * @param kp 角速度比例系数
 */
void executeWalking(unitree::robot::go2::SportClient& sportClient,
                    float errorPx, TurnType turnType,
                    float baseSpeed, float turnSpeed,
                    float maxAngular, float kp)
{
    (void)baseSpeed; // 保留参数以支持后续扩展

    static TurnType lastTurnType = TurnType::STRAIGHT;
    static bool executingRightAngle = false;

    switch (turnType) {
    case TurnType::STRAIGHT:
        // 直行：基础速度前进，微小角速度修正偏移
        {
            executingRightAngle = false;
            float vyaw = std::clamp(kp * errorPx, -maxAngular * 0.3f, maxAngular * 0.3f);
            sportClient.Move(baseSpeed, 0.0f, vyaw);
        }
        break;

    case TurnType::LEFT_CURVE:
    case TurnType::RIGHT_CURVE:
        // 1/4 圆弧弯：平滑转弯，前进速度降低，角速度按比例增大
        {
            executingRightAngle = false;
            float vx   = turnSpeed;
            float vyaw = std::clamp(kp * errorPx * 1.5f, -maxAngular, maxAngular);
            sportClient.Move(vx, 0.0f, vyaw);
        }
        break;

    case TurnType::LEFT_RIGHT_ANGLE:
    case TurnType::RIGHT_RIGHT_ANGLE:
        // 直角转弯：先减速，再原地旋转约 90 度
        {
            if (!executingRightAngle) {
                executingRightAngle = true;
                sportClient.StopMove();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));

                float direction = (turnType == TurnType::LEFT_RIGHT_ANGLE) ? 1.0f : -1.0f;
                std::cout << ">>> 直角转弯: "
                          << ((direction > 0) ? "左转" : "右转") << std::endl;

                // 原地旋转
                auto startTime = std::chrono::steady_clock::now();
                float targetDuration = RIGHT_ANGLE_DURATION;
                while (true) {
                    auto elapsed = std::chrono::duration<float>(
                        std::chrono::steady_clock::now() - startTime).count();
                    if (elapsed >= targetDuration) break;
                    sportClient.Move(0.0f, 0.0f, direction * RIGHT_ANGLE_SPEED);
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                sportClient.StopMove();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                executingRightAngle = false;
                std::cout << ">>> 直角转弯完成" << std::endl;
            }
        }
        break;

    case TurnType::CROSS:
        // 十字路口：保持直行，忽略其他方向
        {
            executingRightAngle = false;
            float vx   = baseSpeed;
            float vyaw = std::clamp(kp * errorPx, -maxAngular * 0.2f, maxAngular * 0.2f);
            sportClient.Move(vx, 0.0f, vyaw);
        }
        break;
    }

    lastTurnType = turnType;
}

// ==========================================================================
// 调试绘制函数
// ==========================================================================

/**
 * @brief 在显示图像上绘制路径检测信息
 * @param display 待绘制的显示图像（原地修改）
 * @param binaryFull 完整二值图像
 * @param lowerMidX 下半部分中点 x 坐标
 * @param turnType 检测到的转弯类型
 * @param errorPx 偏移量
 * @param splitRow 上下半部分界线行号
 * @param curveThreshold 曲线阈值
 * @param sharpThreshold 直角阈值
 */
void drawDebugOverlay(cv::Mat& display, const cv::Mat& binaryFull,
                      float lowerMidX, TurnType turnType, float errorPx,
                      int splitRow, int curveThreshold, int sharpThreshold)
{
    (void)curveThreshold;
    (void)sharpThreshold;

    // 绘制上下半部分界线
    cv::line(display, cv::Point(0, splitRow), cv::Point(display.cols - 1, splitRow),
             cv::Scalar(255, 255, 0), 2);

    // 标注区域标签
    cv::putText(display, "Upper: Turn Prediction",
                cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    cv::putText(display, "Lower: Base Direction",
                cv::Point(10, splitRow - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);

    // 绘制下半部分中点及基准线
    if (lowerMidX >= 0) {
        cv::circle(display,
                   cv::Point(static_cast<int>(lowerMidX), splitRow + (display.rows - splitRow) / 2),
                   8, cv::Scalar(0, 0, 255), -1);
        cv::line(display,
                 cv::Point(static_cast<int>(lowerMidX), splitRow),
                 cv::Point(static_cast<int>(lowerMidX), display.rows - 1),
                 cv::Scalar(0, 255, 0), 2);
        cv::line(display,
                 cv::Point(display.cols / 2, splitRow),
                 cv::Point(static_cast<int>(lowerMidX), splitRow + (display.rows - splitRow) / 2),
                 cv::Scalar(0, 165, 255), 1);
    }

    // 在二值图像中叠加半透明红色标记上半部分白色区域
    cv::Mat upperBinary = binaryFull(cv::Rect(0, 0, display.cols, splitRow));
    cv::Mat upperColor;
    cv::cvtColor(upperBinary, upperColor, cv::COLOR_GRAY2BGR);
    upperColor.setTo(cv::Scalar(0, 0, 255), upperBinary);
    cv::addWeighted(display(cv::Rect(0, 0, display.cols, splitRow)), 0.6,
                    upperColor, 0.4, 0.0,
                    display(cv::Rect(0, 0, display.cols, splitRow)));

    // 显示转弯类型文字
    std::string turnText;
    cv::Scalar turnColor;
    switch (turnType) {
    case TurnType::STRAIGHT:
        turnText = "STRAIGHT"; turnColor = cv::Scalar(0, 255, 0); break;
    case TurnType::LEFT_CURVE:
        turnText = "1/4 LEFT CURVE"; turnColor = cv::Scalar(0, 255, 255); break;
    case TurnType::RIGHT_CURVE:
        turnText = "1/4 RIGHT CURVE"; turnColor = cv::Scalar(255, 255, 0); break;
    case TurnType::LEFT_RIGHT_ANGLE:
        turnText = "LEFT 90 DEG!"; turnColor = cv::Scalar(0, 0, 255); break;
    case TurnType::RIGHT_RIGHT_ANGLE:
        turnText = "RIGHT 90 DEG!"; turnColor = cv::Scalar(255, 0, 0); break;
    case TurnType::CROSS:
        turnText = "CROSS -> GO STRAIGHT"; turnColor = cv::Scalar(255, 0, 255); break;
    }
    cv::putText(display, turnText, cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, turnColor, 2);

    // 显示偏移量
    if (lowerMidX >= 0) {
        char buf[64];
        snprintf(buf, sizeof(buf), "Error: %+.0f px", errorPx);
        cv::putText(display, buf, cv::Point(10, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    } else {
        cv::putText(display, "LINE LOST!", cv::Point(10, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }
}

/**
 * @brief 获取当前时间戳字符串
 * @return std::string 格式为 "YYYYMMDD_HHMMSS" 的时间戳
 */
std::string getCurrentTimestamp()
{
    auto now = std::time(nullptr);
    auto tm  = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

// ==========================================================================
// 主函数
// ==========================================================================

/**
 * @brief Go2 视觉巡线行走程序入口
 * @param argc 参数个数
 * @param argv 参数列表，argv[1] 为网络接口名称
 * @return int 0 正常退出，-1 异常退出
 */
int main(int argc, char** argv)
{
    // ---- 解析命令行参数 ----
    std::string netInterface;
    if (argc > 1) {
        netInterface = argv[1];
    }

    bool robotConnected = !netInterface.empty();

    // ---- 初始化 RealSense 相机 ----
    std::cout << "正在初始化 RealSense 相机..." << std::endl;
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_BGR8, IMG_FPS);

    try {
        pipe.start(cfg);
        std::cout << "RealSense 设备已连接" << std::endl;
    } catch (const rs2::error& e) {
        std::cerr << "无法启动 RealSense 设备: " << e.what() << std::endl;
        return -1;
    }

    // ---- 初始化 Unitree SDK 及运动客户端 ----
    unitree::robot::go2::SportClient sportClient;
    if (robotConnected) {
        std::cout << "正在初始化 Unitree SDK (接口: " << netInterface << ")..." << std::endl;
        unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);
        sportClient.SetTimeout(10.0f);
        sportClient.Init();
        std::cout << "运动客户端已就绪" << std::endl;
    } else {
        std::cout << "未指定网络接口，仅预览模式（不控制机器人运动）" << std::endl;
    }

    // ---- 创建显示窗口及滑块 ----
    const std::string winMain    = "Go2 Walking - Line Following";
    const std::string winBinary  = "Binary Image";
    const std::string winTuning  = "Parameter Tuning";

    cv::namedWindow(winMain,   cv::WINDOW_NORMAL);
    cv::resizeWindow(winMain, 960, 720);
    cv::namedWindow(winBinary, cv::WINDOW_NORMAL);
    cv::resizeWindow(winBinary, 640, 480);
    cv::namedWindow(winTuning, cv::WINDOW_NORMAL);
    cv::resizeWindow(winTuning, 400, 300);

    cv::createTrackbar("V Inv Thresh",     winTuning, &g_binaryThreshold, 255, onTrackbar);
    cv::createTrackbar("Erode Iter",       winTuning, &g_erodeIter,       10,  onTrackbar);
    cv::createTrackbar("Dilate Iter",      winTuning, &g_dilateIter,      10,  onTrackbar);
    cv::createTrackbar("Morph Kernel Size", winTuning, &g_morphKernelSize, 15,  onTrackbar);
    cv::createTrackbar("Split Row",        winTuning, &g_splitRow,         IMG_HEIGHT, onTrackbar);
    cv::createTrackbar("Upper Strips",     winTuning, &g_upperStrips,     15,  onTrackbar);
    cv::createTrackbar("Curve Thresh",     winTuning, &g_curveThreshold,  200, onTrackbar);
    cv::createTrackbar("Sharp Thresh",     winTuning, &g_sharpThreshold,  300, onTrackbar);

    std::cout << "\n控制说明: [q/Esc] 退出 | [s] 保存当前帧 | [Space] 暂停/恢复行走" << std::endl;
    std::cout << "滑块说明: 在 Parameter Tuning 窗口中调节图像处理参数" << std::endl;
    std::cout << "========================================" << std::endl;

    // ---- 状态变量 ----
    bool walkingActive = true;
    int  saveCounter   = 0;
    int  frameCount    = 0;
    bool keySpacePressed = false;

    // ---- 主循环 ----
    while (true)
    {
        // 1. 获取 RealSense 帧
        rs2::frameset frames;
        try {
            frames = pipe.wait_for_frames();
        } catch (const rs2::error& e) {
            std::cerr << "获取帧失败: " << e.what() << std::endl;
            break;
        }

        rs2::frame colorFrame = frames.get_color_frame();
        if (!colorFrame) continue;

        cv::Mat bgrFrame(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3,
                         const_cast<void*>(colorFrame.get_data()), cv::Mat::AUTO_STEP);
        ++frameCount;

        // 2. 图像处理：V 通道反二值化 + 形态学
        int kernelSize = std::max(1, g_morphKernelSize);
        cv::Mat binary = vChannelInverseBinary(bgrFrame, g_binaryThreshold);
        cv::Mat binaryMorphed = applyMorphology(binary, g_erodeIter, g_dilateIter, kernelSize);

        // 3. 分割上下半部分
        int splitRow = std::clamp(g_splitRow, 10, IMG_HEIGHT - 10);
        cv::Rect lowerROI(0, splitRow, IMG_WIDTH, IMG_HEIGHT - splitRow);
        cv::Rect upperROI(0, 0,        IMG_WIDTH, splitRow);
        cv::Mat lowerBinary = binaryMorphed(lowerROI);
        cv::Mat upperBinary = binaryMorphed(upperROI);

        // 4. 分析路径
        float lowerMidX = findColumnProjectionCenter(lowerBinary);
        float errorPx   = 0.0f;
        TurnType turnType = TurnType::STRAIGHT;

        if (lowerMidX >= 0) {
            errorPx  = lowerMidX - (IMG_WIDTH / 2.0f);
            int numStrips = std::max(1, g_upperStrips);
            turnType = analyzeUpperHalf(upperBinary, lowerMidX,
                                        g_curveThreshold, g_sharpThreshold, numStrips);
        }

        // 5. 运动控制
        if (robotConnected && walkingActive && lowerMidX >= 0) {
            executeWalking(sportClient, errorPx, turnType,
                          BASE_FORWARD_SPEED, TURN_FORWARD_SPEED,
                          MAX_ANGULAR_SPEED, ANGULAR_KP);
        }

        // 6. 调试可视化
        cv::Mat display = bgrFrame.clone();
        drawDebugOverlay(display, binaryMorphed, lowerMidX, turnType, errorPx,
                        splitRow, g_curveThreshold, g_sharpThreshold);

        // 行走状态指示
        std::string statusText = "Walking: " + std::string(walkingActive ? "ON" : "PAUSED");
        if (!robotConnected) statusText = "Preview Only";
        cv::putText(display, statusText,
                    cv::Point(display.cols - 250, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    walkingActive ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);

        // 帧计数
        cv::putText(display, "Frame: " + std::to_string(frameCount),
                    cv::Point(display.cols - 250, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);

        // 7. 显示
        cv::imshow(winMain, display);
        cv::imshow(winBinary, binaryMorphed);

        // 8. 键盘控制
        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        } else if (key == 's' || key == 'S') {
            std::string ts = getCurrentTimestamp();
            std::string filename = "walking_" + ts + "_" + std::to_string(saveCounter) + ".jpg";
            if (cv::imwrite(filename, display)) {
                std::cout << ">>> 已保存: " << filename << " (第 " << ++saveCounter << " 张)" << std::endl;
            }
        } else if (key == ' ') {
            if (!keySpacePressed) {
                keySpacePressed = true;
                walkingActive = !walkingActive;
                std::cout << ">>> 行走状态: " << (walkingActive ? "恢复" : "暂停") << std::endl;
                if (!walkingActive && robotConnected) {
                    sportClient.StopMove();
                }
            }
        } else {
            keySpacePressed = false;
        }
    }

    // ---- 清理 ----
    if (robotConnected) {
        sportClient.StopMove();
    }
    cv::destroyAllWindows();
    std::cout << "\n程序结束。共处理 " << frameCount << " 帧。" << std::endl;
    return 0;
}
