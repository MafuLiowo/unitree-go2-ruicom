/**
 * @file Go2Action.hpp
 * @brief Go2 视觉动作联动模块，通过 YOLO 识别结果触发预设运动或灯光动作
 *
 * @par 模块说明
 *       本模块集成 Go2 原生摄像头、YOLO 检测器、SportSwitch 运动控制及
 *       LightController 灯光控制，实现"识别即执行"的视觉-动作映射。
 *       检测到对应类别后自动执行预设动作，动作间有冷却间隔避免重复触发。
 *       - "stretch" → 伸懒腰动作
 *       - "hello" → 打招呼动作
 *       - "light" → 前灯闪烁 3 次
 */
#pragma once

#include <unitree/robot/go2/video/video_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <atomic>
#include <thread>
#include "YOLODetector.hpp"
#include "Go2SportSwitch.hpp"
#include "Go2LightController.hpp"

/**
 * @brief Go2 视觉-动作联动类，封装摄像头采集、YOLO推理与动作分发
 *
 * 典型用法:
 * @code
 *   Go2Action::InitChannel("eth0");
 *   std::vector<std::string> class_names = {"stretch", "hello", "light", "one", "two"};
 *   Go2Action action("data/best.onnx", class_names);
 *   if (action.Initialize()) {
 *       action.Run();
 *   }
 * @endcode
 */
class Go2Action
{
public:
    /**
     * @brief 初始化 DDS 通信通道（静态方法，必须在构造 Go2Action 实例前调用）
     * @param netInterface 网络接口名称，如 "eth0"
     */
    static void InitChannel(const std::string& netInterface);

    /**
     * @brief 构造函数
     * @param modelPath ONNX 模型文件路径
     * @param classNames YOLO 类别名称列表，索引需与模型输出一致
     * @param confidenceThreshold 置信度阈值 (0~1)，默认 0.5
     * @param inputSize 模型输入图像尺寸，默认 640x640
     */
    Go2Action(const std::string& modelPath,
              const std::vector<std::string>& classNames,
              float confidenceThreshold = 0.5f,
              const cv::Size& inputSize = cv::Size(640, 640));

    /**
     * @brief 析构函数，释放所有子模块资源
     */
    ~Go2Action();

    /**
     * @brief 初始化各子模块（YOLO检测器、摄像头、运动控制、灯光控制）
     * @return true 初始化成功
     */
    bool Initialize();

    /**
     * @brief 主循环入口：摄像头实时采集 → YOLO检测 → 动作分发 → 画面显示
     */
    void Run();

private:
    /**
     * @brief 从 Go2 原生摄像头获取一帧图像
     * @return cv::Mat 解码后的 BGR 图像，获取失败返回空 Mat
     */
    cv::Mat GetGo2Frame();

    /**
     * @brief 根据检测结果分发执行对应动作（含冷却检查）
     * @param detections 当前帧的检测结果列表
     */
    void DispatchActions(const std::vector<Detection>& detections);

    /**
     * @brief 执行伸懒腰动作（异步线程）
     */
    void ExecuteStretch();

    /**
     * @brief 执行打招呼动作（异步线程）
     */
    void ExecuteHello();

    /**
     * @brief 执行灯光闪烁 3 次（异步线程）
     */
    void ExecuteLightBlink();

    /**
     * @brief 检查指定类别是否处于冷却中
     * @param className 类别名称
     * @return true 表示冷却中，应跳过执行
     */
    bool IsInCooldown(const std::string& className);

    /**
     * @brief 更新冷却计时器
     * @param className 类别名称
     */
    void UpdateCooldown(const std::string& className);

    /**
     * @brief 等待当前异步动作完成
     */
    void WaitForActionDone();

    std::string modelPath_;                ///< ONNX 模型文件路径
    std::vector<std::string> classNames_;  ///< YOLO 类别名称列表
    float confidenceThreshold_;            ///< 置信度阈值
    cv::Size inputSize_;                   ///< 模型输入尺寸

    YOLODetector* detector_;                                ///< YOLO 检测器实例
    unitree::robot::go2::VideoClient* videoClient_;         ///< Go2 原生摄像头客户端
    Go2SportSwitch* sportSwitch_;                           ///< 运动控制实例
    Go2LightController* lightController_;                   ///< 灯光控制实例

    bool initialized_;                      ///< 初始化标志
    int frameCount_;                        ///< 帧计数器
    std::atomic<bool> actionRunning_;       ///< 当前是否有动作正在执行

    /// 各类别最近一次触发时间，用于冷却控制
    std::map<std::string, std::chrono::steady_clock::time_point> actionCooldown_;
    static constexpr float kActionCooldownSec_ = 3.0f;     ///< 动作冷却间隔（秒）

    std::thread actionThread_;              ///< 异步动作执行线程
};
