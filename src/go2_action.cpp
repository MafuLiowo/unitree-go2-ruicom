/**
 * @file go2_action.cpp
 * @brief Go2 视觉动作联动程序，通过 YOLO 识别类别自动触发对应的运动或灯光动作
 *
 * @par 使用说明
 *       go2_action [network_interface]
 *       示例: ./go2_action eth0                  # 指定网络接口
 *             ./go2_action                       # 使用默认接口
 *       控制: [q/Esc] 退出  [s] 保存当前帧
 *       动作映射:
 *         "stretch" → 伸懒腰 (Go2SportSwitch::Stretch)
 *         "hello"             → 打招呼 (Go2SportSwitch::Hello)
 *         "light"             → 前灯闪烁 3 次 (Go2LightController::Blink)
 */
#include "Go2Action.hpp"
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sstream>

// ==========================================================================
// 检测参数常量
// ==========================================================================
constexpr float CONFIDENCE_THRESHOLD = 0.5f;
constexpr float NMS_THRESHOLD      = 0.5f;
constexpr int   INPUT_SIZE         = 640;

// ==========================================================================
// YOLO 类别名称（需与训练模型 class_names 顺序一致）
// ==========================================================================
static std::vector<std::string> kClassNames = {
    "stretch",
    "hello",
    "light",
    "one",
    "two"
};

// ==========================================================================
// Go2Action 静态方法
// ==========================================================================

/**
 * @brief 初始化 DDS 通信通道
 * @param netInterface 网络接口名称，空字符串使用默认配置
 */
void Go2Action::InitChannel(const std::string& netInterface)
{
    if (!netInterface.empty()) {
        unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);
        std::cout << "DDS 信道已初始化 (网络接口: " << netInterface << ")" << std::endl;
    } else {
        unitree::robot::ChannelFactory::Instance()->Init(0);
        std::cout << "DDS 信道已初始化 (默认接口)" << std::endl;
    }
}

// ==========================================================================
// 构造 / 析构 / 初始化
// ==========================================================================

/**
 * @brief 构造函数 — 保存配置参数，不创建子模块实例
 * @param modelPath ONNX 模型文件路径
 * @param classNames YOLO 类别名称列表
 * @param confidenceThreshold 置信度阈值
 * @param inputSize 模型输入尺寸
 */
Go2Action::Go2Action(const std::string& modelPath,
                     const std::vector<std::string>& classNames,
                     float confidenceThreshold,
                     const cv::Size& inputSize)
    : modelPath_(modelPath)
    , classNames_(classNames)
    , confidenceThreshold_(confidenceThreshold)
    , inputSize_(inputSize)
    , detector_(nullptr)
    , videoClient_(nullptr)
    , sportSwitch_(nullptr)
    , lightController_(nullptr)
    , initialized_(false)
    , frameCount_(0)
    , actionRunning_(false)
{
}

/**
 * @brief 析构函数 — 释放子模块资源
 */
Go2Action::~Go2Action()
{
    if (actionThread_.joinable()) {
        actionThread_.join();
    }
    delete videoClient_;
    delete detector_;
    delete sportSwitch_;
    delete lightController_;
}

/**
 * @brief 初始化各子模块
 * @return true 初始化成功
 */
bool Go2Action::Initialize()
{
    // 1. YOLO 检测器 (自动选择 ARM CPU 最优后端)
    detector_ = new YOLODetector(modelPath_, classNames_, inputSize_);
    if (!detector_->initialize()) {
        std::cerr << "错误: YOLO 检测器初始化失败" << std::endl;
        return false;
    }
    std::cout << "YOLO 检测器初始化完成" << std::endl;

    // 2. Go2 原生摄像头
    videoClient_ = new unitree::robot::go2::VideoClient();
    videoClient_->SetTimeout(1.0f);
    videoClient_->Init();
    std::cout << "Go2 原生摄像头已就绪" << std::endl;

    // 3. 运动控制（SportClient 在其构造函数中 Init）
    sportSwitch_ = new Go2SportSwitch();
    std::cout << "运动控制模块已就绪" << std::endl;

    // 4. 灯光控制（VuiClient 在其构造函数中 Init）
    lightController_ = new Go2LightController();
    std::cout << "灯光控制模块已就绪" << std::endl;

    initialized_ = true;
    return true;
}

// ==========================================================================
// 摄像头帧采集
// ==========================================================================

/**
 * @brief 从 Go2 原生摄像头获取一帧图像
 * @return cv::Mat 解码后的 BGR 图像，获取失败返回空 Mat
 */
cv::Mat Go2Action::GetGo2Frame()
{
    std::vector<uint8_t> imageSample;
    int ret = videoClient_->GetImageSample(imageSample);
    if (ret == 0 && !imageSample.empty()) {
        return cv::imdecode(cv::Mat(imageSample), cv::IMREAD_COLOR);
    }
    return cv::Mat();
}

// ==========================================================================
// 冷却管理
// ==========================================================================

/**
 * @brief 检查指定类别是否处于冷却中
 * @param className 类别名称
 * @return true 表示冷却中
 */
bool Go2Action::IsInCooldown(const std::string& className)
{
    auto it = actionCooldown_.find(className);
    if (it == actionCooldown_.end()) {
        return false;
    }
    auto elapsed = std::chrono::steady_clock::now() - it->second;
    return elapsed < std::chrono::duration<float>(kActionCooldownSec_);
}

/**
 * @brief 更新类别冷却计时器
 * @param className 类别名称
 */
void Go2Action::UpdateCooldown(const std::string& className)
{
    actionCooldown_[className] = std::chrono::steady_clock::now();
}

// ==========================================================================
// 动作执行（异步）
// ==========================================================================

/**
 * @brief 等待当前异步动作完成
 */
void Go2Action::WaitForActionDone()
{
    if (actionThread_.joinable()) {
        actionThread_.join();
    }
}

/**
 * @brief 执行伸懒腰动作（在独立线程中运行）
 */
void Go2Action::ExecuteStretch()
{
    actionRunning_ = true;
    WaitForActionDone();
    actionThread_ = std::thread([this]() {
        sportSwitch_->Stretch();
        actionRunning_ = false;
    });
}

/**
 * @brief 执行打招呼动作（在独立线程中运行）
 */
void Go2Action::ExecuteHello()
{
    actionRunning_ = true;
    WaitForActionDone();
    actionThread_ = std::thread([this]() {
        sportSwitch_->Hello();
        actionRunning_ = false;
    });
}

/**
 * @brief 执行灯光闪烁 3 次（在独立线程中运行）
 */
void Go2Action::ExecuteLightBlink()
{
    actionRunning_ = true;
    WaitForActionDone();
    actionThread_ = std::thread([this]() {
        lightController_->Blink(3, 1.0f, 0.7f);
        actionRunning_ = false;
    });
}

// ==========================================================================
// 检测结果分发
// ==========================================================================

/**
 * @brief 将检测结果映射到对应动作并异步执行
 * @param detections 当前帧的检测结果列表
 *
 * 映射规则:
 *   - "stretch" → Stretch (伸懒腰)
 *   - "hello"                → Hello   (打招呼)
 *   - "light"                → 灯光闪烁 3 次
 *
 * 若当前有动作正在执行，或对应类别处于冷却中，则跳过。
 */
void Go2Action::DispatchActions(const std::vector<Detection>& detections)
{
    if (detections.empty()) return;

    for (const auto& det : detections) {
        std::string name = det.class_name;

        // 冷却检查
        if (IsInCooldown(name)) {
            continue;
        }

        // 动作正在执行中则跳过
        if (actionRunning_) {
            std::cout << "[Frame " << frameCount_ << "] 检测到 " << name
                      << "，但当前动作尚未完成，跳过" << std::endl;
            continue;
        }

        // 动作分发
        if (name == "stretch") {
            std::cout << "[Frame " << frameCount_ << "] 检测到 stretch → 执行伸懒腰" << std::endl;
            UpdateCooldown("stretch");
            ExecuteStretch();
        } else if (name == "hello") {
            std::cout << "[Frame " << frameCount_ << "] 检测到 hello → 执行打招呼" << std::endl;
            UpdateCooldown("hello");
            ExecuteHello();
        } else if (name == "light") {
            std::cout << "[Frame " << frameCount_ << "] 检测到 light → 执行灯光闪烁 3 次" << std::endl;
            UpdateCooldown("light");
            ExecuteLightBlink();
        }
    }
}

// ==========================================================================
// 主循环
// ==========================================================================

/**
 * @brief 主循环：摄像头采集 → YOLO检测 → 动作分发 → 画面显示
 */
void Go2Action::Run()
{
    if (!initialized_) {
        std::cerr << "错误: Go2Action 未初始化，请先调用 Initialize()" << std::endl;
        return;
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "Go2 视觉动作联动程序" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "模型文件: " << modelPath_ << std::endl;
    std::cout << "置信度阈值: " << confidenceThreshold_ << std::endl;
    std::cout << "动作冷却: " << kActionCooldownSec_ << " 秒" << std::endl;
    std::cout << "动作映射:" << std::endl;
    std::cout << "  stretch  → 伸懒腰 (Stretch)" << std::endl;
    std::cout << "  hello    → 打招呼 (Hello)" << std::endl;
    std::cout << "  light    → 前灯闪烁 3 次 (Blink)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "\n控制说明: [q/Esc] 退出  [s] 保存当前帧" << std::endl;

    cv::namedWindow("Go2 Action Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("Go2 Action Detection", 960, 720);

    int saveCounter = 0;
    bool running = true;

    while (running) {
        // 1. 获取图像帧
        cv::Mat frame = GetGo2Frame();
        if (frame.empty()) {
            if ((char)cv::waitKey(1) == 'q') break;
            continue;
        }

        // 2. YOLO 检测
        ++frameCount_;
        auto detections = detector_->detect(frame, confidenceThreshold_, NMS_THRESHOLD);

        // 3. 打印检测结果
        if (!detections.empty()) {
            std::cout << "[Frame " << frameCount_ << "] 检测到 " << detections.size()
                      << " 个目标: ";
            for (size_t i = 0; i < detections.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << detections[i].class_name;
            }
            std::cout << std::endl;
        }

        // 4. 动作分发 — 根据检测结果触发对应动作
        DispatchActions(detections);

        // 5. 绘制检测框
        YOLODetector::drawDetections(frame, detections, true);

        // 6. 叠加状态信息
        std::string info = "Frame: " + std::to_string(frameCount_)
                         + "  Found: " + std::to_string(detections.size());
        if (actionRunning_) {
            info += "  [动作执行中]";
        }
        cv::putText(frame, info, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

        // 7. 显示画面
        cv::imshow("Go2 Action Detection", frame);

        // 8. 键盘控制
        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) {
            running = false;
        } else if (key == 's' || key == 'S') {
            auto now = std::time(nullptr);
            auto tm  = *std::localtime(&now);
            std::ostringstream oss;
            oss << "action_detect_" << std::put_time(&tm, "%Y%m%d_%H%M%S")
                << "_" << saveCounter++ << ".jpg";
            std::string filename = oss.str();
            if (cv::imwrite(filename, frame)) {
                std::cout << ">>> 已保存图像: " << filename << std::endl;
            } else {
                std::cerr << "!!! 保存失败: " << filename << std::endl;
            }
        }
    }

    // 清理
    WaitForActionDone();
    cv::destroyAllWindows();
    std::cout << "\n程序结束。共处理 " << frameCount_ << " 帧。" << std::endl;
}

// ==========================================================================
// main 入口
// ==========================================================================

int main(int argc, char** argv)
{
    // 解析网络接口参数
    std::string netInterface;
    if (argc > 1) {
        netInterface = argv[1];
    }

    // 初始化 DDS 通信
    Go2Action::InitChannel(netInterface);

    // 创建视觉动作联动实例
    std::string modelPath = "/home/unitree/ai-unitree-go2-ruicom/data/best.onnx";
    Go2Action action(modelPath, kClassNames, CONFIDENCE_THRESHOLD, cv::Size(INPUT_SIZE, INPUT_SIZE));

    if (!action.Initialize()) {
        std::cerr << "错误: Go2Action 初始化失败" << std::endl;
        return -1;
    }

    action.Run();

    return 0;
}
