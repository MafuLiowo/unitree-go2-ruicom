/**
 * @file go2_yolo_identify.cpp
 * @brief YOLO 目标检测识别程序，基于 ONNX 模型在 CPU 上进行低算力实时推理，
 *        使用 Go2 机器狗原生摄像头作为实时视频源
 *
 * @par 使用说明
 *       go2_yolo_identify [network_interface | image_path]
 *       示例: ./go2_yolo_identify                       # Go2 原生摄像头实时检测 (默认网口)
 *             ./go2_yolo_identify eth0                  # 指定网络接口的 Go2 原生摄像头实时检测
 *             ./go2_yolo_identify test.jpg              # 单张图片检测模式
 *             ./go2_yolo_identify ./images/             # 图片目录批量检测模式
 *       控制: [s] 保存当前帧  [Space/Enter] 下一张(图片模式)  [q/Esc] 退出
 *       模型: 默认加载 data/best.onnx (ONNX CPU 推理 — 低算力 ARM 平台最优)
 *
 * @note 在 ARM 低算力平台上，ONNX + OpenCV DNN CPU 推理比 PyTorch .pt 加载更快、
 *       内存占用更低。本程序强制使用 CPU 后端以确保 ARM 平台兼容性。
 *       实时视频源使用 Go2 原生摄像头 (VideoClient)，而非 RealSense 外接相机。
 */
#include <unitree/robot/go2/video/video_client.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include "YOLODetector.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <filesystem>

namespace fs = std::filesystem;

// ==========================================================================
// 检测参数常量 — 可在此调整
// ==========================================================================
constexpr float CONFIDENCE_THRESHOLD = 0.5f;   ///< 置信度阈值，低于此值的结果被丢弃
constexpr float NMS_THRESHOLD      = 0.5f;     ///< 非极大值抑制 IoU 阈值
constexpr int   INPUT_SIZE         = 640;      ///< YOLO 模型输入分辨率

/**
 * @brief 获取当前时间戳字符串，用于文件命名
 * @return std::string 格式为 "YYYYMMDD_HHMMSS" 的时间戳
 */
static std::string getTimestamp() {
    auto now = std::time(nullptr);
    auto tm  = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

/**
 * @brief 扫描目录中所有图片文件
 * @param dirPath 目录路径
 * @return std::vector<std::string> 图片文件路径列表（按名称排序）
 */
static std::vector<std::string> scanImages(const std::string& dirPath) {
    std::vector<std::string> images;
    for (const auto& entry : fs::directory_iterator(dirPath)) {
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") {
            images.push_back(entry.path().string());
        }
    }
    std::sort(images.begin(), images.end());
    return images;
}

/**
 * @brief 打印单帧检测结果到标准输出
 * @param detections 检测结果列表
 * @param frame_idx  当前帧序号
 */
static void printDetections(const std::vector<Detection>& detections, int frame_idx) {
    if (detections.empty()) {
        std::cout << "[Frame " << frame_idx << "] 未检测到目标" << std::endl;
        return;
    }

    std::cout << "[Frame " << frame_idx << "] 检测到 " << detections.size()
              << " 个目标: ";
    for (size_t i = 0; i < detections.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << detections[i].class_name
                  << "(" << std::fixed << std::setprecision(2)
                  << detections[i].confidence << ")";
    }
    std::cout << std::endl;
}

/**
 * @brief 处理并显示单帧图像
 * @param frame      输入 BGR 图像
 * @param detector   YOLO 检测器
 * @param frame_idx  帧序号 (用于打印)
 */
static void processFrame(cv::Mat& frame, YOLODetector& detector, int frame_idx) {
    auto detections = detector.detect(frame, CONFIDENCE_THRESHOLD, NMS_THRESHOLD);

    // 打印检测结果
    printDetections(detections, frame_idx);

    // 在图像上绘制检测框
    YOLODetector::drawDetections(frame, detections, true);

    // 叠加帧计数信息
    std::string info = "Frame: " + std::to_string(frame_idx)
                     + "  Found: " + std::to_string(detections.size());
    cv::putText(frame, info, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

    cv::imshow("YOLO Detection", frame);
}

/**
 * @brief 从 Go2 原生摄像头获取一帧图像
 * @param video_client Go2 视频客户端指针
 * @return cv::Mat 解码后的 BGR 图像，获取失败返回空 Mat
 */
static cv::Mat getGo2Frame(unitree::robot::go2::VideoClient* video_client) {
    std::vector<uint8_t> image_sample;
    int ret = video_client->GetImageSample(image_sample);
    if (ret == 0 && !image_sample.empty()) {
        return cv::imdecode(cv::Mat(image_sample), cv::IMREAD_COLOR);
    }
    return cv::Mat();
}

int main(int argc, char** argv) {
    // ======================================================================
    // 1. 解析命令行参数
    // ======================================================================
    std::vector<std::string> imageList;
    bool useGo2Camera = true;
    std::string netInterface;

    if (argc >= 2) {
        std::string arg = argv[1];
        // 判断参数是图片路径还是网络接口名: 若路径存在则为图片模式, 否则视为网络接口
        if (fs::is_directory(arg)) {
            imageList = scanImages(arg);
            if (imageList.empty()) {
                std::cerr << "错误: 目录中未找到图片文件" << std::endl;
                return -1;
            }
            useGo2Camera = false;
        } else if (fs::exists(arg)) {
            imageList.push_back(arg);
            useGo2Camera = false;
        } else {
            netInterface = arg;
        }
    }

    // ======================================================================
    // 2. 初始化 YOLO 检测器 — 强制使用 CPU 后端 (ARM 平台最优)
    // ======================================================================
    std::vector<std::string> class_names = {
        "stretch",
        "hello",
        "light",
        "one",
        "two"
    };

    std::cout << "========================================" << std::endl;
    std::cout << "Go2 YOLO 目标检测识别程序" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "模型文件: data/best.onnx" << std::endl;
    std::cout << "推理后端: CPU (OpenCV DNN)" << std::endl;
    std::cout << "输入尺寸: " << INPUT_SIZE << "x" << INPUT_SIZE << std::endl;
    std::cout << "置信度阈值: " << CONFIDENCE_THRESHOLD << std::endl;
    std::cout << "NMS 阈值: " << NMS_THRESHOLD << std::endl;
    std::cout << "类别列表: ";
    for (size_t i = 0; i < class_names.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << class_names[i];
    }
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;

    // 默认加载 ONNX 模型 (data/best.onnx)
    std::string model_path = "/home/mafu/ai-unitree-go2-ruicom/data/best.onnx";
    YOLODetector detector(model_path, class_names, cv::Size(INPUT_SIZE, INPUT_SIZE));

    if (!detector.initialize(false)) {  // false = 强制 CPU 后端
        std::cerr << "错误: YOLO 检测器初始化失败" << std::endl;
        return -1;
    }
    std::cout << "YOLO 检测器初始化完成 (CPU 后端)" << std::endl;

    // ======================================================================
    // 3. 初始化 Go2 原生摄像头 (实时模式) — 仅在摄像头模式下构造 VideoClient
    // ======================================================================
    unitree::robot::go2::VideoClient* video_client = nullptr;

    if (useGo2Camera) {
        // 初始化 DDS 信道工厂 — 根据是否指定网络接口选择初始化方式
        if (!netInterface.empty()) {
            unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);
        } else {
            unitree::robot::ChannelFactory::Instance()->Init(0);
        }

        video_client = new unitree::robot::go2::VideoClient();
        video_client->SetTimeout(1.0f);
        video_client->Init();
        std::cout << "Go2 原生摄像头已连接";
        if (!netInterface.empty()) {
            std::cout << " (网络接口: " << netInterface << ")";
        }
        std::cout << "，开始实时检测..." << std::endl;
    } else {
        std::cout << "图片模式: 共 " << imageList.size() << " 张图片" << std::endl;
    }

    // ======================================================================
    // 4. 主循环 — 实时检测 / 图片检测
    // ======================================================================
    std::cout << "\n控制说明: [q/Esc] 退出";
    if (!useGo2Camera) std::cout << "  [Space/Enter] 下一张";
    std::cout << "  [s] 保存当前帧" << std::endl;

    cv::namedWindow("YOLO Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("YOLO Detection", 960, 720);

    int frame_count  = 0;
    int save_counter = 0;
    size_t img_index = 0;
    bool running     = true;

    while (running) {
        cv::Mat frame;

        // --- 获取图像帧 ---
        if (useGo2Camera) {
            frame = getGo2Frame(video_client);
            if (frame.empty()) {
                // Go2 摄像头未返回有效帧时短暂等待, 保持循环响应性
                if ((char)cv::waitKey(1) == 'q') break;
                continue;
            }
        } else {
            if (img_index >= imageList.size()) break;
            frame = cv::imread(imageList[img_index], cv::IMREAD_COLOR);
            if (frame.empty()) {
                std::cerr << "警告: 无法读取图片 " << imageList[img_index] << std::endl;
                ++img_index;
                continue;
            }
            std::cout << "\n--- 处理图片: " << imageList[img_index] << " ---" << std::endl;
        }

        if (frame.empty()) continue;

        // --- 执行检测 ---
        ++frame_count;
        processFrame(frame, detector, frame_count);

        // --- 键盘控制 ---
        char key = 0;
        if (useGo2Camera) {
            // 实时模式: 短暂等待，不阻塞帧率
            key = (char)cv::waitKey(1);
        } else {
            // 图片模式: 等待用户按键后才切换下一张，避免重复推理
            key = (char)cv::waitKey(0);
        }

        if (key == 'q' || key == 27) {
            running = false;
        } else if (key == 's' || key == 'S') {
            // 保存当前帧
            std::string filename = "yolo_detect_" + getTimestamp()
                                 + "_" + std::to_string(save_counter++) + ".jpg";
            if (cv::imwrite(filename, frame)) {
                std::cout << ">>> 已保存图像: " << filename << std::endl;
            } else {
                std::cerr << "!!! 保存失败: " << filename << std::endl;
            }
            // 图片模式下保存后继续显示，不自动切换
            if (!useGo2Camera) continue;
        } else if (!useGo2Camera && (key == ' ' || key == 13)) {
            ++img_index;
            if (img_index >= imageList.size()) {
                std::cout << "\n所有图片处理完毕。" << std::endl;
                running = false;
            }
        }
    }

    // ======================================================================
    // 5. 清理资源
    // ======================================================================
    cv::destroyAllWindows();
    if (video_client != nullptr) {
        delete video_client;
        video_client = nullptr;
    }
    std::cout << "\n程序结束。共处理 " << frame_count << " 帧。" << std::endl;

    return 0;
}
