/**
 * @file check.cpp
 * @brief 诊断 go2_yolo_identify 段错误的独立检测程序
 *
 * @par 使用说明
 *       check <测试编号> <图片路径>
 *       测试编号:
 *         1 - 仅测试 YOLO 模型加载与初始化 (不涉及 VideoClient)
 *         2 - 仅测试 VideoClient 构造与析构
 *         3 - 综合回归测试: 模拟 go2_yolo_identify 主流程
 *       示例: ./check 1 img.jpg         # 仅测试 YOLO 部分
 *             ./check 2                # 仅测试 VideoClient 构造
 *             ./check 3 img.jpg        # 完整流程测试
 */

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include "YOLODetector.hpp"
#include <unitree/robot/go2/video/video_client.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <csignal>
#include <cstdlib>

// ==========================================================================
// 全局段错误信号处理
// ==========================================================================

static volatile sig_atomic_t segfault_caught = 0;

static void segfaultHandler(int sig) {
    segfault_caught = 1;
    std::cerr << "\n!!! 段错误 (SIGSEGV) 已捕获 !!!" << std::endl;
    std::cerr << "   程序将尝试继续执行并给出诊断信息" << std::endl;
    // 不要 exit，让程序继续跳转到设定好的恢复点
    signal(sig, SIG_DFL); // 恢复默认，避免循环
}

#define CHECKPOINT(name) \
    do { \
        std::cout << "[CHECK] " << name << " ... " << std::flush; \
        if (segfault_caught) { \
            std::cout << "跳过 (因之前段错误)" << std::endl; \
            return 1; \
        } \
    } while(0)

#define SAFE_BLOCK_BEGIN(name) \
    do { \
        std::cout << "[CHECK] " << name << " ... " << std::flush; \
        if (segfault_caught) { \
            std::cout << "跳过 (因之前段错误)" << std::endl; \
            return 1; \
        } \
        signal(SIGSEGV, segfaultHandler); \
    } while(0)

#define SAFE_BLOCK_END(name) \
    do { \
        if (segfault_caught) { \
            std::cout << "段错误! (在 " << name << " 中)" << std::endl; \
            return 1; \
        } \
        std::cout << "OK" << std::endl; \
        signal(SIGSEGV, SIG_DFL); \
    } while(0)


// ==========================================================================
// 测试 1: YOLO 模型加载与初始化
// ==========================================================================
static int testYoloOnly(const char* imgPath) {
    std::cout << "\n========== 测试 1: YOLO 模型加载与初始化 ==========\n" << std::endl;

    // 1a. 模型加载
    SAFE_BLOCK_BEGIN("1a. YOLODetector 构造函数 (模型加载)");
    std::vector<std::string> class_names = {"stretch", "hello", "light", "one", "two"};
    std::string model_path = "/home/mafu/ai-unitree-go2-ruicom/data/best.onnx";
    YOLODetector detector(model_path, class_names, cv::Size(640, 640));
    SAFE_BLOCK_END("1a");

    // 1b. 初始化 (设置后端 + 获取输出层)
    SAFE_BLOCK_BEGIN("1b. detector.initialize(CPU)");
    if (!detector.initialize(false)) {
        std::cerr << "初始化失败!" << std::endl;
        return 1;
    }
    SAFE_BLOCK_END("1b");

    // 1c. 读取测试图片
    SAFE_BLOCK_BEGIN("1c. cv::imread 读取图片");
    cv::Mat testImage = cv::imread(imgPath, cv::IMREAD_COLOR);
    if (testImage.empty()) {
        std::cerr << "无法读取图片: " << imgPath << std::endl;
        return 1;
    }
    std::cout << "图片尺寸: " << testImage.cols << "x" << testImage.rows << std::endl;
    SAFE_BLOCK_END("1c");

    // 1d. 预处理
    SAFE_BLOCK_BEGIN("1d. 预处理 (blobFromImage)");
    cv::Mat blob;
    cv::dnn::blobFromImage(testImage, blob, 1.0/255.0, cv::Size(640,640),
                           cv::Scalar(), true, false);
    std::cout << "blob 尺寸: [" << blob.size[0] << ", " << blob.size[1]
              << ", " << blob.size[2] << ", " << blob.size[3] << "]";
    SAFE_BLOCK_END("1d");

    // 1e. 推理 + 后处理
    SAFE_BLOCK_BEGIN("1e. detect() 推理 + 后处理");
    std::vector<Detection> detections = detector.detect(testImage, 0.5f, 0.5f);
    std::cout << "检测到 " << detections.size() << " 个目标";
    SAFE_BLOCK_END("1e");

    // 1f. 打印结果
    std::cout << "\n检测结果:" << std::endl;
    for (size_t i = 0; i < detections.size(); ++i) {
        const auto& d = detections[i];
        std::cout << "  [" << i << "] " << d.class_name
                  << " conf=" << d.confidence
                  << " bbox=(" << d.bbox.x << "," << d.bbox.y
                  << "," << d.bbox.width << "," << d.bbox.height << ")"
                  << std::endl;
    }

    std::cout << "\n测试 1 通过 ✓\n" << std::endl;
    return 0;
}

// ==========================================================================
// 测试 2: VideoClient 构造与析构
// ==========================================================================
static int testVideoClientOnly() {
    std::cout << "\n========== 测试 2: VideoClient 构造与析构 ==========\n" << std::endl;

    SAFE_BLOCK_BEGIN("2a. VideoClient 默认构造");
    unitree::robot::go2::VideoClient video_client;
    SAFE_BLOCK_END("2a");

    std::cout << "\nVideoClient 对象已成功构造，现在将析构..." << std::endl;
    // 析构时也可能崩溃
    SAFE_BLOCK_BEGIN("2b. VideoClient 析构 (离开作用域)");
    {
        unitree::robot::go2::VideoClient vc2;
        // 强制离开作用域，触发析构
    }
    SAFE_BLOCK_END("2b");

    std::cout << "\n测试 2 通过 ✓\n" << std::endl;
    return 0;
}

// ==========================================================================
// 测试 3: 综合回归测试 — 模拟 go2_yolo_identify 主流程
// ==========================================================================
static int testFull(const char* imgPath) {
    std::cout << "\n========== 测试 3: 综合回归测试 ==========\n" << std::endl;

    // 3a. 模型初始化
    std::vector<std::string> class_names = {"stretch", "hello", "light", "one", "two"};
    std::string model_path = "/home/mafu/ai-unitree-go2-ruicom/data/best.onnx";

    SAFE_BLOCK_BEGIN("3a. YOLODetector 构造 + 初始化");
    YOLODetector detector(model_path, class_names, cv::Size(640, 640));
    if (!detector.initialize(false)) {
        std::cerr << "初始化失败" << std::endl;
        return 1;
    }
    SAFE_BLOCK_END("3a");

    // 3b. VideoClient 构造 (这是原程序中紧接初始化之后的步骤)
    SAFE_BLOCK_BEGIN("3b. VideoClient 构造 — 疑似崩溃点");
    unitree::robot::go2::VideoClient video_client;
    SAFE_BLOCK_END("3b");

    std::cout << "\n>>> VideoClient 构造成功，说明段错误不在构造阶段" << std::endl;
    std::cout << ">>> 原程序崩溃可能在下游环节 (cv::namedWindow / imshow / 推理)" << std::endl;

    // 3c. 读取测试图片
    SAFE_BLOCK_BEGIN("3c. cv::imread 读取图片");
    cv::Mat frame = cv::imread(imgPath, cv::IMREAD_COLOR);
    if (frame.empty()) {
        std::cerr << "无法读取图片: " << imgPath << std::endl;
        return 1;
    }
    SAFE_BLOCK_END("3c");

    // 3d. 执行检测推理
    SAFE_BLOCK_BEGIN("3d. detector.detect() 推理");
    auto detections = detector.detect(frame, 0.5f, 0.5f);
    std::cout << "检测到 " << detections.size() << " 个目标";
    SAFE_BLOCK_END("3d");

    // 3e. 绘制检测框
    SAFE_BLOCK_BEGIN("3e. YOLODetector::drawDetections() 绘制");
    YOLODetector::drawDetections(frame, detections, true);
    SAFE_BLOCK_END("3e");

    // 3f. cv::namedWindow (原程序中有此调用)
    SAFE_BLOCK_BEGIN("3f. cv::namedWindow)");
    cv::namedWindow("Check Test", cv::WINDOW_NORMAL);
    SAFE_BLOCK_END("3f");

    // 3g. cv::imshow
    SAFE_BLOCK_BEGIN("3g. cv::imshow");
    cv::imshow("Check Test", frame);
    SAFE_BLOCK_END("3g");

    // 3h. cv::waitKey (短暂显示)
    std::cout << "\n按任意键继续..." << std::endl;
    SAFE_BLOCK_BEGIN("3h. cv::waitKey");
    cv::waitKey(100);  // 100ms 后自动继续
    SAFE_BLOCK_END("3h");

    cv::destroyAllWindows();

    std::cout << "\n测试 3 通过 ✓ — 所有步骤均正常" << std::endl;
    return 0;
}

// ==========================================================================
// 主入口
// ==========================================================================
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "用法: check <测试编号> [图片路径]\n"
                  << "  1 - 仅测试 YOLO (需要图片路径)\n"
                  << "  2 - 仅测试 VideoClient 构造\n"
                  << "  3 - 完整流程 (需要图片路径)\n"
                  << "示例: ./check 1 img.jpg\n"
                  << "      ./check 2\n"
                  << "      ./check 3 img.jpg\n"
                  << std::endl;
        return 1;
    }

    int testId = std::atoi(argv[1]);

    switch (testId) {
        case 1:
            if (argc < 3) {
                std::cerr << "错误: 测试 1 需要指定图片路径" << std::endl;
                return 1;
            }
            return testYoloOnly(argv[2]);

        case 2:
            return testVideoClientOnly();

        case 3:
            if (argc < 3) {
                std::cerr << "错误: 测试 3 需要指定图片路径" << std::endl;
                return 1;
            }
            return testFull(argv[2]);

        default:
            std::cerr << "错误: 未知测试编号 " << testId << std::endl;
            return 1;
    }
}
