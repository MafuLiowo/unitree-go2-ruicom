#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>

/**
 * @brief 检测结果结构体
 */
struct Detection {
    int class_id;           ///< 类别 ID
    std::string class_name; ///< 类别名称
    float confidence;       ///< 置信度 (0~1)
    cv::Rect bbox;          ///< 边界框 (x, y, width, height)

    Detection() : class_id(-1), confidence(0.0f) {}
    Detection(int id, const std::string& name, float conf, const cv::Rect& box)
        : class_id(id), class_name(name), confidence(conf), bbox(box) {}
};

/**
 * @brief 基于 YOLOv8 ONNX 模型的目标检测器，使用 OpenCV DNN 后端
 *
 * 专为 Go2 机器狗 ARM CPU 平台优化：自动检测可用后端并优先选择推理速度
 * 最快的方案，无需手动指定 GPU/CUDA。
 *
 * 后端选择优先级: OpenVINO (DNN_BACKEND_INFERENCE_ENGINE)
 *               → ARM NEON FP16 (DNN_TARGET_CPU_FP16)
 *               → OpenCV CPU (DNN_BACKEND_OPENCV)
 */
class YOLODetector {
public:
    /**
     * @brief 构造函数 — 加载 ONNX 模型文件
     * @param model_path ONNX 模型文件路径（如 "data/best.onnx"）
     * @param class_names 类别名称列表，索引与模型输出类别对应
     * @param input_size 模型输入图像尺寸，默认 640x640
     */
    YOLODetector(const std::string& model_path,
                 const std::vector<std::string>& class_names,
                 const cv::Size& input_size = cv::Size(640, 640));

    ~YOLODetector();

    /**
     * @brief 初始化检测器 — 自动选择最优 ARM CPU 推理后端
     * @return true 初始化成功，false 初始化失败
     *
     * 后端选择策略:
     * 1. 尝试 OpenVINO (DNN_BACKEND_INFERENCE_ENGINE + DNN_TARGET_CPU)
     * 2. 回退 ARM NEON FP16 (DNN_BACKEND_OPENCV + DNN_TARGET_CPU_FP16)
     * 3. 最终回退标准 OpenCV CPU (DNN_BACKEND_OPENCV + DNN_TARGET_CPU)
     *
     * 同时自动设置 OpenCV 线程数为 CPU 核心数以充分利用多核 ARM 处理器。
     */
    bool initialize();

    /**
     * @brief 对单帧图像执行目标检测
     * @param frame 输入 BGR 图像
     * @param confidence_threshold 置信度阈值 (0~1)，默认 0.5
     * @param nms_threshold NMS IoU 阈值 (0~1)，默认 0.5
     * @return std::vector<Detection> 检测结果列表
     */
    std::vector<Detection> detect(const cv::Mat& frame,
                                  float confidence_threshold = 0.5f,
                                  float nms_threshold = 0.5f);

    /**
     * @brief 在图像上绘制检测结果的边界框与标签
     * @param frame 待绘制的图像（原地修改）
     * @param detections 检测结果列表
     * @param draw_confidence 是否在标签中显示置信度分数
     */
    static void drawDetections(cv::Mat& frame,
                               const std::vector<Detection>& detections,
                               bool draw_confidence = true);

    /**
     * @brief 获取模型要求的输入尺寸
     * @return cv::Size 输入图像尺寸
     */
    cv::Size getInputSize() const { return input_size_; }

    /**
     * @brief 检查检测器是否已初始化
     * @return true 已初始化
     */
    bool isInitialized() const { return initialized_; }

private:
    YOLODetector(const YOLODetector&) = delete;
    YOLODetector& operator=(const YOLODetector&) = delete;

    /**
     * @brief 图像预处理 — 将 BGR 图像转换为模型所需的 blob
     * @param frame 输入 BGR 图像
     * @return cv::Mat 预处理后的四维 blob
     */
    cv::Mat preprocess(const cv::Mat& frame);

    /**
     * @brief 后处理 — 将模型输出解析为检测结果列表
     * @param output 模型输出张量
     * @param frame_size 原始帧尺寸，用于坐标映射
     * @param confidence_threshold 置信度阈值
     * @param nms_threshold NMS IoU 阈值
     * @return std::vector<Detection> 过滤后的检测结果列表
     */
    std::vector<Detection> postprocess(const cv::Mat& output,
                                       const cv::Size& frame_size,
                                       float confidence_threshold,
                                       float nms_threshold);

    cv::dnn::Net net_;                      ///< OpenCV DNN 网络
    std::vector<std::string> class_names_;  ///< 类别名称列表
    cv::Size input_size_;                   ///< 模型输入尺寸
    bool initialized_;                      ///< 初始化标志
    std::vector<std::string> output_names_; ///< 输出层名称列表
    std::string backend_name_;              ///< 当前使用的推理后端名称 (用于日志)
    int num_threads_;                       ///< CPU 推理线程数
};
