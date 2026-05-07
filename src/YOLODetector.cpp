/**
 * @file YOLODetector.cpp
 * @brief 基于 YOLOv8 ONNX 模型的目标检测推理模块
 *
 * @par 模块说明
 *       封装 ONNX 模型加载、预处理(blobFromImage)、CPU/GPU 推理、后处理(NMS)及可视化绘制。
 *       供 go2_yolo_identify、go2_safety_detection 等上层模块调用，无需独立运行。
 *       输入: BGR 彩色图像 (cv::Mat)
 *       输出: std::vector<Detection> 检测结果列表
 */
#include "YOLODetector.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

static std::vector<std::string> split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);
    while (std::getline(tokenStream, token, delimiter)) {
        if (!token.empty()) {
            tokens.push_back(token);
        }
    }
    return tokens;
}

/**
 * @brief 非极大值抑制 (NMS)
 * @param detections 待处理的检测结果列表（会被原地修改）
 * @param threshold IoU 阈值，高于此值视为重叠并剔除
 */
static void nms(std::vector<Detection>& detections, float threshold) {
    if (detections.empty()) return;

    std::sort(detections.begin(), detections.end(),
              [](const Detection& a, const Detection& b) {
                  return a.confidence > b.confidence;
              });

    std::vector<bool> suppressed(detections.size(), false);

    for (size_t i = 0; i < detections.size(); ++i) {
        if (suppressed[i]) continue;

        for (size_t j = i + 1; j < detections.size(); ++j) {
            if (suppressed[j]) continue;

            cv::Rect intersection = detections[i].bbox & detections[j].bbox;
            float intersection_area = intersection.area();
            float union_area = detections[i].bbox.area() + detections[j].bbox.area() - intersection_area;
            float iou = intersection_area / union_area;

            if (iou > threshold) {
                suppressed[j] = true;
            }
        }
    }

    size_t index = 0;
    for (size_t i = 0; i < detections.size(); ++i) {
        if (!suppressed[i]) {
            detections[index++] = detections[i];
        }
    }
    detections.resize(index);
}

/**
 * @brief 构造函数 — 加载 ONNX 模型文件
 * @param model_path ONNX 模型文件路径（如 "data/best.onnx"）
 * @param class_names 类别名称列表，索引与模型输出类别对应
 * @param input_size 模型输入图像尺寸，默认 640x640
 */
YOLODetector::YOLODetector(const std::string& model_path,
                           const std::vector<std::string>& class_names,
                           const cv::Size& input_size)
    : class_names_(class_names)
    , input_size_(input_size)
    , initialized_(false)
    , use_gpu_(false) {
    try {
        net_ = cv::dnn::readNetFromONNX(model_path);
        if (net_.empty()) {
            std::cerr << "错误: 无法从 " << model_path << " 加载模型" << std::endl;
            return;
        }
        std::cout << "模型加载成功: " << model_path << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV 异常: " << e.what() << std::endl;
        return;
    }
}

YOLODetector::~YOLODetector() {
}

/**
 * @brief 初始化检测器 — 设置推理后端并获取输出层名称
 * @param use_gpu 是否尝试使用 GPU 加速（CUDA），不可用时自动回退 CPU
 * @return true 初始化成功，false 初始化失败
 */
bool YOLODetector::initialize(bool use_gpu) {
    use_gpu_ = use_gpu;

    if (use_gpu_) {
        try {
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            std::cout << "使用 CUDA 后端进行 GPU 加速" << std::endl;
        } catch (const cv::Exception& e) {
            std::cerr << "CUDA 后端不可用: " << e.what() << std::endl;
            std::cout << "回退到 CPU 推理" << std::endl;
            use_gpu_ = false;
        }
    }

    if (!use_gpu_) {
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        std::cout << "使用 CPU 后端" << std::endl;
    }

    output_names_ = net_.getUnconnectedOutLayersNames();
    if (output_names_.empty()) {
        std::cerr << "错误: 模型中没有找到输出层" << std::endl;
        return false;
    }

    std::cout << "输出层: ";
    for (const auto& name : output_names_) {
        std::cout << name << " ";
    }
    std::cout << std::endl;

    initialized_ = true;
    return true;
}

/**
 * @brief 图像预处理 — 将 BGR 图像转换为模型所需的 blob
 * @param frame 输入 BGR 图像
 * @return cv::Mat 预处理后的四维 blob
 */
cv::Mat YOLODetector::preprocess(const cv::Mat& frame) {
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1.0 / 255.0, input_size_, cv::Scalar(), true, false);
    return blob;
}

/**
 * @brief 后处理 — 将模型输出解析为检测结果列表
 * @param output 模型输出张量
 * @param frame_size 原始帧尺寸，用于将归一化坐标映射回像素坐标
 * @param confidence_threshold 置信度阈值，低于此值的检测结果被丢弃
 * @param nms_threshold NMS IoU 阈值
 * @return std::vector<Detection> 过滤后的检测结果列表
 */
std::vector<Detection> YOLODetector::postprocess(const cv::Mat& output,
                                                 const cv::Size& frame_size,
                                                 float confidence_threshold,
                                                 float nms_threshold) {
    std::vector<Detection> detections;

    // YOLOv8 输出格式: [1, N+4, M] — N 个类别 + 4 个边界框坐标, M 个候选框
    int dimensions = output.size[1];
    int num_proposals = output.size[2];

    // reshape 为 [dimensions, num_proposals] 后转置方便逐行访问
    cv::Mat output_mat = output.reshape(1, dimensions);
    cv::Mat transposed;
    cv::transpose(output_mat, transposed);

    // 计算从模型输入尺寸到原始帧尺寸的缩放比例
    float x_scale = static_cast<float>(frame_size.width) / input_size_.width;
    float y_scale = static_cast<float>(frame_size.height) / input_size_.height;

    // 遍历所有候选框
    for (int i = 0; i < num_proposals; ++i) {
        const float* row = transposed.ptr<float>(i);

        // 前 4 个值为边界框: [x_center, y_center, width, height]
        float x_center = row[0];
        float y_center = row[1];
        float width = row[2];
        float height = row[3];

        // 跳过无意义的极小框
        if (width < 0.1f || height < 0.1f) continue;

        // 从归一化坐标转换为像素坐标
        float x = (x_center - width / 2.0f) * x_scale;
        float y = (y_center - height / 2.0f) * y_scale;
        width *= x_scale;
        height *= y_scale;

        // 提取类别分数（从索引 4 开始）
        int num_classes = dimensions - 4;
        if (num_classes != static_cast<int>(class_names_.size())) {
            std::cerr << "警告: 模型有 " << num_classes
                      << " 个类别, 但 class_names_ 有 " << class_names_.size() << " 个" << std::endl;
            num_classes = std::min(num_classes, static_cast<int>(class_names_.size()));
        }

        // 找到置信度最高的类别
        int best_class_id = -1;
        float best_score = 0.0f;

        for (int c = 0; c < num_classes; ++c) {
            float score = row[4 + c];
            if (score > best_score) {
                best_score = score;
                best_class_id = c;
            }
        }

        // 低于置信度阈值则丢弃
        if (best_score < confidence_threshold) continue;

        // 构建边界框
        cv::Rect bbox(static_cast<int>(x), static_cast<int>(y),
                     static_cast<int>(width), static_cast<int>(height));

        // 裁剪到图像范围内
        bbox = bbox & cv::Rect(0, 0, frame_size.width, frame_size.height);
        if (bbox.area() == 0) continue;

        // 构建检测结果
        if (best_class_id >= 0 && best_class_id < static_cast<int>(class_names_.size())) {
            detections.emplace_back(best_class_id,
                                   class_names_[best_class_id],
                                   best_score,
                                   bbox);
        }
    }

    // 执行非极大值抑制，去除重复框
    nms(detections, nms_threshold);

    return detections;
}

/**
 * @brief 对单帧图像执行目标检测
 * @param frame 输入 BGR 图像
 * @param confidence_threshold 置信度阈值 (0~1)，默认 0.5
 * @param nms_threshold NMS IoU 阈值 (0~1)，默认 0.5
 * @return std::vector<Detection> 检测结果列表
 */
std::vector<Detection> YOLODetector::detect(const cv::Mat& frame,
                                            float confidence_threshold,
                                            float nms_threshold) {
    if (!initialized_) {
        std::cerr << "错误: 检测器未初始化" << std::endl;
        return {};
    }

    if (frame.empty()) {
        std::cerr << "错误: 输入图像为空" << std::endl;
        return {};
    }

    cv::Mat blob = preprocess(frame);
    net_.setInput(blob);

    std::vector<cv::Mat> outputs;
    net_.forward(outputs, output_names_);

    if (outputs.empty()) {
        std::cerr << "错误: 网络前向传播无输出" << std::endl;
        return {};
    }

    return postprocess(outputs[0], frame.size(), confidence_threshold, nms_threshold);
}

/**
 * @brief 在图像上绘制检测结果的边界框与标签
 * @param frame 待绘制的图像（原地修改）
 * @param detections 检测结果列表
 * @param draw_confidence 是否在标签中显示置信度分数
 */
void YOLODetector::drawDetections(cv::Mat& frame,
                                  const std::vector<Detection>& detections,
                                  bool draw_confidence) {
    static const std::vector<cv::Scalar> colors = {
        cv::Scalar(0, 255, 0),    // 绿色
        cv::Scalar(255, 0, 0),    // 蓝色
        cv::Scalar(0, 0, 255),    // 红色
        cv::Scalar(255, 255, 0),  // 青色
        cv::Scalar(255, 0, 255),  // 品红
        cv::Scalar(0, 255, 255),  // 黄色
    };

    for (const auto& det : detections) {
        cv::Scalar color = colors[det.class_id % colors.size()];

        // 绘制边界框
        cv::rectangle(frame, det.bbox, color, 2);

        // 构造标签文本
        std::string label = det.class_name;
        if (draw_confidence) {
            char conf_text[32];
            snprintf(conf_text, sizeof(conf_text), " %.2f", det.confidence);
            label += conf_text;
        }

        // 计算文本位置
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        cv::Point text_org(det.bbox.x, det.bbox.y - 5);

        if (text_org.y < 0) {
            text_org.y = det.bbox.y + text_size.height + 5;
        }

        // 绘制文本背景
        cv::rectangle(frame,
                     cv::Point(text_org.x, text_org.y - text_size.height - 5),
                     cv::Point(text_org.x + text_size.width, text_org.y + 5),
                     color,
                     cv::FILLED);

        // 绘制文本
        cv::putText(frame, label, text_org,
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
}
