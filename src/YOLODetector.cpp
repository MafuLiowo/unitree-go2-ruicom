/**
 * @file YOLODetector.cpp
 * @brief 基于 YOLO 的安全标识检测与推理模块
 */
#include "YOLODetector.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <numeric>

// Helper function to split string
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

// Helper function for non-maximum suppression
static void nms(std::vector<Detection>& detections, float threshold) {
    if (detections.empty()) return;

    // Sort by confidence (descending)
    std::sort(detections.begin(), detections.end(),
              [](const Detection& a, const Detection& b) {
                  return a.confidence > b.confidence;
              });

    std::vector<bool> suppressed(detections.size(), false);

    for (size_t i = 0; i < detections.size(); ++i) {
        if (suppressed[i]) continue;

        for (size_t j = i + 1; j < detections.size(); ++j) {
            if (suppressed[j]) continue;

            // Calculate intersection over union
            cv::Rect intersection = detections[i].bbox & detections[j].bbox;
            float intersection_area = intersection.area();
            float union_area = detections[i].bbox.area() + detections[j].bbox.area() - intersection_area;
            float iou = intersection_area / union_area;

            if (iou > threshold) {
                suppressed[j] = true;
            }
        }
    }

    // Remove suppressed detections
    size_t index = 0;
    for (size_t i = 0; i < detections.size(); ++i) {
        if (!suppressed[i]) {
            detections[index++] = detections[i];
        }
    }
    detections.resize(index);
}

YOLODetector::YOLODetector(const std::string& model_path,
                           const std::vector<std::string>& class_names,
                           const cv::Size& input_size)
    : class_names_(class_names)
    , input_size_(input_size)
    , initialized_(false)
    , use_gpu_(false) {
    // Load network
    try {
        net_ = cv::dnn::readNetFromONNX(model_path);
        if (net_.empty()) {
            std::cerr << "Error: Failed to load model from " << model_path << std::endl;
            return;
        }
        std::cout << "Model loaded successfully: " << model_path << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV exception: " << e.what() << std::endl;
        return;
    }
}

YOLODetector::~YOLODetector() {
    // Net is automatically cleaned up by OpenCV
}

bool YOLODetector::initialize(bool use_gpu) {
    use_gpu_ = use_gpu;

    // Set backend preferences
    if (use_gpu_ && cv::cuda::getCudaEnabledDeviceCount() > 0) {
        try {
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            std::cout << "Using CUDA backend for GPU acceleration" << std::endl;
        } catch (const cv::Exception& e) {
            std::cerr << "CUDA backend not available: " << e.what() << std::endl;
            std::cout << "Falling back to CPU" << std::endl;
            use_gpu_ = false;
        }
    }

    if (!use_gpu_) {
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        std::cout << "Using CPU backend" << std::endl;
    }

    // Get output layer names
    output_names_ = net_.getUnconnectedOutLayersNames();
    if (output_names_.empty()) {
        std::cerr << "Error: No output layers found in the model" << std::endl;
        return false;
    }

    std::cout << "Output layers: ";
    for (const auto& name : output_names_) {
        std::cout << name << " ";
    }
    std::cout << std::endl;

    initialized_ = true;
    return true;
}

cv::Mat YOLODetector::preprocess(const cv::Mat& frame) {
    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1.0 / 255.0, input_size_, cv::Scalar(), true, false);
    return blob;
}

std::vector<Detection> YOLODetector::postprocess(const cv::Mat& output,
                                                const cv::Size& frame_size,
                                                float confidence_threshold,
                                                float nms_threshold) {
    std::vector<Detection> detections;

    // YOLOv8 output format: [1, 84, 8400] for COCO (80 classes)
    // For our model: [1, 7, 8400]? (4 bbox + 3 classes)
    // Actually need to check model output shape

    // Get output dimensions
    int dimensions = output.size[1];  // Number of features per proposal
    int num_proposals = output.size[2];  // Number of proposals

    // Reshape to [dimensions, num_proposals]
    cv::Mat output_mat = output.reshape(1, dimensions);

    // Transpose to [num_proposals, dimensions]
    cv::Mat transposed;
    cv::transpose(output_mat, transposed);

    // Get scale factors for box coordinates
    float x_scale = static_cast<float>(frame_size.width) / input_size_.width;
    float y_scale = static_cast<float>(frame_size.height) / input_size_.height;

    // Process each proposal
    for (int i = 0; i < num_proposals; ++i) {
        const float* row = transposed.ptr<float>(i);

        // First 4 values are bbox: [x_center, y_center, width, height]
        float x_center = row[0];
        float y_center = row[1];
        float width = row[2];
        float height = row[3];

        // Skip if width or height is too small
        if (width < 0.1f || height < 0.1f) continue;

        // Convert from normalized [0,1] to pixel coordinates
        float x = (x_center - width / 2.0f) * x_scale;
        float y = (y_center - height / 2.0f) * y_scale;
        width *= x_scale;
        height *= y_scale;

        // Get class scores (starting from index 4)
        int num_classes = dimensions - 4;
        if (num_classes != static_cast<int>(class_names_.size())) {
            std::cerr << "Warning: Model has " << num_classes
                      << " classes but class_names_ has " << class_names_.size() << std::endl;
            // Use minimum of the two
            num_classes = std::min(num_classes, static_cast<int>(class_names_.size()));
        }

        // Find best class
        int best_class_id = -1;
        float best_score = 0.0f;

        for (int c = 0; c < num_classes; ++c) {
            float score = row[4 + c];
            if (score > best_score) {
                best_score = score;
                best_class_id = c;
            }
        }

        // Apply confidence threshold
        if (best_score < confidence_threshold) continue;

        // Create detection
        cv::Rect bbox(static_cast<int>(x), static_cast<int>(y),
                     static_cast<int>(width), static_cast<int>(height));

        // Ensure bbox is within image bounds
        bbox = bbox & cv::Rect(0, 0, frame_size.width, frame_size.height);
        if (bbox.area() == 0) continue;

        if (best_class_id >= 0 && best_class_id < static_cast<int>(class_names_.size())) {
            detections.emplace_back(best_class_id,
                                   class_names_[best_class_id],
                                   best_score,
                                   bbox);
        }
    }

    // Apply non-maximum suppression
    nms(detections, nms_threshold);

    return detections;
}

std::vector<Detection> YOLODetector::detect(const cv::Mat& frame,
                                           float confidence_threshold,
                                           float nms_threshold) {
    if (!initialized_) {
        std::cerr << "Error: Detector not initialized" << std::endl;
        return {};
    }

    if (frame.empty()) {
        std::cerr << "Error: Empty input frame" << std::endl;
        return {};
    }

    // Preprocess
    cv::Mat blob = preprocess(frame);

    // Set input
    net_.setInput(blob);

    // Forward pass
    std::vector<cv::Mat> outputs;
    net_.forward(outputs, output_names_);

    if (outputs.empty()) {
        std::cerr << "Error: No output from network" << std::endl;
        return {};
    }

    // Postprocess (use first output)
    return postprocess(outputs[0], frame.size(), confidence_threshold, nms_threshold);
}

void YOLODetector::drawDetections(cv::Mat& frame,
                                 const std::vector<Detection>& detections,
                                 bool draw_confidence) {
    // Define colors for different classes
    static const std::vector<cv::Scalar> colors = {
        cv::Scalar(0, 255, 0),    // Green
        cv::Scalar(255, 0, 0),    // Blue
        cv::Scalar(0, 0, 255),    // Red
        cv::Scalar(255, 255, 0),  // Cyan
        cv::Scalar(255, 0, 255),  // Magenta
        cv::Scalar(0, 255, 255),  // Yellow
    };

    for (const auto& det : detections) {
        // Get color based on class ID
        cv::Scalar color = colors[det.class_id % colors.size()];

        // Draw bounding box
        cv::rectangle(frame, det.bbox, color, 2);

        // Create label
        std::string label = det.class_name;
        if (draw_confidence) {
            char conf_text[32];
            snprintf(conf_text, sizeof(conf_text), " %.2f", det.confidence);
            label += conf_text;
        }

        // Calculate text position
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        cv::Point text_org(det.bbox.x, det.bbox.y - 5);

        // Ensure text is within image bounds
        if (text_org.y < 0) {
            text_org.y = det.bbox.y + text_size.height + 5;
        }

        // Draw text background
        cv::rectangle(frame,
                     cv::Point(text_org.x, text_org.y - text_size.height - 5),
                     cv::Point(text_org.x + text_size.width, text_org.y + 5),
                     color,
                     cv::FILLED);

        // Draw text
        cv::putText(frame, label, text_org,
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
}