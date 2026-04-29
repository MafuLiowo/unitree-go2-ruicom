/**
 * @file go2_safety_detection.cpp
 * @brief 安全标识识别与检测模块，结合 YOLO 检测和运动控制进行实时响应
 *
 * @par 使用说明
 *       go2_safety_detection <network_interface>
 *       示例: ./go2_safety_detection eth0
 *       控制: [s] 保存当前帧  [q/Esc] 退出
 *       说明: 模型文件需位于 ../models/yolov8n_safety_signs.onnx
 */
#include <unitree/robot/go2/video/video_client.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <opencv2/opencv.hpp>
#include "YOLODetector.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <queue>
#include <atomic>
#include <thread>
#include <mutex>

// Configuration constants
constexpr float CONFIDENCE_THRESHOLD = 0.7f;      // Minimum confidence for detection
constexpr float NMS_THRESHOLD = 0.5f;             // Non-maximum suppression threshold
constexpr int CONSECUTIVE_FRAMES_THRESHOLD = 3;   // Required consecutive detections
constexpr int ACTION_COOLDOWN_MS = 5000;          // Cooldown between actions (ms)

/**
 * @brief Class to manage action triggering based on detection results
 */
class ActionTrigger {
public:
    ActionTrigger() : last_action_time_(std::chrono::steady_clock::now()) {}

    /**
     * @brief Check if an action should be triggered
     * @param class_name Detected class name
     * @param confidence Detection confidence
     * @return true if action should be triggered
     */
    bool shouldTrigger(const std::string& class_name, float confidence) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Check confidence threshold
        if (confidence < CONFIDENCE_THRESHOLD) {
            return false;
        }

        // Check cooldown
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_action_time_).count();

        if (elapsed < ACTION_COOLDOWN_MS) {
            return false;
        }

        // Update detection history
        detection_history_[class_name].push_back(true);

        // Keep only recent history
        if (detection_history_[class_name].size() > CONSECUTIVE_FRAMES_THRESHOLD) {
            detection_history_[class_name].pop_front();
        }

        // Check if we have enough consecutive detections
        if (detection_history_[class_name].size() == CONSECUTIVE_FRAMES_THRESHOLD) {
            bool all_detected = true;
            for (bool detected : detection_history_[class_name]) {
                if (!detected) {
                    all_detected = false;
                    break;
                }
            }

            if (all_detected) {
                last_action_time_ = now;
                detection_history_[class_name].clear();  // Reset after triggering
                return true;
            }
        }

        return false;
    }

    /**
     * @brief Update detection history (call every frame)
     * @param detections Current frame detections
     */
    void updateFrame(const std::vector<Detection>& detections) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Mark all tracked classes as not detected in this frame
        for (auto& [class_name, history] : detection_history_) {
            history.push_back(false);

            // Keep only recent history
            if (history.size() > CONSECUTIVE_FRAMES_THRESHOLD) {
                history.pop_front();
            }
        }

        // Mark actually detected classes
        for (const auto& det : detections) {
            if (det.confidence >= CONFIDENCE_THRESHOLD) {
                detection_history_[det.class_name].push_back(true);

                // Keep only recent history
                if (detection_history_[det.class_name].size() > CONSECUTIVE_FRAMES_THRESHOLD) {
                    detection_history_[det.class_name].pop_front();
                }
            }
        }
    }

private:
    std::map<std::string, std::deque<bool>> detection_history_;
    std::chrono::steady_clock::time_point last_action_time_;
    std::mutex mutex_;
};

/**
 * @brief Perform action based on detected class
 * @param sport_client Unitree sport client
 * @param class_name Detected class name
 */
void performAction(unitree::robot::go2::SportClient& sport_client,
                   const std::string& class_name) {
    std::cout << ">>> Performing action for: " << class_name << std::endl;

    if (class_name == "caution_shock") {
        std::cout << "    Action: Stretch (伸懒腰)" << std::endl;
        sport_client.Stretch();
    } else if (class_name == "caution_oxidizer") {
        std::cout << "    Action: Hello (打招呼)" << std::endl;
        sport_client.Hello();
    } else {
        std::cout << "    No action defined for class: " << class_name << std::endl;
    }
}

int main(int argc, char** argv) {
    // Parse command line arguments
    std::string netInterface;
    if (argc > 1) {
        netInterface = argv[1];
    } else {
        std::cout << "Usage: " << argv[0] << " <network_interface>" << std::endl;
        std::cout << "Example: " << argv[0] << " eth0" << std::endl;
        return -1;
    }

    std::cout << "========================================" << std::endl;
    std::cout << "Go2 Safety Sign Detection System" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Network interface: " << netInterface << std::endl;
    std::cout << "Confidence threshold: " << CONFIDENCE_THRESHOLD << std::endl;
    std::cout << "Consecutive frames required: " << CONSECUTIVE_FRAMES_THRESHOLD << std::endl;
    std::cout << "Action cooldown: " << ACTION_COOLDOWN_MS << " ms" << std::endl;
    std::cout << "========================================" << std::endl;

    // Initialize Unitree SDK
    std::cout << "\nInitializing Unitree SDK..." << std::endl;
    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);
        std::cout << "SDK initialized successfully" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize SDK: " << e.what() << std::endl;
        return -1;
    }

    // Initialize video client
    std::cout << "Initializing video client..." << std::endl;
    unitree::robot::go2::VideoClient video_client;
    video_client.SetTimeout(1.0f);
    video_client.Init();  // Init returns void
    std::cout << "Video client initialized" << std::endl;

    // Initialize sport client
    std::cout << "Initializing sport client..." << std::endl;
    unitree::robot::go2::SportClient sport_client;
    sport_client.SetTimeout(10.0f);
    sport_client.Init();  // Init returns void
    std::cout << "Sport client initialized" << std::endl;

    // Load class names (from dataset)
    std::vector<std::string> class_names = {
        "caution_oxidizer",
        "caution_radiation",
        "caution_shock"
    };

    std::cout << "\nClass names loaded:" << std::endl;
    for (size_t i = 0; i < class_names.size(); ++i) {
        std::cout << "  " << i << ": " << class_names[i] << std::endl;
    }

    // Initialize YOLO detector
    std::cout << "\nInitializing YOLO detector..." << std::endl;
    std::string model_path = "../models/yolov8n_safety_signs.onnx";
    YOLODetector detector(model_path, class_names, cv::Size(640, 640));

    if (!detector.initialize(true)) {  // Try GPU first
        std::cerr << "Failed to initialize YOLO detector" << std::endl;
        return -1;
    }
    std::cout << "YOLO detector initialized" << std::endl;

    // Initialize action trigger
    ActionTrigger action_trigger;

    // Main loop
    std::cout << "\nStarting main loop. Press 'q' or ESC to exit." << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  [q] or [ESC] - Exit program" << std::endl;
    std::cout << "  [s] - Save current frame" << std::endl;

    std::vector<uint8_t> image_sample;
    int frame_count = 0;
    int save_counter = 0;

    cv::namedWindow("Go2 Safety Sign Detection", cv::WINDOW_AUTOSIZE);

    while (true) {
        // Get frame from camera
        int ret = video_client.GetImageSample(image_sample);

        if (ret == 0 && !image_sample.empty()) {
            // Decode image
            cv::Mat rawData(image_sample);
            cv::Mat frame = cv::imdecode(rawData, cv::IMREAD_COLOR);

            if (!frame.empty()) {
                frame_count++;

                // Run detection
                auto detections = detector.detect(frame, CONFIDENCE_THRESHOLD, NMS_THRESHOLD);

                // Update action trigger with current detections
                action_trigger.updateFrame(detections);

                // Check for actions to trigger
                for (const auto& det : detections) {
                    if (action_trigger.shouldTrigger(det.class_name, det.confidence)) {
                        performAction(sport_client, det.class_name);
                    }
                }

                // Draw detections on frame
                YOLODetector::drawDetections(frame, detections);

                // Display frame counter and info
                std::string status_text = "Frame: " + std::to_string(frame_count);
                cv::putText(frame, status_text, cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

                if (!detections.empty()) {
                    std::string detection_text = "Detections: " + std::to_string(detections.size());
                    cv::putText(frame, detection_text, cv::Point(10, 60),
                               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                }

                // Show frame
                cv::imshow("Go2 Safety Sign Detection", frame);

                // Handle keyboard input
                char key = (char)cv::waitKey(1);

                if (key == 'q' || key == 27) {  // 'q' or ESC
                    std::cout << "\nExiting..." << std::endl;
                    break;
                } else if (key == 's' || key == 'S') {
                    // Save current frame
                    auto now = std::chrono::system_clock::now();
                    auto time = std::chrono::system_clock::to_time_t(now);
                    char timestamp[64];
                    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&time));

                    std::string filename = "detection_" + std::string(timestamp) +
                                          "_" + std::to_string(save_counter++) + ".jpg";

                    if (cv::imwrite(filename, frame)) {
                        std::cout << ">>> Saved frame: " << filename << std::endl;
                    } else {
                        std::cerr << "!!! Failed to save frame: " << filename << std::endl;
                    }
                }
            }
        } else {
            // No frame available, just check for exit
            if ((char)cv::waitKey(1) == 'q') break;
        }

        // Print progress every 100 frames
        if (frame_count % 100 == 0) {
            std::cout << "Processed " << frame_count << " frames" << std::endl;
        }
    }

    // Cleanup
    cv::destroyAllWindows();
    std::cout << "\nProgram terminated. Total frames processed: " << frame_count << std::endl;

    return 0;
}