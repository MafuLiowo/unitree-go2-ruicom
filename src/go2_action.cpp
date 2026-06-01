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

// ==========================================================================
// 检测参数常量
// ==========================================================================
constexpr float CONFIDENCE_THRESHOLD = 0.5f;
constexpr int   INPUT_SIZE           = 640;

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
