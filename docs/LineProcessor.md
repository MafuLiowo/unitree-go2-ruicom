# LineProcessor 模块使用说明

`LineProcessor` 是一个用于处理 Go2 机器人视频流图像的工具类，主要功能包括二值化、形态学操作、自适应阈值处理等。

## 使用示例

```cpp
#include "LineProcessor.hpp"

int main()
{
    unitree::robot::ChannelFactory::Instance()->Init(0);
    unitree::robot::go2::VideoClient video_client;
    video_client.SetTimeout(1.0f);
    video_client.Init();

    LineProcessor processor;
    processor.setConfig(60, cv::Rect(0, 240, 640, 240));

    std::vector<uint8_t> image_sample;
    cv::Mat mask;

    while (true) {
        video_client.GetImageSample(image_sample);
        mask = processor.process(image_sample);

        if (!mask.empty()) {
            cv::imshow("Mask", mask);
        }

        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) break;
    }

    return 0;
}
```

## 动态调参接口

`LineProcessor` 提供了丰富的Setter方法，可在运行时调整图像处理参数：

| 方法 | 说明 | 默认值 |
|------|------|--------|
| `setThreshold(int)` | 二值化阈值 | 60 |
| `setROI(cv::Rect)` | 感兴趣区域 | 空 |
| `setBlurSize(int)` | 高斯模糊核大小（需为奇数） | 5 |
| `setMorphSize(int)` | 形态学操作核大小 | 3 |
| `setMorphIterations(int erode, int dilate)` | 腐蚀/膨忙迭代次数 | 1, 2 |
| `setHSVChannel(int)` | HSV通道选择：0-H, 1-S, 2-V | 2 (V) |
| `setUseAdaptiveThreshold(bool)` | 启用自适应阈值 | false |
| `getLineCenter(cv::Mat)` | 计算线条质心x坐标 | - |

## 巡线功能接口

- `cv::Mat process(const cv::Mat& frame)`: 直接接收OpenCV矩阵进行处理。
- `float getLineCenter(const cv::Mat& binary)`: 接收二值化后的图像（通常是 `process` 的返回结果），返回线条在图像中的横坐标质心（0 到 cols 之间）。如果未检测到线条，返回 `-1.0`。

## 典型配置示例

```cpp
// 明亮环境下的标准配置
processor.setThreshold(80);
processor.setBlurSize(5);
processor.setMorphSize(3);
processor.setMorphIterations(1, 2);
processor.setHSVChannel(2);  // 使用V通道
processor.setUseAdaptiveThreshold(false);

// 光照不均环境下的鲁棒配置
processor.setBlurSize(7);
processor.setMorphSize(5);
processor.setMorphIterations(2, 3);
processor.setHSVChannel(1);  // 尝试使用S通道
processor.setUseAdaptiveThreshold(true);  // 启用自适应阈值
```

> **提示**：如果图像模糊效果不佳，尝试增大 `setBlurSize` 的值；如需去除更多噪点，可增加 `setMorphIterations` 的膨胀次数。