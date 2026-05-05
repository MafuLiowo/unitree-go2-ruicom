Unitree_SDK2 的官方仓库位置在 @unitree_sdk2/ ，可以在这里找到 Unitree_SDK2 的接口定义以及代码示例。

在完成代码更改（如编写完新功能或修复 Bug）后，请务必同步更新项目文档：
1. **AGENTS.md**：本文件，应根据需要同步更新（如调整代码规范、仓库结构描述等）。
2. **README.md**：应包含仓库的整体架构描述，确保其与最新代码实现保持一致。
3. **docs/ 目录**：具体的模块使用说明、接口调用示例等详细文档应放在 `docs/` 目录下，不要放在 README 中。

## 代码规范与注释要求

在编写或修改代码时，必须遵循以下注释规范：

1. **文件头部注释**：每个源文件（`.cpp`/`.hpp`）必须在文件开头添加 Doxygen 风格的文件注释，包含：
   - `@file`：文件名。
   - `@brief`：简要描述该文件的用途或所属模块。
   - `@par 使用说明`：描述程序的命令行用法、参数、操作控制等（可执行程序），或模块的功能定位（库文件）。

   示例：
   ```cpp
   /**
    * @file go2_line_following.cpp
    * @brief 巡线参数调优工具，支持 RealSense 实时视频流和本地图片两种输入源
    *
    * @par 使用说明
    *       go2_line_following [image_path_or_dir]
    *       示例: ./go2_line_following                     # RealSense 模式
    *             ./go2_line_following ./images/           # 图片目录模式
    *       控制: [Space/Enter] 下一张  [q/Esc] 退出
    */
   ```

2. **中文注释**：所有代码注释必须使用中文。

3. **函数级注释**：每个函数（包括构造函数、成员函数、全局函数）都必须添加规范的 Doxygen 风格注释，包含：
   - `@brief`：简要描述函数的功能。
   - `@param`：详细说明每个参数的含义及用途。
   - `@return`：说明返回值的含义（如果有）。

4. **关键逻辑注释**：对于函数内部复杂的算法步骤或关键逻辑块，需添加必要的行间注释。

示例：
```cpp
/**
 * @brief 计算线条质心横坐标
 * @param binary 输入的二值化图像
 * @return float 线条中心的x坐标，如果未检测到线条则返回 -1.0
 */
float getLineCenter(const cv::Mat& binary);
```

## 仓库文件结构描述

```text
.
├── AGENTS.md               # 本文件，存放 Agent 指令及仓库结构描述
├── CMakeLists.txt          # 项目 CMake 构建配置文件
├── README.md               # 项目自述文件，包含整体架构描述及环境准备说明
├── build/                  # 编译构建目录（通常在 .gitignore 中忽略）
├── docs/                   # 详细模块文档及使用说明目录
│   ├── LineProcessor.md    # LineProcessor 模块的使用示例与参数配置
│   └── safety_detection.md # 安全检测模块文档
├── include/                # 头文件目录
│   ├── Go2SportSwitch.hpp  # Go2 高层运动控制封装，提供 Stretch/Hello/StopMove/FrontJump/WalkStair
│   └── ImageProcessor.hpp  # 图像处理模块接口（连通域分析、路径检测）
├── src/                    # 源文件目录
│   ├── go2_video_display.cpp     # RealSense 实时画面显示程序
│   ├── go2_process_image.cpp     # HSV 颜色识别与 V 通道反二值化程序
│   ├── go2_process_images.cpp    # 图像处理模块实现（连通域分析/路径检测）
│   ├── go2_seeking_way.cpp       # 视觉寻路主程序，基于连通域分析的路径跟踪
│   ├── go2_sport_switch.cpp      # 高层运动控制交互程序，支持 Stretch/Hello/StopMove/FrontJump/WalkStair
│   ├── go2_light_controller.cpp  # 灯光控制客户端，控制 Go2 机器人 LED 灯效
│   └── go2_images_identify.cpp   # 图像标识识别程序（待实现）

└── unitree_sdk2/           # Unitree SDK2 库及头文件
    ├── build/              # Build artifacts (binaries, CMake cache)
    ├── cmake/              # CMake configuration files
    ├── CMakeLists.txt      # Project CMake build configuration
    ├── example/            # Example code for different robot models and features
    │   ├── go2/            # Go2 robot examples
    │   ├── g1/             # G1 robot examples
    │   ├── h1/             # H1 robot examples
    │   ├── a2/             # A2 robot examples
    │   └── ...
    ├── include/            # Header files
    │   └── unitree/        # Core SDK headers
    ├── lib/                # Compiled libraries
    ├── LICENSE             # License information
    ├── README.md           # Original SDK README
    └── thirdparty/         # Third-party dependencies
```