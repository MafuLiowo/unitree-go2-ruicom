Unitree_SDK2 的官方仓库位置在 @unitree_sdk2/ ，可以在这里找到 Unitree_SDK2 的接口定义以及代码示例。

在完成代码更改（如编写完新功能或修复 Bug）后，请务必同步更新项目文档：
1. **README.md**：应包含仓库的整体架构描述，确保其与最新代码实现保持一致。
2. **docs/ 目录**：具体的模块使用说明、接口调用示例等详细文档应放在 `docs/` 目录下，不要放在 README 中。
3. **AGENTS.md**：应包含整个仓库的文件结构描述，并保持实时更新。

## 仓库文件结构描述

```text
.
├── AGENTS.md            # 本文件，存放 Agent 指令及仓库结构描述
├── CMakeLists.txt       # 项目 CMake 构建配置文件
├── README.md            # 项目自述文件，包含整体架构描述
├── build/               # 编译构建目录（通常在 .gitignore 中忽略）
├── docs/                # 详细模块文档及使用说明目录
│   └── LineProcessor.md # LineProcessor 模块的使用示例与参数配置
├── include/             # 头文件目录
│   └── LineProcessor.hpp
├── src/                 # 源文件目录
│   └── LineProcessor.cpp
├── go2_video_client.cpp # 主程序入口，基于 Unitree SDK2 的视频客户端示例
└── unitree_sdk2/        # Unitree SDK2 库及头文件（已在项目中引用）
```