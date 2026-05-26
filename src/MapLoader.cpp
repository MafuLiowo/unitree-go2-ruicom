/**
 * @file MapLoader.cpp
 * @brief 地图加载模块实现，包含 YAML 元数据解析和 PGM 图像加载
 */
#include "MapLoader.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include <cstring>
#include <cmath>

bool MapLoader::loadFromYaml(const std::string& yamlPath)
{
    std::ifstream file(yamlPath);
    if (!file.is_open()) {
        std::cerr << "[MapLoader] 无法打开 YAML 文件: " << yamlPath << std::endl;
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();

    if (!parseYamlContent(buffer.str())) {
        std::cerr << "[MapLoader] YAML 解析失败" << std::endl;
        return false;
    }

    // 确定 PGM 文件的绝对路径
    std::string yamlDir = yamlPath.substr(0, yamlPath.find_last_of("/\\") + 1);
    std::string pgmPath = yamlDir + meta_.imagePath;

    if (!loadPgm(pgmPath)) {
        std::cerr << "[MapLoader] PGM 加载失败: " << pgmPath << std::endl;
        return false;
    }

    meta_.width  = 0;
    meta_.height = 0;

    return true;
}

bool MapLoader::parseYamlContent(const std::string& content)
{
    std::istringstream stream(content);
    std::string line;

    while (std::getline(stream, line)) {
        // 去除行首行尾空格
        size_t start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) continue;
        size_t end   = line.find_last_not_of(" \t\r\n");
        line = line.substr(start, end - start + 1);

        // 跳过注释
        if (line.empty() || line[0] == '#') continue;

        // 查找冒号分隔符
        size_t colonPos = line.find(':');
        if (colonPos == std::string::npos) continue;

        std::string key = line.substr(0, colonPos);
        std::string val = line.substr(colonPos + 1);

        // 去除 key 尾部空格
        size_t keyEnd = key.find_last_not_of(" \t");
        if (keyEnd != std::string::npos) key = key.substr(0, keyEnd + 1);
        // 去除 key 首引号
        if (!key.empty() && (key[0] == '\"' || key[0] == '\'')) {
            key = key.substr(1, key.size() - 2);
        }

        // 去除 val 首尾空格和引号
        size_t valStart = val.find_first_not_of(" \t");
        if (valStart == std::string::npos) continue;
        val = val.substr(valStart);
        if (!val.empty() && (val[0] == '\"' || val[0] == '\'')) {
            val = val.substr(1, val.size() - 2);
        }

        if (key == "image") {
            meta_.imagePath = val;
        } else if (key == "resolution") {
            meta_.resolution = std::stof(val);
        } else if (key == "origin") {
            // 格式: [x, y, yaw]
            std::string originStr = val;
            // 去除方括号
            size_t b1 = originStr.find('[');
            size_t b2 = originStr.find(']');
            if (b1 != std::string::npos && b2 != std::string::npos) {
                originStr = originStr.substr(b1 + 1, b2 - b1 - 1);
            }
            // 逗号分割
            std::istringstream iss(originStr);
            std::string token;
            int idx = 0;
            while (std::getline(iss, token, ',') && idx < 3) {
                // 去除空格
                size_t tStart = token.find_first_not_of(" \t");
                if (tStart != std::string::npos) {
                    token = token.substr(tStart);
                }
                float v = std::stof(token);
                if (idx == 0)      meta_.originX = v;
                else if (idx == 1) meta_.originY = v;
                else               meta_.originYaw = v;
                idx++;
            }
        } else if (key == "negate") {
            meta_.negate = (val == "1" || val == "true" || val == "True");
        } else if (key == "occupied_thresh") {
            meta_.occupiedThresh = std::stof(val);
        } else if (key == "free_thresh") {
            meta_.freeThresh = std::stof(val);
        }
    }

    return true;
}

bool MapLoader::loadPgm(const std::string& pgmPath)
{
    std::ifstream file(pgmPath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "[MapLoader] 无法打开 PGM 文件: " << pgmPath << std::endl;
        return false;
    }

    // 读取 PGM 头
    std::string magic;
    file >> magic;
    if (magic != "P5" && magic != "P2") {
        std::cerr << "[MapLoader] 不支持的 PGM 格式: " << magic << std::endl;
        return false;
    }

    bool isBinary = (magic == "P5");

    // 跳过注释
    int width = 0, height = 0, maxVal = 0;
    std::string token;
    while (width == 0) {
        file >> token;
        if (token[0] == '#') {
            std::string dummy;
            std::getline(file, dummy);
        } else {
            width = std::stoi(token);
        }
    }
    file >> height;
    file >> maxVal;

    // 读取像素后的换行符
    if (isBinary) {
        file.get(); // 消耗换行符
    }

    meta_.width  = width;
    meta_.height = height;

    size_t totalPixels = static_cast<size_t>(width * height);
    mapData_.resize(totalPixels);

    if (isBinary) {
        // 二进制 PGM (P5)
        std::vector<unsigned char> rawPixels(totalPixels);
        file.read(reinterpret_cast<char*>(rawPixels.data()),
                  static_cast<std::streamsize>(totalPixels));

        if (static_cast<size_t>(file.gcount()) != totalPixels) {
            std::cerr << "[MapLoader] PGM 二进制数据长度不足" << std::endl;
            return false;
        }

        for (size_t i = 0; i < totalPixels; ++i) {
            float normalized = static_cast<float>(rawPixels[i]) / static_cast<float>(maxVal);
            // 翻转：白色(255)=空闲，黑色(0)=占据（标准PGM约定）
            float occupiedProb = 1.0f - normalized;

            if (meta_.negate) {
                occupiedProb = 1.0f - occupiedProb;
            }

            if (occupiedProb > meta_.occupiedThresh) {
                mapData_[i] = 100; // 占据
            } else if (occupiedProb < meta_.freeThresh) {
                mapData_[i] = 0;   // 空闲
            } else {
                mapData_[i] = -1;  // 未知
            }
        }
    } else {
        // ASCII PGM (P2)
        for (size_t i = 0; i < totalPixels; ++i) {
            int pixelVal;
            file >> pixelVal;
            float normalized = static_cast<float>(pixelVal) / static_cast<float>(maxVal);
            float occupiedProb = 1.0f - normalized;

            if (meta_.negate) {
                occupiedProb = 1.0f - occupiedProb;
            }

            if (occupiedProb > meta_.occupiedThresh) {
                mapData_[i] = 100;
            } else if (occupiedProb < meta_.freeThresh) {
                mapData_[i] = 0;
            } else {
                mapData_[i] = -1;
            }
        }
    }

    file.close();
    return true;
}

int8_t MapLoader::at(int mx, int my) const
{
    if (mx < 0 || mx >= meta_.width || my < 0 || my >= meta_.height) {
        return -1;
    }
    return mapData_[static_cast<size_t>(my * meta_.width + mx)];
}

bool MapLoader::worldToMap(float wx, float wy, int& mx, int& my) const
{
    mx = static_cast<int>(std::floor((wx - meta_.originX) / meta_.resolution));
    my = static_cast<int>(std::floor((wy - meta_.originY) / meta_.resolution));
    return (mx >= 0 && mx < meta_.width && my >= 0 && my < meta_.height);
}

void MapLoader::mapToWorld(int mx, int my, float& wx, float& wy) const
{
    wx = meta_.originX + (static_cast<float>(mx) + 0.5f) * meta_.resolution;
    wy = meta_.originY + (static_cast<float>(my) + 0.5f) * meta_.resolution;
}
