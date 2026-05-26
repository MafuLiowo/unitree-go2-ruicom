/**
 * @file MapLoader.hpp
 * @brief 地图加载模块，解析 YAML 地图描述文件及其对应的 PGM 图像文件
 *
 * @par 功能说明
 *       解析标准的 ROS map_server 兼容 YAML 地图元数据文件，
 *       加载对应的 PGM 图像作为占用栅格地图数据。
 *       支持将地图数据序列化为标准 ROS nav_msgs::OccupancyGrid 消息。
 */
#pragma once

#include <vector>
#include <string>
#include <cstdint>

/**
 * @brief 地图元数据结构体
 */
struct MapMetaData
{
    std::string imagePath;    ///< PGM 图像文件路径（相对于 YAML 文件）
    float       resolution   = 0.05f;  ///< 地图分辨率 (m/pixel)
    float       originX      = 0.0f;   ///< 地图原点 x 坐标 (m)
    float       originY      = 0.0f;   ///< 地图原点 y 坐标 (m)
    float       originYaw    = 0.0f;   ///< 地图原点偏航角 (rad)
    bool        negate       = false;  ///< 是否反转像素/占用关系
    float       occupiedThresh = 0.65f; ///< 占据阈值
    float       freeThresh     = 0.196f; ///< 空闲阈值
    int         width         = 0;     ///< 地图宽度（栅格数）
    int         height        = 0;     ///< 地图高度（栅格数）
};

/**
 * @brief 地图加载器类
 *
 * 解析 ROS 标准的 YAML 地图描述文件和 PGM 图像，
 * 并提供转换为不同格式的方法。
 */
class MapLoader
{
public:
    /**
     * @brief 从 YAML 文件路径加载地图
     * @param yamlPath YAML 文件路径
     * @return true 加载成功
     */
    bool loadFromYaml(const std::string& yamlPath);

    /**
     * @brief 获取地图元数据
     * @return const MapMetaData& 地图元数据引用
     */
    const MapMetaData& getMetaData() const { return meta_; }

    /**
     * @brief 获取占用栅格地图数据（行优先，0-100：空闲，-1：未知）
     * @return const std::vector<int8_t>& 地图数据引用
     */
    const std::vector<int8_t>& getMapData() const { return mapData_; }

    /**
     * @brief 获取指定栅格的值
     * @param mx 栅格 x 坐标
     * @param my 栅格 y 坐标
     * @return int8_t 栅格值（0-100：空闲，-1：未知）
     */
    int8_t at(int mx, int my) const;

    /**
     * @brief 世界坐标转栅格坐标
     * @param wx 世界 x (m)
     * @param wy 世界 y (m)
     * @param mx 输出参数：栅格 x
     * @param my 输出参数：栅格 y
     * @return true 在地图范围内
     */
    bool worldToMap(float wx, float wy, int& mx, int& my) const;

    /**
     * @brief 栅格坐标转世界坐标
     * @param mx 栅格 x
     * @param my 栅格 y
     * @param wx 输出参数：世界 x (m)
     * @param wy 输出参数：世界 y (m)
     */
    void mapToWorld(int mx, int my, float& wx, float& wy) const;

    /**
     * @brief 检查地图是否已加载
     * @return true 已加载
     */
    bool isLoaded() const { return !mapData_.empty(); }

    /**
     * @brief 将 YAML 字符串解析为键值对（简化的 YAML 解析，仅支持简单格式）
     * @param content YAML 文件内容字符串
     * @return true 解析成功
     */
    bool parseYamlContent(const std::string& content);

    /**
     * @brief 加载 PGM 图像文件
     * @param pgmPath PGM 文件路径
     * @return true 加载成功
     */
    bool loadPgm(const std::string& pgmPath);

private:
    MapMetaData meta_;
    std::vector<int8_t> mapData_;
};
