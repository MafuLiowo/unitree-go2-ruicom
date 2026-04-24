/**
 * @file LineProcessor.cpp
 * @brief 图像线条检测处理器，提供图像预处理、二值化、形态学操作和线条质心计算
 *
 * @par 模块说明
 *       本模块封装了完整的线条检测管线:
 *       彩色图 → HSV通道提取 → 高斯模糊 → 阈值二值化 → 形态学(腐蚀+膨胀) → ROI裁剪 → 质心计算
 *       供 go2_line_following 等上层模块调用，无需独立运行。
 */
#include "LineProcessor.hpp"

/**
 * @brief 构造函数，初始化默认图像处理参数
 */
LineProcessor::LineProcessor()
    : _threshold(60), _roi(0, 0, 0, 0)
    , _blurSize(5), _morphSize(3)
    , _erodeIter(1), _dilateIter(2)
    , _hsvChannel(2), _useAdaptive(false)
{
    _morphKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(_morphSize, _morphSize));
}

/**
 * @brief 处理原始字节流图像数据
 * @param rawData 原始图像字节向量（通常是编码后的JPG数据）
 * @return cv::Mat 处理后的二值化图像（如果设置了ROI，则返回ROI区域）
 */
cv::Mat LineProcessor::process(const std::vector<uint8_t>& rawData)
{
    if (rawData.empty()) {
        return cv::Mat();
    }

    cv::Mat rawImg(rawData);
    cv::Mat img = cv::imdecode(rawImg, cv::IMREAD_COLOR);
    return process(img);
}

/**
 * @brief 核心处理流程：预处理 -> 二值化 -> 形态学处理 -> ROI裁剪
 * @param img 输入的彩色图像 (cv::Mat)
 * @return cv::Mat 处理后的二值化图像
 */
cv::Mat LineProcessor::process(const cv::Mat& img)
{
    if (img.empty()) {
        return cv::Mat();
    }

    cv::Mat preprocessed = preprocess(img);
    cv::Mat binary = binarize(preprocessed);
    cv::Mat morphed = applyMorphology(binary);

    if (_roi.width > 0 && _roi.height > 0) {
        cv::Rect clipped = _roi & cv::Rect(0, 0, morphed.cols, morphed.rows);
        return morphed(clipped).clone();
    }

    return morphed;
}

/**
 * @brief 计算二值化图像中线条的质心横坐标
 * @param binary 输入的二值化图像
 * @return float 线条中心的x坐标，如果未检测到线条则返回 -1.0
 */
float LineProcessor::getLineCenter(const cv::Mat& binary)
{
    if (binary.empty()) return -1.0f;

    cv::Moments m = cv::moments(binary, true);
    if (m.m00 > 0) {
        return static_cast<float>(m.m10 / m.m00);
    }
    return -1.0f;
}

/**
 * @brief 图像预处理：颜色空间转换、通道提取及高斯模糊
 * @param img 输入的彩色图像
 * @return cv::Mat 预处理后的灰度图/单通道图
 */
cv::Mat LineProcessor::preprocess(const cv::Mat& img)
{
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    cv::Mat target = channels[_hsvChannel];

    cv::Mat blurred;
    cv::GaussianBlur(target, blurred, cv::Size(_blurSize, _blurSize), 0);

    return blurred;
}

/**
 * @brief 二值化处理
 * @param gray 输入的灰度图
 * @return cv::Mat 二值化后的图像
 */
cv::Mat LineProcessor::binarize(const cv::Mat& gray)
{
    cv::Mat binary;
    if (_useAdaptive) {
        cv::adaptiveThreshold(gray, binary, 255,
                             cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                             cv::THRESH_BINARY_INV, 11, 2);
    } else {
        cv::threshold(gray, binary, _threshold, 255, cv::THRESH_BINARY_INV);
    }
    return binary;
}

/**
 * @brief 执行形态学操作（先腐蚀后膨胀），用于去除噪声和填充空隙
 * @param binary 输入的二值图像
 * @return cv::Mat 处理后的图像
 */
cv::Mat LineProcessor::applyMorphology(const cv::Mat& binary)
{
    cv::Mat eroded, dilated;
    cv::erode(binary, eroded, _morphKernel, cv::Point(-1, -1), _erodeIter);
    cv::dilate(eroded, dilated, _morphKernel, cv::Point(-1, -1), _dilateIter);
    return dilated;
}

/**
 * @brief 设置二值化阈值
 * @param threshold 阈值 (0-255)
 */
void LineProcessor::setThreshold(int threshold)
{
    _threshold = threshold;
}

/**
 * @brief 设置感兴趣区域 (ROI)
 * @param roi ROI矩形区域
 */
void LineProcessor::setROI(cv::Rect roi)
{
    _roi = roi;
}

/**
 * @brief 设置高斯模糊核大小
 * @param size 核大小（会自动转换为奇数）
 */
void LineProcessor::setBlurSize(int size)
{
    _blurSize = (size % 2 == 1) ? size : size + 1;
}

/**
 * @brief 设置形态学操作的核大小
 * @param size 核大小
 */
void LineProcessor::setMorphSize(int size)
{
    _morphSize = size;
    _morphKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(_morphSize, _morphSize));
}

/**
 * @brief 设置形态学迭代次数
 * @param erode 腐蚀次数
 * @param dilate 膨胀次数
 */
void LineProcessor::setMorphIterations(int erode, int dilate)
{
    _erodeIter = erode;
    _dilateIter = dilate;
}

/**
 * @brief 设置处理所使用的 HSV 通道
 * @param channelIndex 通道索引 (0:H, 1:S, 2:V)
 */
void LineProcessor::setHSVChannel(int channelIndex)
{
    _hsvChannel = std::clamp(channelIndex, 0, 2);
}

/**
 * @brief 是否启用自适应阈值
 * @param enable true 为启用
 */
void LineProcessor::setUseAdaptiveThreshold(bool enable)
{
    _useAdaptive = enable;
}
