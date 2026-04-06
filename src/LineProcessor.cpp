#include "LineProcessor.hpp"

LineProcessor::LineProcessor()
    : _threshold(60), _roi(0, 0, 0, 0)
    , _blurSize(5), _morphSize(3)
    , _erodeIter(1), _dilateIter(2)
    , _hsvChannel(2), _useAdaptive(false)
{
    _morphKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(_morphSize, _morphSize));
}

cv::Mat LineProcessor::process(const std::vector<uint8_t>& rawData)
{
    if (rawData.empty()) {
        return cv::Mat();
    }

    cv::Mat rawImg(rawData);
    cv::Mat img = cv::imdecode(rawImg, cv::IMREAD_COLOR);
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

cv::Mat LineProcessor::applyMorphology(const cv::Mat& binary)
{
    cv::Mat eroded, dilated;
    cv::erode(binary, eroded, _morphKernel, cv::Point(-1, -1), _erodeIter);
    cv::dilate(eroded, dilated, _morphKernel, cv::Point(-1, -1), _dilateIter);
    return dilated;
}

void LineProcessor::setThreshold(int threshold)
{
    _threshold = threshold;
}

void LineProcessor::setROI(cv::Rect roi)
{
    _roi = roi;
}

void LineProcessor::setBlurSize(int size)
{
    _blurSize = (size % 2 == 1) ? size : size + 1;
}

void LineProcessor::setMorphSize(int size)
{
    _morphSize = size;
    _morphKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(_morphSize, _morphSize));
}

void LineProcessor::setMorphIterations(int erode, int dilate)
{
    _erodeIter = erode;
    _dilateIter = dilate;
}

void LineProcessor::setHSVChannel(int channelIndex)
{
    _hsvChannel = std::clamp(channelIndex, 0, 2);
}

void LineProcessor::setUseAdaptiveThreshold(bool enable)
{
    _useAdaptive = enable;
}