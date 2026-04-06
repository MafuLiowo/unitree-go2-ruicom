#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdint>

class LineProcessor {
public:
    LineProcessor();

    cv::Mat process(const std::vector<uint8_t>& rawData);

    void setThreshold(int threshold);
    void setROI(cv::Rect roi);

    void setBlurSize(int size);
    void setMorphSize(int size);
    void setMorphIterations(int erode, int dilate);
    void setHSVChannel(int channelIndex);
    void setUseAdaptiveThreshold(bool enable);

private:
    cv::Mat preprocess(const cv::Mat& img);
    cv::Mat binarize(const cv::Mat& gray);
    cv::Mat applyMorphology(const cv::Mat& binary);

    int _threshold;
    cv::Rect _roi;

    int _blurSize;
    int _morphSize;
    int _erodeIter;
    int _dilateIter;
    int _hsvChannel;
    bool _useAdaptive;

    cv::Mat _morphKernel;
};