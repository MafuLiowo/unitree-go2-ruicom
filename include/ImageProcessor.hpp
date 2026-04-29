#ifndef __GO2_IMAGE_PROCESSOR_HPP__
#define __GO2_IMAGE_PROCESSOR_HPP__

#include <opencv2/opencv.hpp>

struct PathAnalysis {
    float midpointX = -1.0f;
    bool pathFound = false;
    float whiteRatio = 0.0f;
    bool isTurn = false;
    int turnDirection = 0;
    float leftWhiteRatio = 0.0f;
    float centerWhiteRatio = 0.0f;
    float rightWhiteRatio = 0.0f;
    cv::Mat displayImage;
};

cv::Mat toHighContrast(const cv::Mat& frame, int thresholdValue = 60);

PathAnalysis analyzePath(const cv::Mat& binary);

#endif
