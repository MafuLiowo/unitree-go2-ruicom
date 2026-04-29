#include "ImageProcessor.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>

cv::Mat toHighContrast(const cv::Mat& frame, int thresholdValue)
{
    cv::Mat gray, highContrast;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    cv::threshold(gray, highContrast, thresholdValue, 255, cv::THRESH_BINARY_INV);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::dilate(highContrast, highContrast, kernel, cv::Point(-1, -1), 1);

    return highContrast;
}

PathAnalysis analyzePath(const cv::Mat& binary)
{
    PathAnalysis result;
    if (binary.empty()) return result;

    cv::Mat display;
    cv::cvtColor(binary, display, cv::COLOR_GRAY2BGR);

    int imgWidth = binary.cols;
    int imgHeight = binary.rows;

    int nearStart = imgHeight * 60 / 100;
    int nearEnd   = imgHeight * 95 / 100;
    int nearHeight = nearEnd - nearStart;
    cv::Rect nearRoi(0, nearStart, imgWidth, nearHeight);
    cv::Mat nearZone = binary(nearRoi);

    int farStart = imgHeight * 20 / 100;
    int farEnd   = imgHeight * 55 / 100;
    int farHeight = farEnd - farStart;
    cv::Rect farRoi(0, farStart, imgWidth, farHeight);
    cv::Mat farZone = binary(farRoi);

    int leftBandEnd   = imgWidth / 3;
    int rightBandStart = imgWidth * 2 / 3;
    int centerWidth   = rightBandStart - leftBandEnd;

    cv::Rect leftRect(0, 0, leftBandEnd, nearHeight);
    cv::Rect centerRect(leftBandEnd, 0, centerWidth, nearHeight);
    cv::Rect rightRect(rightBandStart, 0, imgWidth - rightBandStart, nearHeight);

    int leftPixels   = cv::countNonZero(nearZone(leftRect));
    int centerPixels = cv::countNonZero(nearZone(centerRect));
    int rightPixels  = cv::countNonZero(nearZone(rightRect));

    float totalNearPixels = (float)nearHeight * imgWidth;
    float totalLeft   = (float)nearHeight * leftBandEnd;
    float totalCenter = (float)nearHeight * centerWidth;
    float totalRight  = (float)nearHeight * (imgWidth - rightBandStart);

    result.leftWhiteRatio   = (totalLeft > 0)   ? leftPixels / totalLeft : 0.0f;
    result.centerWhiteRatio = (totalCenter > 0) ? centerPixels / totalCenter : 0.0f;
    result.rightWhiteRatio  = (totalRight > 0)  ? rightPixels / totalRight : 0.0f;

    float totalWhite = (float)(leftPixels + centerPixels + rightPixels);
    result.whiteRatio = (totalNearPixels > 0) ? totalWhite / totalNearPixels : 0.0f;

    if (totalWhite < 15)
    {
        result.pathFound = false;
        result.midpointX = -1.0f;
    }
    else
    {
        result.pathFound = true;

        float weightedSum = 0.0f;
        for (int col = 0; col < imgWidth; col++)
        {
            int whiteCount = cv::countNonZero(
                nearZone(cv::Rect(col, 0, 1, nearHeight)));
            weightedSum += (float)whiteCount * ((float)col / (float)imgWidth);
        }
        result.midpointX = weightedSum / totalWhite;

        cv::Rect farLeftRect(0, 0, leftBandEnd, farHeight);
        cv::Rect farCenterRect(leftBandEnd, 0, centerWidth, farHeight);
        cv::Rect farRightRect(rightBandStart, 0, imgWidth - rightBandStart, farHeight);

        int farLeftPixels   = cv::countNonZero(farZone(farLeftRect));
        int farCenterPixels = cv::countNonZero(farZone(farCenterRect));
        int farRightPixels  = cv::countNonZero(farZone(farRightRect));

        float farLeftRatio   = (totalLeft > 0)   ? farLeftPixels / totalLeft : 0.0f;
        float farCenterRatio = (totalCenter > 0) ? farCenterPixels / totalCenter : 0.0f;
        float farRightRatio  = (totalRight > 0)  ? farRightPixels / totalRight : 0.0f;

        bool nearOffCenter = result.midpointX < 0.15f || result.midpointX > 0.85f;
        bool nearCenterEmpty = result.centerWhiteRatio < 0.015f;
        bool strongLeftSide = result.leftWhiteRatio > result.centerWhiteRatio * 4.0f
                              && result.leftWhiteRatio > 0.03f;
        bool strongRightSide = result.rightWhiteRatio > result.centerWhiteRatio * 4.0f
                               && result.rightWhiteRatio > 0.03f;

        if (nearOffCenter && nearCenterEmpty && farCenterRatio < 0.02f)
        {
            if (strongLeftSide && result.leftWhiteRatio > farLeftRatio * 0.5f)
            {
                result.isTurn = true;
                result.turnDirection = -1;
            }
            else if (strongRightSide && result.rightWhiteRatio > farRightRatio * 0.5f)
            {
                result.isTurn = true;
                result.turnDirection = 1;
            }
        }
    }

    cv::rectangle(display, cv::Rect(0, nearStart, leftBandEnd, nearHeight),
                  cv::Scalar(0, 0, 255), 1);
    cv::rectangle(display, cv::Rect(leftBandEnd, nearStart, centerWidth, nearHeight),
                  cv::Scalar(255, 0, 0), 1);
    cv::rectangle(display, cv::Rect(rightBandStart, nearStart, imgWidth - rightBandStart, nearHeight),
                  cv::Scalar(0, 0, 255), 1);

    cv::line(display, cv::Point(leftBandEnd, 0), cv::Point(leftBandEnd, imgHeight),
             cv::Scalar(100, 100, 100), 1);
    cv::line(display, cv::Point(rightBandStart, 0), cv::Point(rightBandStart, imgHeight),
             cv::Scalar(100, 100, 100), 1);
    cv::line(display, cv::Point(0, nearStart), cv::Point(imgWidth, nearStart),
             cv::Scalar(0, 255, 255), 1);
    cv::line(display, cv::Point(0, farStart), cv::Point(imgWidth, farStart),
             cv::Scalar(255, 255, 0), 1);
    cv::line(display, cv::Point(0, farEnd), cv::Point(imgWidth, farEnd),
             cv::Scalar(255, 255, 0), 1);

    if (result.pathFound)
    {
        int dotX = (int)(result.midpointX * (float)imgWidth);
        int dotY = nearStart + nearHeight / 2;
        cv::circle(display, cv::Point(dotX, dotY), 8, cv::Scalar(0, 255, 0), -1);

        cv::line(display, cv::Point(dotX, 0), cv::Point(dotX, imgHeight),
                 cv::Scalar(0, 255, 0), 2);

        int centerX = imgWidth / 2;
        cv::line(display, cv::Point(centerX, nearStart), cv::Point(dotX, dotY),
                 cv::Scalar(0, 255, 0), 2);
    }

    if (result.isTurn)
    {
        std::string turnText = (result.turnDirection < 0) ? "LEFT TURN" : "RIGHT TURN";
        cv::putText(display, turnText, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }

    cv::resize(display, result.displayImage, cv::Size(), 3.0, 3.0, cv::INTER_NEAREST);
    return result;
}
