/**
 * @file go2_process_images.cpp
 * @brief Go2 机器人图像处理模块，将摄像头画面转换为高对比度图像并分析路径特征
 *
 * @par 功能定位
 *       本模块为库文件，提供 toHighContrast() 和 analyzePath() 两个核心函数，
 *       供 go2_seeking_way 和 go2_process_images_main 等可执行程序调用。
 *       - toHighContrast(): 将彩色图像转为高对比度二值图，提取白色路径区域
 *       - analyzePath(): 分析二值图中的路径中点位置、转弯检测等特征
 */
#include "ImageProcessor.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>

/**
 * @brief 将彩色帧转换为高对比度二值图像，提取白色可行路径区域
 * @param frame 输入的彩色图像（BGR格式）
 * @param thresholdValue 二值化阈值，像素灰度值高于此值的设为白色(255)，低于的设为黑色(0)
 * @return cv::Mat 高对比度二值图像
 */
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

/**
 * @brief 分析二值化图像中的路径特征，包括路径中点位置、白色比例、转弯检测
 * @param binary 输入的二值化图像（白色=可行路径，黑色=障碍/不可行区域）
 * @return PathAnalysis 包含路径分析结果的结构体
 */
PathAnalysis analyzePath(const cv::Mat& binary)
{
    PathAnalysis result;
    if (binary.empty()) return result;

    // 将灰度二值图转为三通道彩色图，用于后续绘制可视化标记
    cv::Mat display;
    cv::cvtColor(binary, display, cv::COLOR_GRAY2BGR);

    int imgWidth = binary.cols;
    int imgHeight = binary.rows;

    // 定义近端分析区域：图像下方 60%~95% 区域，即靠近机器人的地面区域
    int nearStart = imgHeight * 60 / 100;
    int nearEnd   = imgHeight * 95 / 100;
    int nearHeight = nearEnd - nearStart;
    cv::Rect nearRoi(0, nearStart, imgWidth, nearHeight);
    cv::Mat nearZone = binary(nearRoi);

    // 定义远端分析区域：图像上方 20%~55% 区域，即远离机器人的前方区域
    int farStart = imgHeight * 20 / 100;
    int farEnd   = imgHeight * 55 / 100;
    int farHeight = farEnd - farStart;
    cv::Rect farRoi(0, farStart, imgWidth, farHeight);
    cv::Mat farZone = binary(farRoi);

    // 将图像水平方向三等分：左侧区域、中间区域、右侧区域
    int leftBandEnd   = imgWidth / 3;
    int rightBandStart = imgWidth * 2 / 3;
    int centerWidth   = rightBandStart - leftBandEnd;

    // 在近端区域内划分左、中、右三个子区域
    cv::Rect leftRect(0, 0, leftBandEnd, nearHeight);
    cv::Rect centerRect(leftBandEnd, 0, centerWidth, nearHeight);
    cv::Rect rightRect(rightBandStart, 0, imgWidth - rightBandStart, nearHeight);

    // 统计近端各区域的白色像素数量
    int leftPixels   = cv::countNonZero(nearZone(leftRect));
    int centerPixels = cv::countNonZero(nearZone(centerRect));
    int rightPixels  = cv::countNonZero(nearZone(rightRect));

    // 计算各区域的理论最大像素数（即总面积），用于归一化
    float totalNearPixels = (float)nearHeight * imgWidth;
    float totalLeft   = (float)nearHeight * leftBandEnd;
    float totalCenter = (float)nearHeight * centerWidth;
    float totalRight  = (float)nearHeight * (imgWidth - rightBandStart);

    // 计算近端各区域的白色占比
    result.leftWhiteRatio   = (totalLeft > 0)   ? leftPixels / totalLeft : 0.0f;
    result.centerWhiteRatio = (totalCenter > 0) ? centerPixels / totalCenter : 0.0f;
    result.rightWhiteRatio  = (totalRight > 0)  ? rightPixels / totalRight : 0.0f;

    float totalWhite = (float)(leftPixels + centerPixels + rightPixels);
    result.whiteRatio = (totalNearPixels > 0) ? totalWhite / totalNearPixels : 0.0f;

    // 白色像素过少时认为没有检测到路径
    if (totalWhite < 15)
    {
        result.pathFound = false;
        result.midpointX = -1.0f;
    }
    else
    {
        result.pathFound = true;

        // 加权平均法计算路径的水平中点位置（0.0=最左，1.0=最右）
        float weightedSum = 0.0f;
        for (int col = 0; col < imgWidth; col++)
        {
            int whiteCount = cv::countNonZero(
                nearZone(cv::Rect(col, 0, 1, nearHeight)));
            weightedSum += (float)whiteCount * ((float)col / (float)imgWidth);
        }
        result.midpointX = weightedSum / totalWhite;

        // 在远端区域内划分左、中、右三个子区域
        cv::Rect farLeftRect(0, 0, leftBandEnd, farHeight);
        cv::Rect farCenterRect(leftBandEnd, 0, centerWidth, farHeight);
        cv::Rect farRightRect(rightBandStart, 0, imgWidth - rightBandStart, farHeight);

        int farLeftPixels   = cv::countNonZero(farZone(farLeftRect));
        int farCenterPixels = cv::countNonZero(farZone(farCenterRect));
        int farRightPixels  = cv::countNonZero(farZone(farRightRect));

        float farLeftRatio   = (totalLeft > 0)   ? farLeftPixels / totalLeft : 0.0f;
        float farCenterRatio = (totalCenter > 0) ? farCenterPixels / totalCenter : 0.0f;
        float farRightRatio  = (totalRight > 0)  ? farRightPixels / totalRight : 0.0f;

        // 转弯检测条件：路径中点偏离中心、中间区域空旷、且某一侧有大量白色像素
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
                result.turnDirection = -1;  // 左转弯
            }
            else if (strongRightSide && result.rightWhiteRatio > farRightRatio * 0.5f)
            {
                result.isTurn = true;
                result.turnDirection = 1;   // 右转弯
            }
        }
    }

    // 以下为可视化绘制：用矩形框标出近端左/中/右区域
    cv::rectangle(display, cv::Rect(0, nearStart, leftBandEnd, nearHeight),
                  cv::Scalar(0, 0, 255), 1);
    cv::rectangle(display, cv::Rect(leftBandEnd, nearStart, centerWidth, nearHeight),
                  cv::Scalar(255, 0, 0), 1);
    cv::rectangle(display, cv::Rect(rightBandStart, nearStart, imgWidth - rightBandStart, nearHeight),
                  cv::Scalar(0, 0, 255), 1);

    // 绘制区域分隔线及近端/远端边界线
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
        // 在路径中点位置绘制绿色圆点及竖直指示线
        int dotX = (int)(result.midpointX * (float)imgWidth);
        int dotY = nearStart + nearHeight / 2;
        cv::circle(display, cv::Point(dotX, dotY), 8, cv::Scalar(0, 255, 0), -1);

        cv::line(display, cv::Point(dotX, 0), cv::Point(dotX, imgHeight),
                 cv::Scalar(0, 255, 0), 2);

        // 绘制图像中心到路径中点的连线，表示偏差方向
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

    // 放大 3 倍以方便观察（使用最近邻插值保持像素清晰）
    cv::resize(display, result.displayImage, cv::Size(), 3.0, 3.0, cv::INTER_NEAREST);
    return result;
}
