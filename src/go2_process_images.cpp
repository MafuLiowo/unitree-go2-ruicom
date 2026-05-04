/**
 * @file go2_process_images.cpp
 * @brief Go2 机器人图像处理模块，将摄像头画面转换为高对比度图像并分析路径特征
 *
 * @par 功能定位
 *       本模块为库文件，提供 toHighContrast() 和 analyzePath() 两个核心函数，
 *       供 go2_seeking_way 和 go2_process_images_main 等可执行程序调用。
 *       - toHighContrast(): 将彩色图像转为高对比度二值图，提取白色路径区域
 *       - analyzePath(): 使用连通域分析，在白色二值图中选取最靠近图像中央的
 *         连续白色区域，以其质心横坐标作为路径中点，并进行转弯检测
 */
#include "ImageProcessor.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <cfloat>

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
 * @brief 使用连通域分析二值化图像中的路径特征
 *
 * @par 算法说明
 *       在近端区域内进行连通域分析，找出所有连续白色区域（连通域），
 *       选取其中水平位置最靠近图像中央的连通域，以其质心横坐标作为路径中点。
 *       该算法可有效避开图像左右两侧的离散噪点，聚焦于中央区域的实际路径。
 *
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

    // 记录选中连通域的边界框，用于可视化
    int selectedBlobLeft = 0, selectedBlobTop = 0;
    int selectedBlobWidth = 0, selectedBlobHeight = 0;

    // 白色像素过少时认为没有检测到路径
    if (totalWhite < 15)
    {
        result.pathFound = false;
        result.midpointX = -1.0f;
    }
    else
    {
        // 连通域分析：在近端区域内找出所有连续白色区域
        // 从中选取水平位置最靠近图像中央的连通域，以其质心横坐标作为路径中点
        cv::Mat labels, stats, centroids;
        int nComponents = cv::connectedComponentsWithStats(
            nearZone, labels, stats, centroids, 8, CV_32S);

        int bestComponent = -1;
        float bestCenterX = 0.5f;
        float bestDist = FLT_MAX;
        int bestArea = 0;
        const int minBlobArea = 15;  // 最小连通域面积，过滤离散噪点

        for (int i = 1; i < nComponents; i++)
        {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < minBlobArea) continue;

            double cx = centroids.at<double>(i, 0);
            float normalizedCx = (float)cx / (float)imgWidth;
            float dist = std::abs(normalizedCx - 0.5f);

            // 优先选靠近中央的连通域；距离相同时选面积更大者
            if (dist < bestDist || (dist == bestDist && area > bestArea))
            {
                bestDist = dist;
                bestComponent = i;
                bestCenterX = normalizedCx;
                bestArea = area;
            }
        }

        if (bestComponent >= 0)
        {
            result.pathFound = true;
            result.midpointX = bestCenterX;

            // 记录选中连通域的边界框（在 nearZone 坐标系中）
            selectedBlobLeft   = stats.at<int>(bestComponent, cv::CC_STAT_LEFT);
            selectedBlobTop    = stats.at<int>(bestComponent, cv::CC_STAT_TOP);
            selectedBlobWidth  = stats.at<int>(bestComponent, cv::CC_STAT_WIDTH);
            selectedBlobHeight = stats.at<int>(bestComponent, cv::CC_STAT_HEIGHT);
        }
        else
        {
            result.pathFound = false;
            result.midpointX = -1.0f;
        }

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
        if (result.pathFound)
        {
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

        // 绘制选中连通域的边界框（青色），将 nearZone 坐标平移回全图坐标
        if (selectedBlobWidth > 0 && selectedBlobHeight > 0)
        {
            cv::rectangle(display,
                cv::Rect(selectedBlobLeft, selectedBlobTop + nearStart,
                         selectedBlobWidth, selectedBlobHeight),
                cv::Scalar(255, 255, 0), 2);
        }
    }

    if (result.isTurn)
    {
        std::string turnText = (result.turnDirection < 0) ? "LEFT TURN" : "RIGHT TURN";
        cv::putText(display, turnText, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }

    // 保持原始尺寸（使用最近邻插值保持像素清晰）
    cv::resize(display, result.displayImage, cv::Size(), 1.0, 1.0, cv::INTER_NEAREST);
    return result;
}
