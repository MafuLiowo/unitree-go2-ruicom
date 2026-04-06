#include <unitree/robot/go2/video/video_client.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstring>

int main(int argc, char** argv)
{
    std::string netInterface;
    if (argc > 1) {
        netInterface = argv[1];
    }

    if (!netInterface.empty()) {
        unitree::robot::ChannelFactory::Instance()->Init(0, netInterface);
    } else {
        unitree::robot::ChannelFactory::Instance()->Init(0);
    }

    unitree::robot::go2::VideoClient video_client;
    video_client.SetTimeout(1.0f);
    video_client.Init();

    std::vector<uint8_t> image_sample;
    int ret;

    cv::namedWindow("Go2 Real-time Video", cv::WINDOW_AUTOSIZE);

    while (true)
    {
        ret = video_client.GetImageSample(image_sample);

        if (ret == 0 && !image_sample.empty()) {
            cv::Mat rawData(image_sample);
            cv::Mat frame = cv::imdecode(rawData, cv::IMREAD_COLOR);

            if (!frame.empty()) {
                cv::imshow("Go2 Real-time Video", frame);
            }
        }

        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}