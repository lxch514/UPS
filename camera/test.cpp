#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
    // 打开USB摄像头
    VideoCapture cap("/dev/video41", cv::CAP_V4L2);

    if (!cap.isOpened())
    {
        cerr << "Error: 无法打开摄像头！" << endl;
        return -1;
    }

    // 设置分辨率
    cap.set(CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(CAP_PROP_FRAME_HEIGHT, 1080);

    Mat frame;

    // 读取一帧
    cap >> frame;

    if (frame.empty())
    {
        cerr << "Error: 未获取到图像！" << endl;
        return -1;
    }

    // 保存图片
    if (imwrite("test.jpg", frame))
    {
        cout << "图片保存成功：test.jpg" << endl;
    }
    else
    {
        cout << "图片保存失败！" << endl;
    }

    cap.release();

    return 0;
}