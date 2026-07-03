// send_image_scp.cpp
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdlib>
#include <string>

using namespace cv;
using namespace std;

int main()
{
    // 1. 打开摄像头并拍照
    VideoCapture cap("/dev/video41", cv::CAP_V4L2);
    if (!cap.isOpened())
    {
        cerr << "Error: 无法打开摄像头！" << endl;
        return -1;
    }

    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);

    Mat frame;
    cap >> frame;
    cap.release();

    if (frame.empty())
    {
        cerr << "Error: 拍照失败！" << endl;
        return -1;
    }

    // 2. 保存为本地文件
    string filename = "test.jpg";
    imwrite(filename, frame);
    cout << "拍照成功: " << filename << endl;

    // 3. 使用SCP传输到AP端
    string scp_cmd = "scp " + filename + " root@192.168.0.250:/H300_UDP/camera/";
    
    cout << "正在传输到 AP: " << scp_cmd << endl;
    
    int result = system(scp_cmd.c_str());
    
    if (result == 0)
    {
        cout << "传输成功！图片已保存到 AP 的 /H300_UDP/camera/ 目录" << endl;
    }
    else
    {
        cerr << "传输失败！请检查网络和SSH配置" << endl;
        return -1;
    }

    return 0;
}