#include <iostream>
#include <iomanip>
#include <cmath>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include "../Drive_Json.hpp"
#include "ThreadsFlow/FlowController.hpp"
#include "yolov5/yolov5.hpp"
#include "Uart/uart.hpp"

class ChiliAPM
{

public:
    ChiliAPM() { RPiAPMInit(); }

    ~ChiliAPM();

    void RPiAPMInit();

    void Json_test();

    void Uartsend_test();

    void UartKey_test();

    void ChiliAPMStartUp();

    void YOLODetectTaskReg();

    void UartSendTaskReg();

    void TaskThreadPrint();

protected:
    struct TaskThread
    {
        const int _flag_Sys_CPU_Asign = 2;
        float _flag_UARTFlowFreq = 10.f;

        std::unique_ptr<FlowThread> UARTFlow;
        std::unique_ptr<FlowThread> YOLOFlow;
    } TF;

    struct DataProcess
    {
        int chili_x = 0;
        int chili_y = 0;
        int chili_z = 0;
        bool chili_cut = 0;
        char data_uart[6] = {01, 02, 0, 0, 0, 0};
    } DP;

private:
    double configSettle(const char *configDir, const char *Target);

    inline int GetTimestamp();

    struct DataConfig
    {
        int fd_uart;
        int uart_baud;
        char key;
        double totalRedArea = 0.0;
        double detected_area = 0;
        int detected_area_max = 20000;
        int json_test_data = 0;
    } DC;

};

ChiliAPM::~ChiliAPM()
{
    close(DC.fd_uart);
}

void ChiliAPM::RPiAPMInit()
{
    DC.uart_baud = 115200;

    DC.fd_uart = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (DC.fd_uart < 0)
    {
        std::cerr << "Error opening serial port" << std::endl;
        return;
    }
    set_serial(DC.fd_uart, DC.uart_baud, 8, 'N', 1);

}

void ChiliAPM::YOLODetectTaskReg()
{

    TF.YOLOFlow.reset(new FlowThread(
        [&]
        {


        },
        TF._flag_Sys_CPU_Asign, TF._flag_UARTFlowFreq));
    //     YOLOV5 yolo;
    // yolo_data data;
    // cv::VideoCapture capture(0);
    // cv::Mat mask1, mask2, mask, hsv;
    // if (!capture.isOpened())
    // {
    //     std::cout << "Error opening video stream or file" << std::endl;
    // }
    // // 获取视频流的帧率和帧大小
    // double fps = capture.get(cv::CAP_PROP_FPS);
    // cv::Size frameSize(capture.get(cv::CAP_PROP_FRAME_WIDTH), capture.get(cv::CAP_PROP_FRAME_HEIGHT));

    // // 初始化VideoWriter对象
    // cv::VideoWriter writer("output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, frameSize);

    // // 检查是否成功初始化
    // if (!writer.isOpened())
    // {
    //     std::cout << "Could not open the output video file for write\n";
    // }
//     while (true)
//     {

//         cv::Mat frame;
//         capture >> frame;
//         if (frame.empty())
//         {
//             std::cout << "Fail to read image from camera!" << std::endl;
//             break;
//         }
//         data = yolo.yolov5(frame);

//         double max_confidence = 0.0;
//         cv::Rect max_conf_rect;
//         for (auto &obj : data.detections)
//         {
//             if (obj.conf > max_confidence) // 如果当前对象的置信度大于最大置信度
//             {
//                 max_confidence = obj.conf;                 // 更新最大置信度
//                 max_conf_rect = get_rect(frame, obj.bbox); // 更新最大置信度的矩形框
//             }
//         }
//         if (max_confidence > 0.0)
//         {

//             cv::rectangle(frame, max_conf_rect, cv::Scalar(0, 255, 0), 2);
//             std::string label = my_classes[(int)data.detections[0].class_id];
//             char score_str[8];
//             sprintf(score_str, "%.2f", max_confidence);
//             std::string conf(score_str);
//             std::string label_with_conf = label + " " + conf;
//             cv::putText(frame, label_with_conf, cv::Point(max_conf_rect.x, max_conf_rect.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(255, 255, 255), 2);
//             if (max_conf_rect.x + max_conf_rect.width / 2 < frame.cols / 2) // 如果物体在图像中间的左边
//             {
//                 data_x--; // 计数器减一
//             }
//             else // 如果物体在图像中间的右边
//             {
//                 data_x++; // 计数器加一
//             }
//             if (max_conf_rect.y + max_conf_rect.height / 2 < frame.rows / 2) // 如果物体在图像中间的上面部分
//             {
//                 data_z++; // data_y增加
//             }
//             else // 如果物体在图像中间的下面部分
//             {
//                 data_z--; // data_y减少
//             }
//         }

//         cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
//         cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1);    // 红色的低范围阈值
//         cv::inRange(hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask2); // 红色的高范围阈值

//         // 使用bitwise_or合并两个掩码
//         cv::bitwise_or(mask1, mask2, mask);

//         // 接下来可以使用mask进行轮廓查找等操作
//         std::vector<std::vector<cv::Point>> contours;
//         cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

//         for (size_t i = 0; i < contours.size(); i++)
//         {
//             double area = cv::contourArea(contours[i]); // 计算每个轮廓的面积
//             totalRedArea += area;                       // 累加红色区域的面积
//         }
//         // 在原图上绘制轮廓
//         cv::drawContours(frame, contours, -1, cv::Scalar(0, 255, 0), 2); // 使用绿色绘制所有轮廓
//         cv::Rect image_top(0, 0, frame.cols, 100);                       // 定义图像最上方100单位的区域
//         cv::Rect image_bottom(0, frame.rows - 100, frame.cols, 100);     // 定义图像最下方100单位的区域
//         int mode2 = 0;                                                   // 初始化mode2
//         for (const auto &contour : contours)
//         {
//             if (cv::boundingRect(contour).br().y <= image_top.br().y || cv::boundingRect(contour).tl().y >= image_bottom.tl().y)
//             {
//                 mode2 = 1; // 如果红色区域在图像的最下方100单位或最上方100单位，则将mode2设为1
//                 break;     // 找到符合条件的红色区域后，退出循环
//             }
//             else
//             {
//                 mode2 = 0;
//                 break;
//             }
//         }

//         detected_area = max_conf_rect.width * max_conf_rect.height; // 计算识别框的面积

//         // std::cout << "Detected area: " << detected_area << std::endl; // 输出识别框的面积
//         // std::cout << "Total red area: " << totalRedArea << std::endl; // 输出红色区域的总面积
//         std::cout << "detected_area:" << detected_area << "\r\n";
//         std::cout << "totalRedArea2:" << totalRedArea << "\r\n";
//         std::cout << "mode2:" << mode2 << "\r\n";
//         // std::cout<<"cut:"<<cut<<"\r\n";
//         if (detected_area > 20000)
//         {
//             mode = 1;
//             detected_area_max = 200000;
//         }

//         if (mode == 1 & totalRedArea > 1100000 & mode2 == 1)
//         {
//             cut = 1;
//             data_y = 2;
//             mode = 0;
//             mode3 = 0;
//         }
//         else
//         {
//             cut = 0;
//         }

//         cv::imshow("yolov5", frame);
//         writer.write(frame);

//         key = cv::waitKey(1);
//         if (key == 'q')
//         {
//             int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
//             set_serial(fd, 115200, 8, 'N', 1);
//             char buff2[6] = {01, 02, 0, 0, 0, 0};
//             serial_write(fd, buff2, 6);

//             break;
//         }
//     }
//     capture.release();
}

void ChiliAPM::UartSendTaskReg()
{
    TF.UARTFlow.reset(new FlowThread(
        [&]
        {
            DP.data_uart[2] = DP.chili_x;
            DP.data_uart[3] = DP.chili_y;
            DP.data_uart[4] = DP.chili_z;
            DP.data_uart[5] = DP.chili_cut;
            serial_write(DC.fd_uart, DP.data_uart, sizeof(data));

        },
        TF._flag_Sys_CPU_Asign, TF._flag_UARTFlowFreq));
}


void ChiliAPM::ChiliAPMStartUp()
{
    YOLODetectTaskReg();

    UartSendTaskReg();
}

void ChiliAPM::Json_test()
{   
    DC.json_test_data = configSettle("../Chili.json", "json_test_data");
    std::cout << "DC.json_test_data: " << DC.json_test_data << "\r\n";
}

void ChiliAPM::Uartsend_test()
{
    serial_write(DC.uart_baud, DP.data_uart, 6);
}

void ChiliAPM::UartKey_test()
{
    while (true)
    {
        std::cin >> DC.key;
        if (DC.key == 'w')
        {

            DP.data_uart[2] += 10; // 按下w时，第三个数据加10
            std::cout << "buff2[2]" << int(DP.data_uart[2]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); // 发送数据
        }
        else if (DC.key == 'e')
        {
            DP.data_uart[2] -= 10;
            std::cout << "buff2[2]" << int(DP.data_uart[2]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); // 发送数据
        }
        else if (DC.key == 'r')
        {
            DP.data_uart[4] -= 5;
            std::cout << "buff2[3]" << int(DP.data_uart[4]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); // 发送数据
        }
        else if (DC.key == 't')
        {
            DP.data_uart[4] += 5;
            std::cout << "buff2[3]" << int(DP.data_uart[4]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); // 发送数据
        }
        else if (DC.key == 'a')
        {
            DP.data_uart[5] = 0;
            std::cout << "buff2[5]" << int(DP.data_uart[5]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); // 发送数据
        }
        else if (DC.key == 's')
        {
            DP.data_uart[5] = 1;
            std::cout << "buff2[5]" << int(DP.data_uart[5]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); // 发送数据
        }
        else if (DC.key == 'z')
        {
            DP.data_uart[3] = 0;
            std::cout << "buff2[3]" << int(DP.data_uart[3]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); // 发送数据
        }
        else if (DC.key == 'x')
        {
            DP.data_uart[3] = 1;
            std::cout << "buff2[3]" << int(DP.data_uart[3]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); // 发送数据
        }
        else if (DC.key == 'c')
        {
            DP.data_uart[3] = 2;
            std::cout << "buff2[3]" << int(DP.data_uart[3]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); // 发送数据
        }
    }
}

double ChiliAPM::configSettle(const char *configDir, const char *Target)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Configdata = nlohmann::json::parse(content);
    return Configdata[Target].get<double>();
}

inline int ChiliAPM::GetTimestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * (uint64_t)1000000 + tv.tv_usec;
}

void ChiliAPM::TaskThreadPrint()
{
    while (true)
    {
        // std::cout << "666"
        //           << "\r\n";
        usleep(100000);
    }
}