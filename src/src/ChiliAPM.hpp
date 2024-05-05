#include <iostream>
#include <iomanip>
#include <cmath>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <signal.h>
#include <sys/time.h>
#include <chrono>
#include "../Drive_Json.hpp"
#include "ThreadsFlow/FlowController.hpp"
#include "yolov5/yolov5.hpp"
#include "Uart/uart.hpp"
#include "MTF-02/MTF-02.hpp"

inline volatile sig_atomic_t SystemSignal;
std::chrono::steady_clock::time_point lastCutTime;
std::chrono::steady_clock::time_point delayStartTime;

class ChiliAPM
{

public:
    ChiliAPM() { RPiAPMInit(); }

    ~ChiliAPM();

    void RPiAPMInit();

    void Json_test();

    void Uartsend_test();

    void UartKey_test();

    void MTF02_test();

    void ChiliAPMStartUp();

    void YOLODetectTaskReg();

    void UartSendTaskReg();

    void DistanceReadTaskReg();

    void TaskThreadPrint();

protected:
    struct TaskThread
    {
        const int _flag_Sys_CPU_Asign = 2;
        float _flag_UARTFlowFreq = 10.f;
        float _flag_MTF02FlowFreq = 1000.f;
        float _flag_YOLOFlowFreq = 100.f;

        bool _flag_Print_Task_Running = false;

        std::unique_ptr<FlowThread> UARTFlow;
        std::unique_ptr<FlowThread> YOLOFlow;
        std::unique_ptr<FlowThread> MTF02Flow;
    } TF;

    struct DataProcess
    {
        int chili_x = 0;
        int chili_y = 0;
        int chili_z = 0;
        bool chili_cut = 0;
        char data_uart[6] = {01, 02, 0, 0, 0, 0};
        int chili_x_compensation = 0;
        int chili_y_compensation = 0;
        int chili_z_compensation = 0;
        double distance = 0;
    } DP;

private:
    double configSettle(const char *configDir, const char *Target);

    std::string configSettleStr(const char *configDir, const char *Target);

    inline int GetTimestamp();

    void saftycheck();
    struct DataConfig
    {
        int fd_uart;
        int uart_baud = 9600;
        char key;
        double totalRedArea = 0.0;
        double detected_area = 0;
        int detected_area_max = 20000;
        int json_test_data = 0;
        double max_confidence = 0.0;
        cv::Rect max_conf_rect;
        std::string __UartDevice;
        std::string __MTF02Device;
        int Distance_Target = 40;
        double totalRedArea_Target = 1190270;
        int Distance_Back = 10;
    } DC;

    char score_str[8];
    std::string label;
    std::string label_with_conf;
    cv::Mat mask1, mask2, mask, hsv;
    double fps;
    cv::VideoCapture capture;
    int capture_id;
    MTF02 *mtf02Test;
    MTF02::MTF02Data MTF02_Data;
    int cut_mode = 0;
};

ChiliAPM::~ChiliAPM()
{
    close(DC.fd_uart);
}

void ChiliAPM::RPiAPMInit()
{
    DC.uart_baud = configSettle("../Chili.json", "uart_baud");
    DC.__UartDevice = configSettleStr("../Chili.json", "uard_device");
    DC.__MTF02Device = configSettleStr("../Chili.json", "mtf02_device");
    DP.chili_x_compensation = configSettle("../Chili.json", "chili_x_compensation");
    DP.chili_y_compensation = configSettle("../Chili.json", "chili_y_compensation");
    DP.chili_z_compensation = configSettle("../Chili.json", "chili_z_compensation");
    capture_id = configSettle("../Chili.json", "capture_id");
    DC.detected_area_max = configSettle("../Chili.json", "detected_area_max");
    DC.Distance_Target = configSettle("../Chili.json", "Distance_Target");
    DC.totalRedArea_Target = configSettle("../Chili.json", "totalRedArea_Target");
    DC.Distance_Back = configSettle("../Chili.json", "Distance_Back");

    DC.fd_uart = open(DC.__UartDevice.c_str(), O_RDWR | O_NOCTTY);
    if (DC.fd_uart < 0)
    {
        std::cerr << "Error opening serial port" << std::endl;
        return;
    }
    set_serial(DC.fd_uart, DC.uart_baud, 8, 'N', 1);
    usleep(500);
    mtf02Test = new MTF02(DC.__MTF02Device.c_str(), 0x31);
    usleep(500);
}

void ChiliAPM::DistanceReadTaskReg()
{
    TF.MTF02Flow.reset(new FlowThread(
        [&]
        {
            MTF02_Data = mtf02Test->MTF02DataGet();
        },
        TF._flag_Sys_CPU_Asign, TF._flag_MTF02FlowFreq));
}

void ChiliAPM::YOLODetectTaskReg()
{

    TF.YOLOFlow.reset(new FlowThread(
        [&]
        {
            YOLOV5 yolo;
            yolo_data data;
            capture.open(capture_id);
            if (!capture.isOpened())
            {
                std::cout << "Error opening video stream or file" << std::endl;
            }
            // 获取视频流的帧率和帧大小
            fps = capture.get(cv::CAP_PROP_FPS);
            cv::Size frameSize(capture.get(cv::CAP_PROP_FRAME_WIDTH), capture.get(cv::CAP_PROP_FRAME_HEIGHT));

            // 初始化VideoWriter对象
            cv::VideoWriter writer("output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, frameSize);
            // 检查是否成功初始化
            if (!writer.isOpened())
            {
                std::cout << "Could not open the output video file for write\n";
            }
            DP.chili_y = configSettle("../Chili.json", "chili_y");
            while (true)
            {
                cv::Mat frame;
                capture >> frame;
                if (frame.empty())
                {
                    std::cout << "Fail to read image from camera!" << std::endl;
                    break;
                }

                data = yolo.yolov5(frame);

                // 在处理每一帧之前重置最大置信度和矩形框
                DC.max_confidence = 0.0;       // 重置最大置信度
                DC.max_conf_rect = cv::Rect(); // 重置矩形框
                for (auto &obj : data.detections)
                {
                    if (obj.conf > DC.max_confidence) // 如果当前对象的置信度大于最大置信度
                    {
                        DC.max_confidence = obj.conf;                 // 更新最大置信度
                        DC.max_conf_rect = get_rect(frame, obj.bbox); // 更新最大置信度的矩形框
                    }
                }
                // 用于计算物体的偏移量
                if (DC.max_confidence > 0.0)
                {

                    cv::rectangle(frame, DC.max_conf_rect, cv::Scalar(0, 255, 0), 2);
                    label = my_classes[(int)data.detections[0].class_id];
                    sprintf(score_str, "%.2f", DC.max_confidence);
                    std::string conf(score_str);
                    label_with_conf = label + " " + conf;
                    cv::putText(frame, label_with_conf, cv::Point(DC.max_conf_rect.x, DC.max_conf_rect.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(255, 255, 255), 2);
                    if (DC.max_conf_rect.x + DC.max_conf_rect.width / 2 < frame.cols / 2) // 如果物体在图像中间的左边
                    {
                        DP.chili_x--; // 计数器减一
                    }
                    else // 如果物体在图像中间的右边
                    {
                        DP.chili_x++; // 计数器加一
                    }
                    if (DC.max_conf_rect.y + DC.max_conf_rect.height / 2 < 2 * frame.rows / 3) // 如果物体在图像中间的上面部分
                    {
                        DP.chili_z++; // data_y增加
                    }
                    else // 如果物体在图像中间的下面部分
                    {
                        DP.chili_z--; // data_y减少
                    }
                }
                // 用于计算红色的面积
                cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
                cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1);    // 红色的低范围阈值
                cv::inRange(hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask2); // 红色的高范围阈值
                // 使用bitwise_or合并两个掩码
                cv::bitwise_or(mask1, mask2, mask);

                // 接下来可以使用mask进行轮廓查找等操作
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                DC.totalRedArea = 0;
                for (size_t i = 0; i < contours.size(); i++)
                {
                    double area = cv::contourArea(contours[i]); // 计算每个轮廓的面积
                    DC.totalRedArea += area;                    // 累加红色区域的面积
                }
                // 在原图上绘制轮廓
                cv::drawContours(frame, contours, -1, cv::Scalar(0, 255, 0), 2); // 使用绿色绘制所有轮廓

                DC.detected_area = DC.max_conf_rect.width * DC.max_conf_rect.height; // 计算识别框的面积
                if (MTF02_Data.Distance<DC.Distance_Target & DC.totalRedArea> DC.totalRedArea_Target & MTF02_Data.Distance != 0)
                {
                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::seconds>(now - lastCutTime).count() > 10)
                    {
                        cut_mode = 1;
                        lastCutTime = now; // 更新时间戳
                    }
                    else
                    {
                        cut_mode = 0;
                    }
                }
                else if (MTF02_Data.Distance < DC.Distance_Back)
                {
                    DP.chili_y = 3;
                }
                else
                {
                    cut_mode = 0;
                }

                cv::imshow("yolov5", frame);
                writer.write(frame);

                DC.key = cv::waitKey(1);
                if (DC.key == 'q')
                {
                    capture.release();
                    break;
                }
                if (SystemSignal == SIGTERM || SystemSignal == SIGINT)
                {
                    capture.release();
                    break;
                }
            }
        },
        TF._flag_Sys_CPU_Asign, TF._flag_YOLOFlowFreq)); // 135156
}

void ChiliAPM::UartSendTaskReg()
{
    TF.UARTFlow.reset(new FlowThread(
        [&]
        {
            DP.data_uart[2] = (DP.chili_x + DP.chili_x_compensation < -127) ? -127 : (DP.chili_x + DP.chili_x_compensation > 127 ? 127 : DP.chili_x + DP.chili_x_compensation);
            DP.data_uart[3] = (DP.chili_y + DP.chili_y_compensation < -127) ? -127 : (DP.chili_y + DP.chili_y_compensation > 127 ? 127 : DP.chili_y + DP.chili_y_compensation);
            DP.data_uart[4] = (DP.chili_z + DP.chili_z_compensation < -127) ? -127 : (DP.chili_z + DP.chili_z_compensation > 127 ? 127 : DP.chili_z + DP.chili_z_compensation);
            if (cut_mode)
            {
                DP.chili_cut = 1;
            }
            else
            {
                DP.chili_cut = 0;
            }
            DP.data_uart[5] = DP.chili_cut;
            serial_write(DC.fd_uart, DP.data_uart, sizeof(data));
        },
        TF._flag_Sys_CPU_Asign, TF._flag_UARTFlowFreq)); // 354696
}

void ChiliAPM::ChiliAPMStartUp()
{
    YOLODetectTaskReg();

    DistanceReadTaskReg();

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

void ChiliAPM::MTF02_test()
{
    while (true)
    {
        MTF02_Data = mtf02Test->MTF02DataGet();
        std::cout << " Speed_X " << MTF02_Data.Speed_X << "\r\n";
        std::cout << " Speed_Y " << MTF02_Data.Speed_Y << "\r\n";
        std::cout << " Speed_X " << MTF02_Data.Pos_X << "\r\n";
        std::cout << " Speed_Y " << MTF02_Data.Pos_Y << "\r\n";
        std::cout << " Distance " << MTF02_Data.Distance << "\r\n";
        std::cout << "\033[5A";
        std::cout << "\033[K";
        usleep(5000);
    }
}

void ChiliAPM::UartKey_test()
{
    while (true)
    {
        std::cin >> DC.key;
        if (DC.key == 'w')
        {

            DP.data_uart[2] += 10; // 右转
            std::cout << "buff2[2]" << int(DP.data_uart[2]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6);
        }
        else if (DC.key == 'e')
        {
            DP.data_uart[2] -= 10; // 左转
            std::cout << "buff2[2]" << int(DP.data_uart[2]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6);
        }
        else if (DC.key == 'r')
        {
            DP.data_uart[4] -= 5;
            std::cout << "buff2[4]" << int(DP.data_uart[4]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); // 降低
        }
        else if (DC.key == 't')
        {
            DP.data_uart[4] += 5;
            std::cout << "buff2[4]" << int(DP.data_uart[4]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); // 升高
        }
        else if (DC.key == 'a')
        {
            DP.data_uart[5] = 0;
            std::cout << "buff2[5]" << int(DP.data_uart[5]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); // 剪
        }
        else if (DC.key == 's')
        {
            DP.data_uart[5] = 1;
            std::cout << "buff2[5]" << int(DP.data_uart[5]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); // 开
        }
        else if (DC.key == 'z')
        {
            DP.data_uart[3] = 0;
            std::cout << "buff2[3]" << int(DP.data_uart[3]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); // 不动
        }
        else if (DC.key == 'x')
        {
            DP.data_uart[3] = 1;
            std::cout << "buff2[3]" << int(DP.data_uart[3]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); //   前进
        }
        else if (DC.key == 'c')
        {
            DP.data_uart[3] = 2;
            std::cout << "buff2[3]" << int(DP.data_uart[3]) << "\r\n";
            serial_write(DC.fd_uart, DP.data_uart, 6); // 后退
        }
    }
}

void ChiliAPM::saftycheck()
{
    if (SystemSignal == SIGTERM || SystemSignal == SIGINT)
    {
        TF._flag_Print_Task_Running = false;
        TF.UARTFlow->FlowStopAndWait();
        TF.YOLOFlow->FlowStopAndWait();
        usleep(10000);
        {
            int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
            set_serial(fd, 115200, 8, 'N', 1);
            char buff2[6] = {01, 02, 1, 0, 1, 0};
            serial_write(fd, buff2, 6);
        }
        TF.UARTFlow.reset();
        TF.YOLOFlow.reset();
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

std::string ChiliAPM::configSettleStr(const char *configDir, const char *Target)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Configdata = nlohmann::json::parse(content);
    return Configdata[Target].get<std::string>();
}

inline int ChiliAPM::GetTimestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * (uint64_t)1000000 + tv.tv_usec;
}

void ChiliAPM::TaskThreadPrint()
{
    TF._flag_Print_Task_Running = true;
    while (TF._flag_Print_Task_Running)
    {
        saftycheck();
        std::cout << "\033[100A";
        std::cout << "\033[K";
        std::cout << std::setw(7) << std::setfill(' ') << " Distance " << MTF02_Data.Distance << "\r\n";
        std::cout << std::setw(7) << std::setfill(' ') << " DC.totalRedArea " << DC.totalRedArea << "\r\n";
        std::cout << std::setw(7) << std::setfill(' ') << " DP.chili_cut " << DP.chili_cut << "\r\n";
        std::cout << std::setw(7) << std::setfill(' ') << " DC.Distance_Target " << DC.Distance_Target << "\r\n";
        std::cout << std::setw(7) << std::setfill(' ') << " DC.totalRedArea_Target " << DC.totalRedArea_Target << "\r\n";
        std::cout << std::setw(7) << std::setfill(' ') << " DC.Distance_Back " << DC.Distance_Back << "\r\n";
        std::cout << "chili_z: " << DP.chili_z << "\r\n";
        std::cout << "chili_x: " << DP.chili_x << "\r\n";
        usleep(10000);
    }
}
