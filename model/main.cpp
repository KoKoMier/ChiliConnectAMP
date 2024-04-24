#include <iostream>
#include "src/yolov5/yolov5.hpp"
#include "src/Uart/uart.hpp"
#include <iomanip>
#include <cmath>
#include <unistd.h>
#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
    int argvs;

    while ((argvs = getopt(argc, argv, "a::s::g::m::c::f::R")) != -1)
    {
        switch (argvs)
        {
        case 'a':
        {
            int mode3 = 1;
            YOLOV5 yolo;
            yolo_data data;
            cv::VideoCapture capture(0);
            cv::Mat mask1, mask2, mask, hsv;
            if (!capture.isOpened())
            {
                std::cout << "Error opening video stream or file" << std::endl;
                return -1;
            }
            // 获取视频流的帧率和帧大小
            double fps = capture.get(cv::CAP_PROP_FPS);
            cv::Size frameSize(capture.get(cv::CAP_PROP_FRAME_WIDTH), capture.get(cv::CAP_PROP_FRAME_HEIGHT));

            // 初始化VideoWriter对象
            cv::VideoWriter writer("output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, frameSize);

            // 检查是否成功初始化
            if (!writer.isOpened())
            {
                std::cout << "Could not open the output video file for write\n";
                return -1;
            }
            int key;
            int data_x = 0;
            int data_y = 2;
            int data_z = 0;
            bool cut = 0;
            int mode = 0;
            char data_uart[6] = {01, 02, 0, 0, 0, 0};
            double totalRedArea = 0.0;
            double detected_area = 0;
            int detected_area_max = 20000;
            std::thread serialThread([&]()
                                     {
                            
                                          int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
                                          if (fd < 0)
                                          {
                                              std::cerr << "Error opening serial port" << std::endl;
                                              return;
                                          }
                                          set_serial(fd, 115200, 8, 'N', 1);

                                          while (true)
                                          {
                                            data_uart[2] = data_x;
                                            data_uart[3] = data_y;
                                            data_uart[4] = data_z+5;
                                            data_uart[5] = cut;
                                            // std::cout<<"data_x:"<<data_x<<"\r\n";
                                            // std::cout<<"data_y:"<<data_y<<"\r\n";
                                            // std::cout<<"data_z:"<<data_z<<"\r\n";
                                            // std::cout<<"cut:"<<cut<<"\r\n";
                                            serial_write(fd, data_uart, sizeof(data));
                                            if(cut==1)
                                            {
                                                cut == 0;
                                            }
                                            std::this_thread::sleep_for(std::chrono::milliseconds(100));

                                          }

                                          close(fd); });

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

                double max_confidence = 0.0;
                cv::Rect max_conf_rect;
                for (auto &obj : data.detections)
                {
                    if (obj.conf > max_confidence) // 如果当前对象的置信度大于最大置信度
                    {
                        max_confidence = obj.conf;                 // 更新最大置信度
                        max_conf_rect = get_rect(frame, obj.bbox); // 更新最大置信度的矩形框
                    }
                }
                if (max_confidence > 0.0)
                {

                    cv::rectangle(frame, max_conf_rect, cv::Scalar(0, 255, 0), 2);
                    std::string label = my_classes[(int)data.detections[0].class_id];
                    char score_str[8];
                    sprintf(score_str, "%.2f", max_confidence);
                    std::string conf(score_str);
                    std::string label_with_conf = label + " " + conf;
                    cv::putText(frame, label_with_conf, cv::Point(max_conf_rect.x, max_conf_rect.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(255, 255, 255), 2);
                    if (max_conf_rect.x + max_conf_rect.width / 2 < frame.cols / 2) // 如果物体在图像中间的左边
                    {
                        data_x--; // 计数器减一
                    }
                    else // 如果物体在图像中间的右边
                    {
                        data_x++; // 计数器加一
                    }
                    if (max_conf_rect.y + max_conf_rect.height / 2 < frame.rows / 2) // 如果物体在图像中间的上面部分
                    {
                        data_z++; // data_y增加
                    }
                    else // 如果物体在图像中间的下面部分
                    {
                        data_z--; // data_y减少
                    }
                }

                cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
                cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1);    // 红色的低范围阈值
                cv::inRange(hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask2); // 红色的高范围阈值

                // 使用bitwise_or合并两个掩码
                cv::bitwise_or(mask1, mask2, mask);

                // 接下来可以使用mask进行轮廓查找等操作
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                for (size_t i = 0; i < contours.size(); i++)
                {
                    double area = cv::contourArea(contours[i]); // 计算每个轮廓的面积
                    totalRedArea += area;                       // 累加红色区域的面积
                }
                // 在原图上绘制轮廓
                cv::drawContours(frame, contours, -1, cv::Scalar(0, 255, 0), 2); // 使用绿色绘制所有轮廓
                cv::Rect image_top(0, 0, frame.cols, 100);                       // 定义图像最上方100单位的区域
                cv::Rect image_bottom(0, frame.rows - 100, frame.cols, 100);     // 定义图像最下方100单位的区域
                int mode2 = 0;                                                   // 初始化mode2
                for (const auto &contour : contours)
                {
                    if (cv::boundingRect(contour).br().y <= image_top.br().y || cv::boundingRect(contour).tl().y >= image_bottom.tl().y)
                    {
                        mode2 = 1; // 如果红色区域在图像的最下方100单位或最上方100单位，则将mode2设为1
                        break;     // 找到符合条件的红色区域后，退出循环
                    }
                    else
                    {
                        mode2 = 0;
                        break; 
                    }
                }

                detected_area = max_conf_rect.width * max_conf_rect.height; // 计算识别框的面积

                // std::cout << "Detected area: " << detected_area << std::endl; // 输出识别框的面积
                // std::cout << "Total red area: " << totalRedArea << std::endl; // 输出红色区域的总面积
                std::cout << "detected_area:" << detected_area << "\r\n";
                std::cout << "totalRedArea2:" << totalRedArea << "\r\n";
                std::cout << "mode2:" << mode2 << "\r\n";
                // std::cout<<"cut:"<<cut<<"\r\n";
                if (detected_area > 20000)
                {
                    mode = 1;
                    detected_area_max = 200000;
                }

                if (mode == 1 & totalRedArea > 1100000 & mode2 == 1)
                {
                    cut = 1;
                    data_y = 2;
                    mode = 0;
                    mode3 = 0;
                }
                else
                {
                    cut = 0;
                }
                
                cv::imshow("yolov5", frame);
                writer.write(frame);

                key = cv::waitKey(1);
                if (key == 'q')
                {
                    int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
                    set_serial(fd, 115200, 8, 'N', 1);
                    char buff2[6] = {01, 02, 0, 0, 0, 0};
                    serial_write(fd, buff2, 6);

                    break;
                }
            }
            capture.release();
        }
        break;
        case 's':
        {
            int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
            char buff[3];
            int a = 0;
            char buff2[6] = {01, 02, 0, 0, 0, 1};
            set_serial(fd, 115200, 8, 'N', 1);
            char key;

            while (true)
            {
                std::cin >> key;
                if (key == 'w')
                {

                    buff2[2] += 10; // 按下w时，第三个数据加10
                    std::cout << "buff2[2]" << int(buff2[2]) << "\r\n";
                    serial_write(fd, buff2, 6); // 发送数据
                }
                else if (key == 'e')
                {
                    buff2[2] -= 10;
                    std::cout << "buff2[2]" << int(buff2[2]) << "\r\n";
                    serial_write(fd, buff2, 6); // 发送数据
                }
                else if (key == 'r')
                {
                    buff2[4] -= 5;
                    std::cout << "buff2[3]" << int(buff2[4]) << "\r\n";
                    serial_write(fd, buff2, 6); // 发送数据
                }
                else if (key == 't')
                {
                    buff2[4] += 5;
                    std::cout << "buff2[3]" << int(buff2[4]) << "\r\n";
                    serial_write(fd, buff2, 6); // 发送数据
                }
                else if (key == 'a')
                {
                    buff2[5] = 0;
                    std::cout << "buff2[5]" << int(buff2[5]) << "\r\n";
                    serial_write(fd, buff2, 6); // 发送数据
                }
                else if (key == 's')
                {
                    buff2[5] = 1;
                    std::cout << "buff2[5]" << int(buff2[5]) << "\r\n";
                    serial_write(fd, buff2, 6); // 发送数据
                }
                else if (key == 'z')
                {
                    buff2[3] = 0;
                    std::cout << "buff2[3]" << int(buff2[3]) << "\r\n";
                    serial_write(fd, buff2, 6); // 发送数据
                }
                else if (key == 'x')
                {
                    buff2[3] = 1;
                    std::cout << "buff2[3]" << int(buff2[3]) << "\r\n";
                    serial_write(fd, buff2, 6); // 发送数据
                }
                else if (key == 'c')
                {
                    buff2[3] = 2;
                    std::cout << "buff2[3]" << int(buff2[3]) << "\r\n";
                    serial_write(fd, buff2, 6); // 发送数据
                }
            }
        }
        break;
        case 'g':
        {
            int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
            char buff[3];
            char buff2[6] = {01, 02, 70, 00, 00, 00};
            set_serial(fd, 115200, 8, 'N', 1);

            serial_write(fd, buff2, 6);
        }
        break;
        }
    }
}