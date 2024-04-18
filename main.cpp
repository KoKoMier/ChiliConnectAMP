#include <iostream>
#include "src/yolov5/yolov5.hpp"
#include "src/Uart/uart.hpp"
#include <iomanip>
#include <cmath>
#include <unistd.h>

int main(int argc, char **argv)
{
    int argvs;

    while ((argvs = getopt(argc, argv, "a::s::g::m::c::f::R")) != -1)
    {
        switch (argvs)
        {
        case 'a':
        {
            YOLOV5 yolo;
            yolo_data data;
            cv::VideoCapture capture(0);
            if (!capture.isOpened())
            {
                std::cout << "Error opening video stream or file" << std::endl;
                return -1;
            }
            int key;
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
                    if (obj.conf > max_confidence)
                    {
                        max_confidence = obj.conf;
                        max_conf_rect = get_rect(frame, obj.bbox);
                    }
                }

                if (max_confidence > 0.0)
                {
                    std::cout << "Max confidence rect x: " << max_conf_rect.x << ", y: " << max_conf_rect.y << std::endl;
                    std::cout << "Max confidence rect area: " << max_conf_rect.area() << std::endl;//4000 --- 30cm
                }

                for (auto &obj : data.detections)
                {
                    cv::Rect r = get_rect(frame, obj.bbox);

                    cv::rectangle(frame, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                    std::string label = my_classes[(int)obj.class_id];
                    char score_str[8];
                    sprintf(score_str, "%.2f", obj.conf);
                    std::string conf(score_str);
                    std::string label_with_conf = label + " " + conf;
                    cv::putText(frame, label_with_conf, cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                }

                cv::imshow("yolov5", frame);
                key = cv::waitKey(1);
                if (key == 'q')
                {
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
            char buff2[6] = {01, 02, 0, 33, 44, 1};
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
                    buff2[2] -= 10; // 按下e时，第三个数据减10
                    std::cout << "buff2[2]" << int(buff2[2]) << "\r\n";
                    serial_write(fd, buff2, 6); // 发送数据
                }
                else if (key == 'r')
                {
                    buff2[4] -= 5; // 按下e时，第三个数据减5
                    std::cout << "buff2[3]" << int(buff2[4]) << "\r\n";
                    serial_write(fd, buff2, 6); // 发送数据
                }
                else if (key == 't')
                {
                    buff2[4] += 5; // 按下e时，第三个数据减5
                    std::cout << "buff2[3]" << int(buff2[4]) << "\r\n";
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