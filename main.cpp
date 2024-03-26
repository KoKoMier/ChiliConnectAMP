#include <iostream>
#include  "src/yolov5/yolov5.hpp"
#include "src/Uart/uart.hpp"
#include <iomanip>
#include <cmath>
#include <unistd.h>

int main(int argc, char** argv) {
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
            if (!capture.isOpened()) {
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
                  for ( auto& obj : data.detections) {
                    // Ê¹ÓÃobj.bboxÀ´»ñÈ¡±ß½ç¿ò
        // cv::Rect bbox = cv::Rect(static_cast<int>(obj.bbox[0]), 
        //                  static_cast<int>(obj.bbox[1]),
        //                  static_cast<int>(obj.bbox[2]), 
        //                  static_cast<int>(obj.bbox[3]));                    // Ê¹ÓÃobj.labelÀ´»ñÈ¡Àà±ð±êÇ©
                                                         std::cout<<"obj0"<<obj.bbox[0]<<"\r\n";
                                std::cout<<"obj1"<<obj.bbox[1]<<"\r\n";
                                std::cout<<"obj2"<<obj.bbox[2]<<"\r\n";
                                std::cout<<"obj3"<<obj.bbox[3]<<"\r\n";
                    // ÔÚÍ¼ÏñÉÏ»æÖÆ±ß½ç¿òºÍÀà±ð±êÇ©
                    // cv::rectangle(frame, bbox, cv::Scalar(0, 255, 0), 2);

                    cv::Rect r = get_rect(frame,obj.bbox);
                        
                    cv::rectangle(frame, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                        std::string label = my_classes[(int)obj.class_id];
                        cv::putText(frame, label, cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
}
                    cv::imshow("yolov5", frame);
                key = cv::waitKey(1);
                if (key == 'q') {
                    break;
                }
            }
                     capture.release();
 
        }break;
        case 's':
        {
            int fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY);
            char buff[3];
            char buff2[6] = {01,02,22,33,44,66};
            set_serial(fd,115200,8,'N',1);
            while (true)
            {
                serial_write(fd,buff2,6);
                // read(fd,buff,8);
                // if(buff[0]==0x01 && buff[1]==77)
                // {
                //     std::cout<<"data "<<(int)buff[2]<<"\r\n";
                // }
                // else{
                //     std::cout<<"error"<<"\r\n";
                // }
                usleep(1000000);
            }
            
        }break;
        case 'g':
        {
            int fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY);
            char buff[3];
            char buff2[6] = {01,02,77,00,00,00};
            set_serial(fd,115200,8,'N',1);
  
            serial_write(fd,buff2,6);
            
        }break;
    }
           
}
}