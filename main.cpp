#include <iostream>
#include  "src/yolov5/yolov5.hpp"

int main(int argc, char** argv) {
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
 
}