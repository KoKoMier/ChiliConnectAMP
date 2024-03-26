cmake_minimum_required (VERSION 2.8)
project (src)

aux_source_directory(./yolov5 SRC)
add_library(src ${SRC})

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

target_link_libraries(src ${OpenCV_LIBS})


