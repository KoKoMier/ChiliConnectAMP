cmake_minimum_required (VERSION 2.8)
project (ChiliDetection)

# 添加多线程支持
find_package(Threads REQUIRED)

add_subdirectory(src)
link_libraries(src)

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

find_package(CUDA REQUIRED)

include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)

cuda_add_library(myplugins SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/yolov5/yololayer.cu)
target_link_libraries(myplugins nvinfer cudart)

# 添加多线程库到可执行文件
add_executable(ChiliAPM ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
target_link_libraries(ChiliAPM myplugins Threads::Threads)