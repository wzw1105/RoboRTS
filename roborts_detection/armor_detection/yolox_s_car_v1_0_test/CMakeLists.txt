project(yolox_s_car_v1_0_test)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    OpenCV
)

include_directories(include)
include_directories(~/ncnn/build/install/include/ncnn/)

link_directories(
  lib
  ~/ncnn/build/install/lib/
)

add_executable(${PROJECT_NAME}_node
  detect_test.cpp
)

target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node
    PUBLIC
    ${catkin_LIBRARIES}
    ncnn
    yolox_s_car
)