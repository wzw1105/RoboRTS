project(yolox_car)

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

# add_executable(${PROJECT_NAME}_node
#   detect_test.cpp
# )
add_library(yolox_car
    SHARED
    yolox_car.cpp
    armor_process.cpp
)


add_library(detection::yolox_car ALIAS yolox_car)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    detection::tool
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ncnn
    yolox_s_car
)
