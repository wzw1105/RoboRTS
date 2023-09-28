project(decision)
# cmake_minimum_required(VERSION 3.1)
cmake_minimum_required(VERSION 3.5)

SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  actionlib
  roborts_common
  roborts_msgs
  roborts_costmap
  cv_bridge
  image_transport
)

find_package (Threads)

find_package(Eigen3 REQUIRED)

find_package(PCL 1.7 REQUIRED)

find_package(OpenCV 3 REQUIRED)

find_package(ProtoBuf REQUIRED)

find_package(behaviortree_cpp_v3 REQUIRED ) #

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "/usr/include/eigen3"
  "/usr/include/pcl-1.7"
  "/usr/include/openmpi"
  "/usr/include/google/protobuf"
  "/usr/local/include/behaviortree_cpp_v3" 
)

#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/src/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/src/proto
  DecisionProtoSrc
  DecisionProtoHds
  ${ProtoFiles}
)

catkin_package( )


add_executable(test_decision 
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  src/awaken/awaken.cpp
  src/blackboard/blackboard.cpp
  src/blackboard/gimbal_control.cpp
  src/blackboard/kalman_filter.cpp
  src/blackboard/pid.cpp
  src/executor/chassis_executor.cpp
  src/executor/gimbal_executor.cpp
  src/functions/Functions.cpp
  src/move/move_goal.cpp
  src/patrol/patrol.cpp
  src/shoot/shoot.cpp
  src/swing/swing_idle.cpp
  src/test_decision.cpp
  src/tool/tool.cpp
  src/turn/turn.cpp
)

target_link_libraries(test_decision
  # PRIVATE
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${OpenCV_LIBRARIES}
  roborts_costmap
  behaviortree_cpp_v3
  
)

target_include_directories(test_decision PUBLIC
       ${catkin_INCLUDE_DIRS}
        include)

add_dependencies(test_decision
  roborts_msgs_generate_messages
)
