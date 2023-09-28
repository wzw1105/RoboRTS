project(roborts_decision)
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

find_package(Eigen3 REQUIRED)

find_package(PCL 1.7 REQUIRED)

find_package(OpenCV 3 REQUIRED)

find_package(ProtoBuf REQUIRED)

find_package(behaviortree_cpp_v3 REQUIRED ) #

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
  "/usr/include/pcl-1.7"
  "/usr/include/google/protobuf"
  "/usr/local/include/behaviortree_cpp_v3" 
)

#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  DecisionProtoSrc
  DecisionProtoHds
  ${ProtoFiles}
)

catkin_package( )


add_executable(decision_node 
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  # action_node/FindAvailableShootPositionWithGuardMsg.h
  # action_node/InitTheRobot.h
  # action_node/MoveToBloodAddArea.h
  # action_node/MoveToBootArea.h
  # action_node/MoveToBulletsAddArea.h
  # action_node/MoveToEnermyArea.h
  # action_node/Patrol.h
  # action_node/Shoot.h
  # action_node/Swing360.h
  blackboard/blackboard.cpp
  executor/chassis_executor.cpp
  executor/gimbal_executor.cpp
  decision.cpp
)

target_link_libraries(decision_node
  # PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  behaviortree_cpp_v3
)

add_dependencies(decision_node
  roborts_msgs_generate_messages
)
