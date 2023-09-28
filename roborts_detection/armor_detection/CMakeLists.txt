project(armor_detection)

# add_subdirectory(constraint_set)
add_subdirectory(yolox_s_car_v1_0_test)
add_subdirectory(yolox_car)


file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  ArmorDetectionProtoSrc
  ArmorDetectionProtoHds
  ${ProtoFiles})

link_directories(
  yolox_car/lib
  ~/ncnn/build/install/lib/
)

#armor detection node
add_executable(${PROJECT_NAME}_node
  ${ArmorDetectionProtoSrc}
  ${ArmorDetectionProtoHds}
  armor_detection_node.cpp
  gimbal_control.cpp
)

target_link_libraries(armor_detection_node
  PRIVATE
  detection::tool
  # detection::constraint_set
  detection::yolox_car
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_include_directories(armor_detection_node
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
)
# add_dependencies(armor_detection_node
#   roborts_msgs_generate_messages)

#armor_detection_client
add_executable(${PROJECT_NAME}_client
  armor_detection_client.cpp
)

target_include_directories(armor_detection_client
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
  )

target_link_libraries(armor_detection_client
  PRIVATE
  detection::tool
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
)
# add_dependencies(armor_detection_client
#   roborts_msgs_generate_messages)
