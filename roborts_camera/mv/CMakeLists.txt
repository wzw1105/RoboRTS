project(mv_driver)

#uvc_driver
add_library(mv_driver
  SHARED
  mv_driver.cpp
)

link_directories(${PROJECT_SOURCE_DIR}/mv/3rdparty/lib)

target_link_libraries(mv_driver
  PRIVATE
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${BOOST_LIBRARIES}
  -lpthread -lMVSDK
)

target_include_directories(mv_driver
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
  ${PROJECT_SOURCE_DIR}/3rdparty/include
)

add_library(driver::mv_driver ALIAS mv_driver)


# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   ${PROJECT_SOURCE_DIR}/3rdparty/include
# )

# set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

# add_executable(mv_camera_node mv_camera_node.cpp)

# link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${PROJECT_SOURCE_DIR}/3rdparty/lib)

# target_link_libraries(mv_camera_node
#   ${BOOST_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} -lpthread -lMVSDK)