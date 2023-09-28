project(roborts_socket)
cmake_minimum_required(VERSION 3.1)
add_definitions(-std=c++11)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

find_package(Threads REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  roborts_msgs
  )

catkin_package()


add_executable(client_node	
        client_node.cpp
)

target_link_libraries(client_node
        PUBLIC
        
        ${catkin_LIBRARIES}
        
        )
target_include_directories(client_node 
	PUBLIC
        ${catkin_INCLUDE_DIRS})


