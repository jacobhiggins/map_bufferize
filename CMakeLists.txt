cmake_minimum_required(VERSION 3.0.2)
project(map_bufferize)

add_compile_options(-std=c++11 -O3)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)

catkin_package(
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_bufferize_node src/map_bufferize_node.cpp)
target_link_libraries(map_bufferize_node ${catkin_LIBRARIES})