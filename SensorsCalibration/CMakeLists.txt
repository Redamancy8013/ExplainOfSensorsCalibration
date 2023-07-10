cmake_minimum_required(VERSION 3.8)
project(lidar2camera)
set(CMAKE_CXX_FLAGS "-g3 -Wall -pthread")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(Pangolin REQUIRED)
link_directories(${Pangolin_LIBRARY_DIRS})

set(pcl_conversions_DIR "/opt/ros/melodic/share/pcl_conversions/cmake/")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/pcl_conversions/cmake/")

set(pcl_msgs_DIR "/opt/ros/melodic/share/pcl_msgs/cmake/")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/pcl_msgs/cmake/")

set(message_runtime_DIR "/opt/ros/melodic/share/message_runtime/cmake/")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/message_runtime/cmake/")

set(cpp_common_DIR "/opt/ros/melodic/share/cpp_common/cmake/")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/cpp_common/cmake/")

set(roscpp_serialization_DIR "/opt/ros/melodic/share/roscpp_serialization/cmake/")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/roscpp_traits/cmake/")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/rostime/cmake/")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/sensor_msgs/cmake/")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/geometry_msgs/cmake/")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/std_msgs/cmake/")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/roscpp/cmake/")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/rosconsole/cmake/")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/rosgraph_msgs/cmake/")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/xmlrpcpp/cmake/")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/catkin/cmake/")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/std_msgs/cmake/")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/roscpp/cmake/")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/rosconsole/cmake/")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/melodic/share/rosgraph_msgs/cmake/")

find_package(pcl_conversions REQUIRED)

find_package(PCL REQUIRED COMPONENTS common visualization io filters)
find_package(Boost REQUIRED system)
find_package(OpenCV 4.2 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

include_directories(
include
${catkin_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN_ROOT})
#include_directories(${PROJECT_SOURCE_DIR}/src)

link_directories(${OpenCV_LIBRARY_DIRS})

#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(${PROJECT_NAME}_node src/run_lidar2camera.cpp)
target_link_libraries(${PROJECT_NAME}_node 
jsoncpp_lib 
${catkin_LIBRARIES} 
${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_FILTERS_LIBRARIES}
${pcl_conversions_LIBRARIES}
${Pangolin_LIBRARIES} 
${OpenCV_LIBS} 
${Boost_SYSTEM_LIBRARY})
