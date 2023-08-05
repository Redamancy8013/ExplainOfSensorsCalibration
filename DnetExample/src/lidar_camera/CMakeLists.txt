CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(DnetExample)
set(CMAKE_CXX_FLAGS "-Wall -pthread")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common visualization io filters)
find_package(Pangolin REQUIRED)
#find_package(jsoncpp REQUIRED)

SET(ALL_PROJECT_TOPLEVEL_PATH "/home/bryan/projects/ExplainOfSensorsCalibration/${PROJECT_NAME}/")
MESSAGE("ALL_PROJECT_TOPLEVEL_PATH: " ${ALL_PROJECT_TOPLEVEL_PATH})


include_directories(
 /usr/include/jsoncpp
 include
 ${PCL_INCLUDE_DIRS} 
 ${ALL_PROJECT_TOPLEVEL_PATH}/include/
 ${EIGEN_ROOT}
 ${OpenCV_INCLUDE_DIRS}
 ${Pangolin_INCLUDE_DIRS}
 )

LINK_DIRECTORIES(
 ${ALL_PROJECT_TOPLEVEL_PATH}/lib/
 ${PCL_LIBRARY_DIRS}
 ${OpenCV_LIBRARY_DIRS}
 )
add_definitions(${PCL_DEFINITIONS})

SET(EXECUTABLE_OUTPUT_PATH ${ALL_PROJECT_TOPLEVEL_PATH}/bin)

ADD_EXECUTABLE(lidar_camera
receive_main.cpp
receive_module.cpp
../common/timestamp.c)

TARGET_LINK_LIBRARIES(lidar_camera
dnet
jsoncpp
${PCL_COMMON_LIBRARIES} 
${PCL_IO_LIBRARIES} 
${PCL_VISUALIZATION_LIBRARIES} 
${PCL_FILTERS_LIBRARIES}
${OpenCV_LIBS} 
${Pangolin_LIBRARIES} 
)
