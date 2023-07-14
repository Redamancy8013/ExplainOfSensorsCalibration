CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(DnetExample)
SET(ALL_PROJECT_TOPLEVEL_PATH "/home/easycool/project/dnet/${PROJECT_NAME}/")
MESSAGE("ALL_PROJECT_TOPLEVEL_PATH: " ${ALL_PROJECT_TOPLEVEL_PATH})

SET(SOURCES_LIST
	receive_module.cpp
	receive_main.cpp
	../common/timestamp.c
)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common visualization io filters)
find_package(Boost REQUIRED system)

include_directories(
include
 ${PCL_INCLUDE_DIRS} 
 ${ALL_PROJECT_TOPLEVEL_PATH}/include/
 ${EIGEN_ROOT})

LINK_DIRECTORIES(${ALL_PROJECT_TOPLEVEL_PATH}/lib/)

SET(EXECUTABLE_OUTPUT_PATH ${ALL_PROJECT_TOPLEVEL_PATH}/bin)
ADD_EXECUTABLE(receive ${SOURCES_LIST})

TARGET_LINK_LIBRARIES(lidar_receive dnet
${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_FILTERS_LIBRARIES}
${pcl_conversions_LIBRARIES}
pcl_visualization
${Boost_SYSTEM_LIBRARY}
)


