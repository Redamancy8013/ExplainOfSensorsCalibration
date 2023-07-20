cmake_minimum_required(VERSION 3.5)

project(dqt_bag LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fopenmp ")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.7 REQUIRED)
include_directories( "/usr/include/eigen3" )
find_package(OpenCV)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_definitions( ${PCL_DEFINITIONS} )
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} )

#set(Dqtbag_SRC offline_playback.cpp  my_log.cpp)
#add_library(dqtbag SHARED ${Dqtbag_SRC})
#set_target_properties(dqtbag PROPERTIES VERSION 1.0 SOVERSION 1)

add_executable(dqt_bag
      main.cpp
      mainwindow.cpp
      mainwindow.h
      mainwindow.ui
      progress.h
      progress.cpp
      offline_playback.h
      offline_playback.cpp
      ./msg/ugv_topics.h
      ./msg/sensor_topics.h
      ./msg/timestamp.h
      my_log.h
      my_log.cpp
)

target_link_libraries(dqt_bag PRIVATE Qt5::Widgets
                                      dnet
                                      ${PCL_LIBRARIES}
                                      ${QT_LIBRARIES}
                                      ${OpenCV_LIBS}
                                      ${Boost_LIBRARIES})
