cmake_minimum_required(VERSION 2.8.3)
project(naoqi_launcher)

find_package(qibuild)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES naoqi_launcher
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(_srcs
    main.cpp
    publisher.h
    publisher.cpp
)

qi_create_lib(publisher SHARED ${_srcs} SUBFOLDER naoqi)

qi_use_lib(publisher ALCOMMON ALMEMORYFASTACCESS)

# Also create an executable to start and stop capture
qi_create_bin(testPublisher testPublisher.cpp)
qi_use_lib(testPublisher ALCOMMON)
