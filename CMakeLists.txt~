cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
project(publisher)
find_package(qibuild)

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
