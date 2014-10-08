cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
project(getvalorescorriente)
find_package(qibuild)

set(_srcs
    main.cpp
    getvalorescorriente.h
    getvalorescorriente.cpp
)

qi_create_lib(getvalorescorriente SHARED ${_srcs} SUBFOLDER naoqi)

qi_use_lib(getvalorescorriente ALCOMMON ALMEMORYFASTACCESS)

# Also create an executable to start and stop capture
qi_create_bin(captureValues captureValues.cpp)
qi_use_lib(captureValues ALCOMMON)
