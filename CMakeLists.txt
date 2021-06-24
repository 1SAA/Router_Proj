cmake_minimum_required(VERSION 3.16)
project(router)

set(CMAKE_CXX_STANDARD 17)

option(DEBUG_OPTION "toggle debug flag" ON)
if (DEBUG_OPTION)
    set(CMAKE_CXX_FLAGS "-g -DDEBUG_FLAG=1")
else()
endif(DEBUG_OPTION)

unset(DEBUG_OPTION CACHE)

file(GLOB SRC_FILES src/*.cpp)
include_directories(src)

add_executable(cell_move_router ${SRC_FILES})