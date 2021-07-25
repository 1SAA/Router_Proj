cmake_minimum_required(VERSION 3.16)
project(router)

set(CMAKE_CXX_STANDARD 17)

option(DEBUG_OPTION "toggle debug flag" ON)
set(CMAKE_CXX_FLAGS "-Wall -Wformat-overflow=0")
if (DEBUG_OPTION)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -DDEBUG_FLAG=1")
else()
endif(DEBUG_OPTION)

unset(DEBUG_OPTION CACHE)

file(GLOB_RECURSE SRC_FILES src/*.cpp)
file(GLOB_RECURSE FLUTE src/flute/*.c)

include_directories(src/flute 
                    src)
add_subdirectory(src/flute)
add_executable(cell_move_router ${SRC_FILES})
target_link_libraries(cell_move_router flute)
