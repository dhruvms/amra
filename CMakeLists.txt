cmake_minimum_required(VERSION 3.5.1)
project(amra)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fmax-errors=5")

# # To use asan in debug mode
# set(CMAKE_CXX_FLAGS_DEBUG "-fno-omit-frame-pointer -fsanitize=address")
# set(CMAKE_LINKER_FLAGS_DEBUG "-fno-omit-frame-pointer -fsanitize=address")

# # To use asan in release mode
# set(CMAKE_CXX_FLAGS_RELEASE "-fno-omit-frame-pointer -fsanitize=address")
# set(CMAKE_LINKER_FLAGS_RELEASE "-fno-omit-frame-pointer -fsanitize=address")

set(
    SMPL_LIBRARIES
    /home/dhruv/work/code/ros/sbpl/build/smpl/libsmpl.so
)
set(SMPL_INCLUDE_DIRS
    /home/dhruv/work/code/ros/sbpl/devel/include
)

include_directories(include)
include_directories(${SMPL_INCLUDE_DIRS})
add_library(
    AMRA
    src/environment.cpp
    src/heuristic.cpp
    src/amra.cpp
    src/movingai.cpp
    src/constants.cpp)
target_link_libraries(AMRA ${SMPL_LIBRARIES})

add_executable(run2d test/run2d.cpp)
target_link_libraries(run2d AMRA)
