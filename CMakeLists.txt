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
    # /usr/local/lib/libsmpl.so # Tushar
)
set(SMPL_INCLUDE_DIRS
    /home/dhruv/work/code/ros/sbpl/devel/include
    # /usr/local/include/ # Tushar
)

find_package(Boost REQUIRED)
find_package(OMPL REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(${SMPL_INCLUDE_DIRS})
include_directories(include)
add_library(
    AMRA
    src/grid2d.cpp
    src/heuristic.cpp
    src/dubins.cpp
    src/dijkstra.cpp
    src/arastar.cpp
    src/amra.cpp
    src/movingai.cpp
    src/constants.cpp
    src/CostConvergenceTerminationCondition.cpp)
target_link_libraries(AMRA ${SMPL_LIBRARIES})

add_executable(run2d test/run2d.cpp)
target_link_libraries(run2d AMRA)

add_executable(ompl_run test/ompl_run.cpp)
target_include_directories(ompl_run SYSTEM PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_libraries(ompl_run AMRA ${SMPL_LIBRARIES} ${OMPL_LIBRARIES})
