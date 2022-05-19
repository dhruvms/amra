cmake_minimum_required(VERSION 3.5.1)
project(amra)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fmax-errors=5")

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OMPL REQUIRED)

add_library(
    SMPL
    src/smpl/console/ansi.cpp
    src/smpl/console/console.cpp
    src/smpl/unicycle/unicycle.cpp
    src/smpl/unicycle/dubins.cpp)
target_link_libraries(SMPL ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_SYSTEM_LIBRARY})

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(include)

add_library(
    AMRA
    src/amra/grid2d.cpp
    src/amra/griduav.cpp
    src/amra/heuristic.cpp
    src/amra/dubins.cpp
    src/amra/dijkstra.cpp
    src/amra/arastar.cpp
    src/amra/amra.cpp
    src/amra/movingai.cpp
    src/amra/constants.cpp
    src/amra/CostConvergenceTerminationCondition.cpp)
target_link_libraries(AMRA SMPL)

add_executable(run2d test/run2d.cpp)
target_link_libraries(run2d AMRA)

add_executable(runuav test/runuav.cpp)
target_link_libraries(runuav AMRA)

add_executable(ompl_run test/ompl_run.cpp)
target_include_directories(ompl_run SYSTEM PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_libraries(ompl_run AMRA SMPL ${OMPL_LIBRARIES})
