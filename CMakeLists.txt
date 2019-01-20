cmake_minimum_required(VERSION 2.8.3)
project(ForwardKinematics)
add_definitions(-std=c++11 -Wall -O2 )

find_package( Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
        ./src
        ./src/Utility
)


add_library( ForwardKinematics  ./src/ForwardKinematics.cpp)


add_executable(ForKin ./test/ForKin_test.cpp)
target_link_libraries(ForKin ForwardKinematics)