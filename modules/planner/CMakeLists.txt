cmake_minimum_required(VERSION 2.8.7)
project(futurehorn)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fexceptions -frtti -pthread -O2 -march=core2")
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(ROOT    "${CMAKE_CURRENT_SOURCE_DIR}")

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${ROOT}/
    ${ROOT}/TrajectoryControl
    ${ROOT}/Simulation
    ${ROOT}/PathGenerate
    ${ROOT}/../../third/include
    ${ROOT}/../../third/include/rapidjson
)

link_directories(
    
)

file(GLOB SOURCES
    "*.cpp"
    "*.cxx"
    "${ROOT}/Simulation/*.cpp"
    "${ROOT}/TrajectoryControl/*.cpp"
    "${ROOT}/PathGenerate/*.cpp"
)

set( PROJECT_LINK_LIBS
    ${OpenCV_LIBRARIES}
)
add_executable(planner ${SOURCES})
target_link_libraries(planner ${PROJECT_LINK_LIBS})
