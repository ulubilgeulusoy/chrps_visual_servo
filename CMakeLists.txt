cmake_minimum_required(VERSION 3.10)
project(chrps_visual_servo CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find your installed ViSP (from source or packages)
find_package(VISP REQUIRED)

include_directories(${VISP_INCLUDE_DIRS})

add_executable(servoFrankaIBVS_CHRPS src/servoFrankaIBVS_CHRPS.cpp)
target_link_libraries(servoFrankaIBVS_CHRPS ${VISP_LIBRARIES})
