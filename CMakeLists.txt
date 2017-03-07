cmake_minimum_required(VERSION 3.6)
find_package(freenect2 REQUIRED)

add_executable(testFreenect2 main.cpp)
target_link_libraries(testFreenect2 ${freenect2_LIBRARIES})
include_directories(${freenect2_INCLUDE_DIRS})
