cmake_minimum_required(VERSION 3.19)
project(BallARENA-dev)

set(CMAKE_CXX_STANDARD 11)

add_executable(BallARENA-dev ballet.cpp hmath.h ballet_includes.h ballet.h)

target_link_libraries(BallARENA-dev ${OpenCV_LIBS} ${CURL_LIBRARIES})
