cmake_minimum_required(VERSION 3.19)
project(XSENS_Test)

set(CMAKE_CXX_STANDARD 11)

include_directories(xspublic)
link_directories(xspublic/xscommon xspublic/xscontroller xspublic/xstypes)

add_executable(XSENS_Test source_udp.cpp xsens.h)

target_link_libraries(XSENS_Test xscontroller xscommon xstypes pthread dl)