cmake_minimum_required(VERSION 3.25)
project(QuickhullNew)

set(CMAKE_CXX_STANDARD 23)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -lm -O2 -lstdc++")

add_executable(Quickhull demo.cpp QuickHull.cpp QuickHull.h)
