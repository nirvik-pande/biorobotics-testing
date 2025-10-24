cmake_minimum_required(VERSION 3.15)
project(astar CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(astar
  src/main.cpp
)
target_include_directories(astar PRIVATE ${CMAKE_SOURCE_DIR}/include)