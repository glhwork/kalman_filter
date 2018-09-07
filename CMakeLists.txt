cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter)

add_compile_options(-std=c++11)

include_directories(
  include
)

add_executable(${PROJECT_NAME}_main
  src/main.cpp
)


