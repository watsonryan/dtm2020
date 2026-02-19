cmake_minimum_required(VERSION 3.25)

project(dtm2020_package_consumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(dtm2020 CONFIG REQUIRED)

add_executable(dtm2020_package_consumer
  main.cpp
)

target_link_libraries(dtm2020_package_consumer PRIVATE dtm2020::dtm2020)
