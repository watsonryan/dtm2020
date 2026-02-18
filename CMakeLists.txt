cmake_minimum_required(VERSION 3.25)

project(dtm2020_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DTM2020_BUILD_TESTS "Build tests" ON)
option(DTM2020_STRICT_FP "Use strict floating-point flags" ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(warnings)
include(sanitizers)

add_library(dtm2020
  src/dtm2020_operational.cpp
  src/sigma_uncertainty.cpp
)

add_library(dtm2020::dtm2020 ALIAS dtm2020)

target_include_directories(dtm2020
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(DTM2020_STRICT_FP)
  dtm2020_apply_strict_fp_flags(dtm2020)
endif()

dtm2020_apply_common_warnings(dtm2020)

if(DTM2020_BUILD_TESTS)
  include(CTest)
  if(BUILD_TESTING)
    add_subdirectory(tests)
  endif()
endif()
