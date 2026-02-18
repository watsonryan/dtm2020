cmake_minimum_required(VERSION 3.25)

project(dtm2020_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DTM2020_BUILD_TESTS "Build tests" ON)
option(DTM2020_STRICT_FP "Use strict floating-point flags" ON)
option(DTM2020_ENABLE_RESEARCH "Build optional research-model scaffold" ON)
set(DTM2020_SANITIZER "" CACHE STRING "Sanitizer to use: '', address, undefined, thread")
option(DTM2020_ENABLE_PROFILE "Enable profiling-oriented optimization flags" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(warnings)
include(sanitizers)

add_library(dtm2020
  src/dtm2020_operational.cpp
  src/sigma_uncertainty.cpp
)

if(DTM2020_ENABLE_RESEARCH)
  target_sources(dtm2020 PRIVATE
    src/dtm2020_research.cpp
  )
  target_compile_definitions(dtm2020 PUBLIC DTM2020_ENABLE_RESEARCH=1)
endif()

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
dtm2020_apply_runtime_flags(dtm2020)

add_executable(dtm2020_cli
  examples/dtm2020_cli.cpp
)
target_link_libraries(dtm2020_cli PRIVATE dtm2020)
dtm2020_apply_common_warnings(dtm2020_cli)
dtm2020_apply_runtime_flags(dtm2020_cli)

if(DTM2020_BUILD_TESTS)
  include(CTest)
  if(BUILD_TESTING)
    add_subdirectory(tests)
  endif()
endif()
