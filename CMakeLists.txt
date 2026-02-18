cmake_minimum_required(VERSION 3.25)

project(dtm2020_cpp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DTM2020_BUILD_TESTS "Build tests" ON)
option(DTM2020_STRICT_FP "Use strict floating-point flags" ON)
option(DTM2020_ENABLE_RESEARCH "Build optional research-model scaffold" ON)
set(DTM2020_SANITIZER "" CACHE STRING "Sanitizer to use: '', address, undefined, thread")
option(DTM2020_ENABLE_PROFILE "Enable profiling-oriented optimization flags" OFF)
set(DTM2020_OPERATIONAL_COEFF_FILE "" CACHE FILEPATH "Path to external DTM_2020_F107_Kp.dat for golden regression test")
set(DTM2020_RESEARCH_COEFF_FILE "" CACHE FILEPATH "Path to external DTM_2020_F30_ap60.dat for research benchmark test")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(warnings)
include(sanitizers)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(dtm2020
  src/dtm2020_operational.cpp
  src/error_utils.cpp
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
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
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

install(TARGETS dtm2020
  EXPORT dtm2020Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

set(DTM2020_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/dtm2020")

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/dtm2020ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/dtm2020Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/dtm2020Config.cmake"
  INSTALL_DESTINATION "${DTM2020_INSTALL_CMAKEDIR}"
)

install(EXPORT dtm2020Targets
  FILE dtm2020Targets.cmake
  NAMESPACE dtm2020::
  DESTINATION "${DTM2020_INSTALL_CMAKEDIR}"
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/dtm2020Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/dtm2020ConfigVersion.cmake"
  DESTINATION "${DTM2020_INSTALL_CMAKEDIR}"
)

if(DTM2020_BUILD_TESTS)
  include(CTest)
  if(BUILD_TESTING)
    add_subdirectory(tests)
  endif()
endif()
