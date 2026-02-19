cmake_minimum_required(VERSION 3.25)

if(NOT DEFINED DTM2020_SOURCE_DIR)
  message(FATAL_ERROR "DTM2020_SOURCE_DIR is required")
endif()
if(NOT DEFINED DTM2020_BINARY_DIR)
  message(FATAL_ERROR "DTM2020_BINARY_DIR is required")
endif()
if(NOT DEFINED DTM2020_CONSUMER_SOURCE_DIR)
  message(FATAL_ERROR "DTM2020_CONSUMER_SOURCE_DIR is required")
endif()
if(NOT DEFINED DTM2020_RELEASE_PRESET)
  message(FATAL_ERROR "DTM2020_RELEASE_PRESET is required")
endif()

set(install_dir "${DTM2020_BINARY_DIR}/package-smoke-install")
set(consumer_build_dir "${DTM2020_BINARY_DIR}/package-consumer-smoke")
set(release_build_dir "${DTM2020_SOURCE_DIR}/build/${DTM2020_RELEASE_PRESET}")

execute_process(
  COMMAND "${CMAKE_COMMAND}" --preset "${DTM2020_RELEASE_PRESET}"
  WORKING_DIRECTORY "${DTM2020_SOURCE_DIR}"
  RESULT_VARIABLE rc_configure
)
if(NOT rc_configure EQUAL 0)
  message(FATAL_ERROR "package smoke: release configure failed")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" --build --preset "${DTM2020_RELEASE_PRESET}"
  WORKING_DIRECTORY "${DTM2020_SOURCE_DIR}"
  RESULT_VARIABLE rc_build
)
if(NOT rc_build EQUAL 0)
  message(FATAL_ERROR "package smoke: release build failed")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" --install "${release_build_dir}" --prefix "${install_dir}"
  WORKING_DIRECTORY "${DTM2020_SOURCE_DIR}"
  RESULT_VARIABLE rc_install
)
if(NOT rc_install EQUAL 0)
  message(FATAL_ERROR "package smoke: install failed")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -S "${DTM2020_CONSUMER_SOURCE_DIR}" -B "${consumer_build_dir}"
          -G Ninja
          -DCMAKE_BUILD_TYPE=Release
          -DCMAKE_PREFIX_PATH=${install_dir}
  RESULT_VARIABLE rc_consumer_configure
)
if(NOT rc_consumer_configure EQUAL 0)
  message(FATAL_ERROR "package smoke: consumer configure failed")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${consumer_build_dir}"
  RESULT_VARIABLE rc_consumer_build
)
if(NOT rc_consumer_build EQUAL 0)
  message(FATAL_ERROR "package smoke: consumer build failed")
endif()

set(consumer_exe "${consumer_build_dir}/dtm2020_package_consumer")
if(WIN32)
  set(consumer_exe "${consumer_exe}.exe")
endif()

execute_process(
  COMMAND "${consumer_exe}"
  RESULT_VARIABLE rc_consumer_run
)
if(NOT rc_consumer_run EQUAL 0)
  message(FATAL_ERROR "package smoke: consumer execution failed")
endif()
