find_package(Threads REQUIRED)

add_executable(dtm2020_sigma_smoke
  sigma_smoke_test.cpp
)

target_link_libraries(dtm2020_sigma_smoke PRIVATE dtm2020)
dtm2020_apply_common_warnings(dtm2020_sigma_smoke)
dtm2020_apply_runtime_flags(dtm2020_sigma_smoke)

add_test(NAME dtm2020_sigma_smoke COMMAND dtm2020_sigma_smoke)

add_executable(dtm2020_operational_smoke
  operational_smoke_test.cpp
)

target_link_libraries(dtm2020_operational_smoke PRIVATE dtm2020)
dtm2020_apply_common_warnings(dtm2020_operational_smoke)
dtm2020_apply_runtime_flags(dtm2020_operational_smoke)

add_test(NAME dtm2020_operational_smoke COMMAND dtm2020_operational_smoke)

add_executable(dtm2020_operational_golden
  operational_golden_test.cpp
)

target_link_libraries(dtm2020_operational_golden PRIVATE dtm2020)
dtm2020_apply_common_warnings(dtm2020_operational_golden)
dtm2020_apply_runtime_flags(dtm2020_operational_golden)
target_compile_definitions(dtm2020_operational_golden
  PRIVATE
    DTM2020_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    DTM2020_OPERATIONAL_COEFF_FILE="${DTM2020_OPERATIONAL_COEFF_FILE}"
)

add_test(NAME dtm2020_operational_golden COMMAND dtm2020_operational_golden)

add_executable(dtm2020_operational_numerical_regression
  operational_numerical_regression_test.cpp
)

target_link_libraries(dtm2020_operational_numerical_regression PRIVATE dtm2020)
dtm2020_apply_common_warnings(dtm2020_operational_numerical_regression)
dtm2020_apply_runtime_flags(dtm2020_operational_numerical_regression)
target_compile_definitions(dtm2020_operational_numerical_regression
  PRIVATE DTM2020_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

add_test(NAME dtm2020_operational_numerical_regression COMMAND dtm2020_operational_numerical_regression)

add_executable(dtm2020_research_smoke
  research_smoke_test.cpp
)

target_link_libraries(dtm2020_research_smoke PRIVATE dtm2020)
dtm2020_apply_common_warnings(dtm2020_research_smoke)
dtm2020_apply_runtime_flags(dtm2020_research_smoke)

add_test(NAME dtm2020_research_smoke COMMAND dtm2020_research_smoke)

add_executable(dtm2020_research_benchmark
  research_benchmark_test.cpp
)

target_link_libraries(dtm2020_research_benchmark PRIVATE dtm2020)
dtm2020_apply_common_warnings(dtm2020_research_benchmark)
dtm2020_apply_runtime_flags(dtm2020_research_benchmark)
target_compile_definitions(dtm2020_research_benchmark
  PRIVATE
    DTM2020_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    DTM2020_RESEARCH_COEFF_FILE="${DTM2020_RESEARCH_COEFF_FILE}"
)

add_test(NAME dtm2020_research_benchmark COMMAND dtm2020_research_benchmark)

add_executable(dtm2020_operational_perf_benchmark
  operational_perf_benchmark.cpp
)

target_link_libraries(dtm2020_operational_perf_benchmark PRIVATE dtm2020)
dtm2020_apply_common_warnings(dtm2020_operational_perf_benchmark)
dtm2020_apply_runtime_flags(dtm2020_operational_perf_benchmark)

add_executable(dtm2020_research_perf_benchmark
  research_perf_benchmark.cpp
)

target_link_libraries(dtm2020_research_perf_benchmark PRIVATE dtm2020)
dtm2020_apply_common_warnings(dtm2020_research_perf_benchmark)
dtm2020_apply_runtime_flags(dtm2020_research_perf_benchmark)

add_executable(dtm2020_thread_safety_smoke
  thread_safety_smoke_test.cpp
)

target_link_libraries(dtm2020_thread_safety_smoke PRIVATE dtm2020 Threads::Threads)
dtm2020_apply_common_warnings(dtm2020_thread_safety_smoke)
dtm2020_apply_runtime_flags(dtm2020_thread_safety_smoke)

add_test(NAME dtm2020_thread_safety_smoke COMMAND dtm2020_thread_safety_smoke)

if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  set(_dtm2020_release_preset "macos-release")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(_dtm2020_release_preset "linux-release")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
  set(_dtm2020_release_preset "windows-release")
else()
  set(_dtm2020_release_preset "")
endif()

if(_dtm2020_release_preset)
  add_test(
    NAME dtm2020_package_install_smoke
    COMMAND ${CMAKE_COMMAND}
      -DDTM2020_SOURCE_DIR=${CMAKE_SOURCE_DIR}
      -DDTM2020_BINARY_DIR=${CMAKE_BINARY_DIR}
      -DDTM2020_CONSUMER_SOURCE_DIR=${CMAKE_SOURCE_DIR}/tests/package_consumer
      -DDTM2020_RELEASE_PRESET=${_dtm2020_release_preset}
      -P ${CMAKE_SOURCE_DIR}/tests/package_install_smoke.cmake
  )
endif()
