add_executable(dtm2020_sigma_smoke
  sigma_smoke_test.cpp
)

target_link_libraries(dtm2020_sigma_smoke PRIVATE dtm2020)

add_test(NAME dtm2020_sigma_smoke COMMAND dtm2020_sigma_smoke)

add_executable(dtm2020_operational_smoke
  operational_smoke_test.cpp
)

target_link_libraries(dtm2020_operational_smoke PRIVATE dtm2020)

add_test(NAME dtm2020_operational_smoke COMMAND dtm2020_operational_smoke)

add_executable(dtm2020_operational_golden
  operational_golden_test.cpp
)

target_link_libraries(dtm2020_operational_golden PRIVATE dtm2020)

add_test(NAME dtm2020_operational_golden COMMAND dtm2020_operational_golden)

add_executable(dtm2020_research_smoke
  research_smoke_test.cpp
)

target_link_libraries(dtm2020_research_smoke PRIVATE dtm2020)

add_test(NAME dtm2020_research_smoke COMMAND dtm2020_research_smoke)

add_executable(dtm2020_research_benchmark
  research_benchmark_test.cpp
)

target_link_libraries(dtm2020_research_benchmark PRIVATE dtm2020)

add_test(NAME dtm2020_research_benchmark COMMAND dtm2020_research_benchmark)
