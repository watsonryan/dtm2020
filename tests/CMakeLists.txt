add_executable(dtm2020_sigma_smoke
  sigma_smoke_test.cpp
)

target_link_libraries(dtm2020_sigma_smoke PRIVATE dtm2020)

add_test(NAME dtm2020_sigma_smoke COMMAND dtm2020_sigma_smoke)
