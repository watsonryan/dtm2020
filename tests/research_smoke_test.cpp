// Author: Watson
// Purpose: Smoke test for research-model scaffold behavior.

#include <cstdlib>

#include "dtm2020/dtm2020_research.hpp"

int main() {
#ifdef DTM2020_ENABLE_RESEARCH
  auto model = dtm2020::Dtm2020Research::CreateDefault();
  dtm2020::Error error;
  auto out = model.Evaluate(dtm2020::ResearchInputs{}, error);
  (void)out;
  if (error.code != dtm2020::ErrorCode::kInvalidInput) {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
#else
  return EXIT_SUCCESS;
#endif
}
