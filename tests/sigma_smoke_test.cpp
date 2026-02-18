// Author: Watson
// Purpose: Basic smoke test for sigma_function parity branch behavior.

#include <cstdlib>

#include "dtm2020/sigma_uncertainty.hpp"

int main() {
  const float low = dtm2020::SigmaFunctionPercent(0.0F, 12.0F, 100.0F, 150.0F, 120.0F, 3.0F);
  const float mid = dtm2020::SigmaFunctionPercent(0.0F, 12.0F, 100.0F, 300.0F, 120.0F, 3.0F);
  const float high = dtm2020::SigmaFunctionPercent(0.0F, 12.0F, 100.0F, 900.0F, 120.0F, 3.0F);

  if (!(low > 0.0F && mid > 0.0F && high > 0.0F)) {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
