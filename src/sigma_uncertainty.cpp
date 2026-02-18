// Author: Watson
// Purpose: Port of DTM2020 sigma_function.f90 uncertainty formulation.

#include "dtm2020/sigma_uncertainty.hpp"

namespace dtm2020 {
namespace {

float StdPoly(float x, const float* c, int degree) {
  float y = 0.0F;
  float xp = 1.0F;
  for (int i = 0; i <= degree; ++i) {
    y += c[i] * xp;
    xp *= x;
  }
  return y;
}

float StdFpM(float x) {
  constexpr float c[] = {26.04F, -3.36e-2F};
  return StdPoly(x, c, 1);
}

float StdKpM(float x) {
  constexpr float c[] = {22.47F, 7.46e-1F, -1.99e-1F, 6.56e-2F};
  return StdPoly(x, c, 3);
}

float StdAlatM(float x) {
  constexpr float c[] = {22.33F, -7.19e-3F, 4.72e-4F};
  return StdPoly(x, c, 2);
}

float StdHlM(float x) {
  constexpr float c[] = {28.53F, -3.95e-1F, -5.64e-2F, 3.15e-3F};
  return StdPoly(x, c, 3);
}

float StdAltiM(float x) {
  constexpr float c[] = {-9.73F, 7.92e-2F};
  return StdPoly(x, c, 1);
}

float StdDayM(float x) {
  constexpr float c[] = {23.15F, -3.91e-3F};
  return StdPoly(x, c, 1);
}

}  // namespace

float SigmaFunctionPercent(float latitude_deg,
                           float local_time_h,
                           float day_of_year,
                           float altitude_km,
                           float f107m,
                           float kp_delayed_3h) {
  constexpr float kCopt = 0.929F;
  constexpr float kRes = 112.79F;

  float alt_for_sigma = altitude_km;
  if (altitude_km < 200.0F) {
    alt_for_sigma = 200.0F;
  } else if (altitude_km > 500.0F) {
    alt_for_sigma = 500.0F;
  }

  const float sigma_alat = StdAlatM(latitude_deg);
  const float sigma_hl = StdHlM(local_time_h);
  const float sigma_day = StdDayM(day_of_year);
  const float sigma_alti = StdAltiM(alt_for_sigma);
  const float sigma_fp = StdFpM(f107m);
  const float sigma_kp = StdKpM(kp_delayed_3h);

  return sigma_alat + sigma_hl + sigma_day + sigma_alti + sigma_fp + sigma_kp + kCopt - kRes;
}

}  // namespace dtm2020
