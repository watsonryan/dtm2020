#pragma once

// Author: Watson
// Purpose: Density uncertainty model equivalent to sigma_function.f90.

namespace dtm2020 {

float SigmaFunctionPercent(float latitude_deg,
                           float local_time_h,
                           float day_of_year,
                           float altitude_km,
                           float f107m,
                           float kp_delayed_3h);

}  // namespace dtm2020
