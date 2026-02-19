/**
 * @file sigma_uncertainty.hpp
 * @brief Density uncertainty model API equivalent to `sigma_function.f90`.
 */
#pragma once

// Author: Watson
// Purpose: Density uncertainty model equivalent to sigma_function.f90.

namespace dtm2020 {

/**
 * @brief Estimate 1-sigma density uncertainty in percent.
 * @param latitude_deg Geodetic latitude in degrees.
 * @param local_time_h Local solar time in hours.
 * @param day_of_year Day-of-year in [1, 366].
 * @param altitude_km Altitude in kilometers.
 * @param f107m 81-day mean F10.7 index.
 * @param kp_delayed_3h Delayed 3-hour Kp index.
 * @return Estimated uncertainty percentage.
 */
float SigmaFunctionPercent(float latitude_deg,
                           float local_time_h,
                           float day_of_year,
                           float altitude_km,
                           float f107m,
                           float kp_delayed_3h);

}  // namespace dtm2020
