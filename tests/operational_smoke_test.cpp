// Author: Watson
// Purpose: Smoke tests for coefficient parsing and operational evaluation path.

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "dtm2020/dtm2020_operational.hpp"

namespace {

bool WriteSyntheticCoeffFile(const std::filesystem::path& file_path) {
  std::ofstream out(file_path);
  if (!out.is_open()) {
    return false;
  }

  out << "Synthetic DTM coefficients for smoke test\n";
  out << "96\n";

  for (int i = 1; i <= 96; ++i) {
    float tt = 0.0F;
    float h = 0.0F;
    float he = 0.0F;
    float o = 0.0F;
    float az2 = 0.0F;
    float o2 = 0.0F;
    float az = 0.0F;
    float t0 = 0.0F;
    float tp = 0.0F;

    if (i == 1) {
      tt = 1000.0F;
      h = 2.0e11F;
      he = 1.0e11F;
      o = 3.0e11F;
      az2 = 5.0e10F;
      o2 = 4.0e10F;
      az = 2.0e10F;
      t0 = 500.0F;
      tp = 100.0F;
    }
    if (i == 21) {
      az2 = 2.0F;
    }

    out << std::setw(4) << i << " " << std::uppercase << std::scientific << std::setprecision(6);
    if (i == 1) {
      out << "1.000000D+03";
    } else {
      out << tt;
    }
    out << " " << 0.0F << " "
        << h << " " << 0.0F << " "
        << he << " " << 0.0F << " "
        << o << " " << 0.0F << " "
        << az2 << " " << 0.0F << " "
        << o2 << " " << 0.0F << " "
        << az << " " << 0.0F << " "
        << t0 << " " << 0.0F << " "
        << tp << " " << 0.0F << "\n";
  }

  return true;
}

bool AlmostEqual(double a, double b, double tol) {
  return std::abs(a - b) <= tol;
}

}  // namespace

int main() {
  const auto tmp_file = std::filesystem::temp_directory_path() / "dtm2020_coeff_smoke.dat";
  if (!WriteSyntheticCoeffFile(tmp_file)) {
    return EXIT_FAILURE;
  }

  dtm2020::Error error;
  auto model = dtm2020::Dtm2020Operational::LoadFromFile(tmp_file, error);
  if (!model || error.code != dtm2020::ErrorCode::kNone) {
    return EXIT_FAILURE;
  }

  const dtm2020::OperationalInputs base = {
      .altitude_km = 200.0,
      .latitude_deg = 25.0,
      .longitude_deg = 10.0,
      .local_time_h = 4.0,
      .day_of_year = 120.0,
      .f107 = 120.0,
      .f107m = 120.0,
      .kp_delayed_3h = 2.0,
      .kp_mean_24h = 2.0,
  };

  auto out1 = model->Evaluate(base, error);
  if (error.code != dtm2020::ErrorCode::kNone) {
    return EXIT_FAILURE;
  }

  if (!(std::isfinite(out1.temperature_k) && std::isfinite(out1.density_g_cm3) && out1.density_g_cm3 > 0.0)) {
    return EXIT_FAILURE;
  }

  auto out2 = model->Evaluate(
      dtm2020::OperationalInputs{.altitude_km = base.altitude_km,
                                 .latitude_deg = base.latitude_deg,
                                 .longitude_deg = base.longitude_deg + 360.0,
                                 .local_time_h = base.local_time_h + 24.0,
                                 .day_of_year = base.day_of_year,
                                 .f107 = base.f107,
                                 .f107m = base.f107m,
                                 .kp_delayed_3h = base.kp_delayed_3h,
                                 .kp_mean_24h = base.kp_mean_24h},
      error);

  if (error.code != dtm2020::ErrorCode::kNone) {
    return EXIT_FAILURE;
  }

  if (!AlmostEqual(out1.temperature_k, out2.temperature_k, 1e-5) ||
      !AlmostEqual(out1.density_g_cm3, out2.density_g_cm3, 1e-18)) {
    return EXIT_FAILURE;
  }

  auto bad = model->Evaluate(
      dtm2020::OperationalInputs{.altitude_km = 120.0,
                                 .latitude_deg = base.latitude_deg,
                                 .longitude_deg = base.longitude_deg,
                                 .local_time_h = base.local_time_h,
                                 .day_of_year = base.day_of_year,
                                 .f107 = base.f107,
                                 .f107m = base.f107m,
                                 .kp_delayed_3h = base.kp_delayed_3h,
                                 .kp_mean_24h = base.kp_mean_24h},
      error);
  (void)bad;
  if (error.code != dtm2020::ErrorCode::kInvalidInput) {
    return EXIT_FAILURE;
  }

  const float sigma = model->DensityUncertaintyPercent(base);
  if (!(std::isfinite(sigma) && sigma > 0.0F)) {
    return EXIT_FAILURE;
  }

  const dtm2020::Dtm2020Operational::Options mcm_on{.emulate_mcm_transition = true};
  const dtm2020::Dtm2020Operational::Options mcm_off{.emulate_mcm_transition = false};
  auto model_mcm_on = dtm2020::Dtm2020Operational::LoadFromFile(tmp_file, error, mcm_on);
  if (!model_mcm_on || error.code != dtm2020::ErrorCode::kNone) {
    return EXIT_FAILURE;
  }
  auto model_mcm_off = dtm2020::Dtm2020Operational::LoadFromFile(tmp_file, error, mcm_off);
  if (!model_mcm_off || error.code != dtm2020::ErrorCode::kNone) {
    return EXIT_FAILURE;
  }

  auto out_mcm_on = model_mcm_on->Evaluate(
      dtm2020::OperationalInputs{.altitude_km = 130.0,
                                 .latitude_deg = base.latitude_deg,
                                 .longitude_deg = base.longitude_deg,
                                 .local_time_h = base.local_time_h,
                                 .day_of_year = base.day_of_year,
                                 .f107 = base.f107,
                                 .f107m = base.f107m,
                                 .kp_delayed_3h = base.kp_delayed_3h,
                                 .kp_mean_24h = base.kp_mean_24h},
      error);
  if (error.code != dtm2020::ErrorCode::kNone) {
    return EXIT_FAILURE;
  }
  auto out_mcm_off = model_mcm_off->Evaluate(
      dtm2020::OperationalInputs{.altitude_km = 130.0,
                                 .latitude_deg = base.latitude_deg,
                                 .longitude_deg = base.longitude_deg,
                                 .local_time_h = base.local_time_h,
                                 .day_of_year = base.day_of_year,
                                 .f107 = base.f107,
                                 .f107m = base.f107m,
                                 .kp_delayed_3h = base.kp_delayed_3h,
                                 .kp_mean_24h = base.kp_mean_24h},
      error);
  if (error.code != dtm2020::ErrorCode::kNone) {
    return EXIT_FAILURE;
  }

  if (AlmostEqual(out_mcm_on.density_g_cm3, out_mcm_off.density_g_cm3, 1e-22)) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
