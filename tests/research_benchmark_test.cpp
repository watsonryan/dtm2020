// Author: Watson
// Purpose: Validate research-model outputs against published F30/Hp60 benchmark values.

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>

#include "dtm2020/logging.hpp"
#include "dtm2020/dtm2020_research.hpp"

namespace {

bool NearlyEqualRelative(double a, double b, double rel_tol, double abs_floor) {
  const double diff = std::abs(a - b);
  const double scale = std::max(std::abs(a), std::abs(b));
  return diff <= std::max(abs_floor, rel_tol * scale);
}

}  // namespace

int main() {
#ifdef DTM2020_ENABLE_RESEARCH
  const auto log = dtm2020::MakeStderrLogSink();
  const auto coeff_file = std::filesystem::path("/Users/rmw/Documents/code/mcm/data/DTM_2020_F30_ap60.dat");
  if (!std::filesystem::exists(coeff_file)) {
    return EXIT_SUCCESS;
  }

  auto model = dtm2020::Dtm2020Research::LoadFromFile(coeff_file);
  if (!model) {
    dtm2020::LogError(log, "load failed", model.error());
    return EXIT_FAILURE;
  }

  dtm2020::ResearchInputs in{};
  in.latitude_deg = 0.0;
  in.longitude_deg = 0.0;
  in.local_time_h = 12.0;
  in.day_of_year = 180.0;
  in.ap60 = {15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0};

  // Benchmark targets from Mod_DTM2020F30Hp60_Benchmark.f90
  struct Case {
    double alt_km;
    double f30;
    double f30m;
    double tz;
    double tinf;
    double ro;
  };

  const Case cases[] = {
      {300.0, 80.0, 80.0, 835.545, 836.285, 0.93545e-14},
      {300.0, 180.0, 180.0, 1280.280, 1297.688, 0.28889e-13},
      {800.0, 80.0, 80.0, 836.285, 836.285, 0.29765e-17},
      {800.0, 180.0, 180.0, 1297.687, 1297.688, 0.31045e-16},
  };

  for (const auto& c : cases) {
    in.altitude_km = c.alt_km;
    in.f30 = c.f30;
    in.f30m = c.f30m;
    const auto out = model.value().Evaluate(in);
    if (!out) {
      dtm2020::LogError(log, "eval failed", out.error());
      return EXIT_FAILURE;
    }

    const bool ok_tz = NearlyEqualRelative(out.value().temperature_k, c.tz, 2e-3, 0.8);
    const bool ok_tinf = NearlyEqualRelative(out.value().exospheric_temp_k, c.tinf, 2e-3, 0.8);
    const bool ok_ro = NearlyEqualRelative(out.value().density_g_cm3, c.ro, 1e-1, 1e-18);
    if (!ok_tz || !ok_tinf || !ok_ro) {
      dtm2020::Log(log,
                   dtm2020::LogLevel::kError,
                   "case alt=" + std::to_string(c.alt_km) + " f30=" + std::to_string(c.f30) +
                       " f30m=" + std::to_string(c.f30m) + " got(tz,tinf,ro)=(" +
                       std::to_string(out.value().temperature_k) + "," +
                       std::to_string(out.value().exospheric_temp_k) + "," +
                       std::to_string(out.value().density_g_cm3) + ") expected=(" +
                       std::to_string(c.tz) + "," + std::to_string(c.tinf) + "," + std::to_string(c.ro) + ")");
      return EXIT_FAILURE;
    }
  }

  return EXIT_SUCCESS;
#else
  return EXIT_SUCCESS;
#endif
}
