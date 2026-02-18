// Author: Watson
// Purpose: Smoke test for research-model evaluation path.

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>

#include "dtm2020/dtm2020_research.hpp"

namespace {

bool WriteSyntheticCoeffFile(const std::filesystem::path& file_path) {
  std::ofstream out(file_path);
  if (!out.is_open()) {
    return false;
  }

  out << "Synthetic DTM research coefficients for smoke test\n";
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

    out << std::setw(4) << i << " " << std::uppercase << std::scientific << std::setprecision(6)
        << tt << " " << 0.0F << " "
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

}  // namespace

int main() {
#ifdef DTM2020_ENABLE_RESEARCH
  const auto coeff_file = std::filesystem::temp_directory_path() / "dtm2020_coeff_research_smoke.dat";
  if (!WriteSyntheticCoeffFile(coeff_file)) {
    return EXIT_FAILURE;
  }

  dtm2020::Error error;
  auto model = dtm2020::Dtm2020Research::LoadFromFile(coeff_file, error);
  if (!model || error.code != dtm2020::ErrorCode::kNone) {
    return EXIT_FAILURE;
  }

  dtm2020::ResearchInputs in{};
  in.altitude_km = 220.0;
  in.latitude_deg = 10.0;
  in.longitude_deg = 25.0;
  in.local_time_h = 5.0;
  in.day_of_year = 123.0;
  in.f30 = 130.0;
  in.f30m = 120.0;
  in.ap60 = {6.0, 5.0, 5.5, 5.8, 6.1, 5.2, 5.1, 5.0, 4.9, 4.8};

  auto out = model->Evaluate(in, error);
  if (error.code != dtm2020::ErrorCode::kNone) {
    return EXIT_FAILURE;
  }

  if (!std::isfinite(out.temperature_k) || !std::isfinite(out.density_g_cm3) || out.density_g_cm3 <= 0.0) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
#else
  return EXIT_SUCCESS;
#endif
}
