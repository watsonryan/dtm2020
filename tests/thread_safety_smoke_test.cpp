// Author: Watson
// Purpose: Verify thread-safe concurrent Evaluate calls on shared model instances.

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <thread>
#include <vector>

#include "dtm2020/dtm2020_operational.hpp"

namespace {

bool WriteSyntheticCoeffFile(const std::filesystem::path& file_path) {
  std::ofstream out(file_path);
  if (!out.is_open()) {
    return false;
  }

  out << "Synthetic DTM coefficients for thread-safety smoke test\n";
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

bool NearlyEqual(double a, double b) {
  const double diff = std::abs(a - b);
  const double scale = std::max(std::abs(a), std::abs(b));
  return diff <= std::max(1e-20, 1e-12 * scale);
}

}  // namespace

int main() {
  const auto coeff_file = std::filesystem::temp_directory_path() / "dtm2020_coeff_thread_smoke.dat";
  if (!WriteSyntheticCoeffFile(coeff_file)) {
    return EXIT_FAILURE;
  }

  auto model = dtm2020::Dtm2020Operational::LoadFromFile(coeff_file);
  if (!model) {
    return EXIT_FAILURE;
  }

  std::vector<dtm2020::OperationalInputs> inputs;
  inputs.reserve(1024);
  for (int i = 0; i < 1024; ++i) {
    const double x = static_cast<double>(i);
    inputs.push_back(dtm2020::OperationalInputs{
        .altitude_km = 125.0 + std::fmod(0.73 * x, 875.0),
        .latitude_deg = -90.0 + std::fmod(1.37 * x, 180.0),
        .longitude_deg = -180.0 + std::fmod(2.11 * x, 360.0),
        .local_time_h = std::fmod(0.19 * x, 24.0),
        .day_of_year = 1.0 + std::fmod(3.0 * x, 365.0),
        .f107 = 70.0 + std::fmod(0.51 * x, 230.0),
        .f107m = 70.0 + std::fmod(0.47 * x, 230.0),
        .kp_delayed_3h = std::fmod(0.03 * x, 9.0),
        .kp_mean_24h = std::fmod(0.025 * x, 9.0),
    });
  }

  std::vector<dtm2020::Outputs> baseline(inputs.size());
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    auto out = model.value().Evaluate(inputs[i]);
    if (!out) {
      return EXIT_FAILURE;
    }
    baseline[i] = out.value();
  }

  std::atomic<bool> ok{true};
  const unsigned hw = std::max(2U, std::thread::hardware_concurrency());
  std::vector<std::thread> workers;
  workers.reserve(hw);
  for (unsigned tid = 0; tid < hw; ++tid) {
    workers.emplace_back([&, tid]() {
      for (std::size_t i = tid; i < inputs.size(); i += hw) {
        auto out = model.value().Evaluate(inputs[i]);
        if (!out) {
          ok.store(false, std::memory_order_relaxed);
          return;
        }
        if (!NearlyEqual(out.value().temperature_k, baseline[i].temperature_k) ||
            !NearlyEqual(out.value().density_g_cm3, baseline[i].density_g_cm3) ||
            !NearlyEqual(out.value().exospheric_temp_k, baseline[i].exospheric_temp_k)) {
          ok.store(false, std::memory_order_relaxed);
          return;
        }
      }
    });
  }

  for (auto& t : workers) {
    t.join();
  }

  return ok.load(std::memory_order_relaxed) ? EXIT_SUCCESS : EXIT_FAILURE;
}
