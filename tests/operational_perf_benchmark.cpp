// Author: Watson
// Purpose: Benchmark operational model throughput with deterministic input corpus.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

#include "dtm2020/dtm2020_operational.hpp"

namespace {

bool WriteSyntheticCoeffFile(const std::filesystem::path& file_path) {
  std::ofstream out(file_path);
  if (!out.is_open()) {
    return false;
  }

  out << "Synthetic DTM coefficients for performance benchmark\n";
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

std::vector<dtm2020::OperationalInputs> MakeCorpus() {
  std::vector<dtm2020::OperationalInputs> corpus;
  corpus.reserve(4096);
  for (int i = 0; i < 4096; ++i) {
    const double x = static_cast<double>(i);
    dtm2020::OperationalInputs in{};
    in.altitude_km = 125.0 + std::fmod(0.73 * x, 875.0);
    in.latitude_deg = -90.0 + std::fmod(1.37 * x, 180.0);
    in.longitude_deg = -180.0 + std::fmod(2.11 * x, 360.0);
    in.local_time_h = std::fmod(0.19 * x, 24.0);
    in.day_of_year = 1.0 + std::fmod(3.0 * x, 365.0);
    in.f107 = 70.0 + std::fmod(0.51 * x, 230.0);
    in.f107m = 70.0 + std::fmod(0.47 * x, 230.0);
    in.kp_delayed_3h = std::fmod(0.03 * x, 9.0);
    in.kp_mean_24h = std::fmod(0.025 * x, 9.0);
    corpus.push_back(in);
  }
  return corpus;
}

}  // namespace

int main() {
  const auto coeff_file = std::filesystem::temp_directory_path() / "dtm2020_coeff_perf_benchmark.dat";
  if (!WriteSyntheticCoeffFile(coeff_file)) {
    std::cerr << "failed to write synthetic coefficient file\n";
    return EXIT_FAILURE;
  }

  auto model = dtm2020::Dtm2020Operational::LoadFromFile(coeff_file);
  if (!model) {
    std::cerr << "load failed: " << model.error().message << "\n";
    return EXIT_FAILURE;
  }

  const auto corpus = MakeCorpus();
  constexpr int kSamples = 15;
  constexpr int kIterations = 80;

  volatile double sink = 0.0;
  for (int i = 0; i < 2; ++i) {
    for (const auto& in : corpus) {
      const auto out = model.value().Evaluate(in);
      if (!out) {
        std::cerr << "warmup evaluate failed: " << out.error().message << "\n";
        return EXIT_FAILURE;
      }
      sink += out.value().density_g_cm3;
    }
  }

  std::vector<double> ns_per_eval;
  ns_per_eval.reserve(kSamples);
  for (int s = 0; s < kSamples; ++s) {
    const auto t0 = std::chrono::steady_clock::now();
    for (int it = 0; it < kIterations; ++it) {
      for (const auto& in : corpus) {
        const auto out = model.value().Evaluate(in);
        if (!out) {
          std::cerr << "evaluate failed: " << out.error().message << "\n";
          return EXIT_FAILURE;
        }
        sink += out.value().temperature_k * 1e-6;
      }
    }
    const auto t1 = std::chrono::steady_clock::now();
    const auto elapsed_ns =
        static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
    const double eval_count = static_cast<double>(kIterations) * static_cast<double>(corpus.size());
    ns_per_eval.push_back(elapsed_ns / eval_count);
  }

  std::sort(ns_per_eval.begin(), ns_per_eval.end());
  const double median = ns_per_eval[ns_per_eval.size() / 2];
  const std::size_t p95_idx =
      static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(ns_per_eval.size()))) - 1;
  const double p95 = ns_per_eval[std::min(p95_idx, ns_per_eval.size() - 1)];
  const double avg =
      std::accumulate(ns_per_eval.begin(), ns_per_eval.end(), 0.0) / static_cast<double>(ns_per_eval.size());

  std::cout << std::fixed << std::setprecision(2)
            << "operational_perf benchmark\n"
            << "samples=" << kSamples << " corpus_size=" << corpus.size() << " iterations=" << kIterations << "\n"
            << "ns_per_eval_avg=" << avg << "\n"
            << "ns_per_eval_median=" << median << "\n"
            << "ns_per_eval_p95=" << p95 << "\n"
            << "sink=" << sink << "\n";
  return EXIT_SUCCESS;
}
