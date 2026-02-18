// Author: Watson
// Purpose: Guard operational numerics against regressions using frozen vectors.

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "dtm2020/logging.hpp"
#include "dtm2020/dtm2020_operational.hpp"

namespace {

struct Row {
  dtm2020::OperationalInputs in{};
  dtm2020::Outputs out{};
};

bool NearlyEqualRelative(double a, double b, double rel_tol, double abs_floor) {
  const double diff = std::abs(a - b);
  const double scale = std::max(std::abs(a), std::abs(b));
  return diff <= std::max(abs_floor, rel_tol * scale);
}

bool ParseCsv(const std::filesystem::path& path, std::vector<Row>& rows) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return false;
  }
  std::string line;
  std::getline(in, line);
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    std::stringstream ss(line);
    std::string token;
    std::vector<double> vals;
    while (std::getline(ss, token, ',')) {
      vals.push_back(std::stod(token));
    }
    if (vals.size() != 19) {
      return false;
    }

    Row r;
    r.in.altitude_km = vals[0];
    r.in.latitude_deg = vals[1];
    r.in.longitude_deg = vals[2];
    r.in.local_time_h = vals[3];
    r.in.day_of_year = vals[4];
    r.in.f107 = vals[5];
    r.in.f107m = vals[6];
    r.in.kp_delayed_3h = vals[7];
    r.in.kp_mean_24h = vals[8];
    r.out.temperature_k = vals[9];
    r.out.exospheric_temp_k = vals[10];
    r.out.density_g_cm3 = vals[11];
    r.out.mean_mol_mass_g = vals[12];
    r.out.d_h_g_cm3 = vals[13];
    r.out.d_he_g_cm3 = vals[14];
    r.out.d_o_g_cm3 = vals[15];
    r.out.d_n2_g_cm3 = vals[16];
    r.out.d_o2_g_cm3 = vals[17];
    r.out.d_n_g_cm3 = vals[18];
    rows.push_back(r);
  }
  return true;
}

}  // namespace

int main() {
  const auto log = dtm2020::MakeStderrLogSink();
  const auto coeff_file = std::filesystem::path(DTM2020_SOURCE_DIR) / "testdata/operational_regression_coeff.dat";
  const auto csv_file = std::filesystem::path(DTM2020_SOURCE_DIR) / "testdata/operational_regression_vectors.csv";
  if (!std::filesystem::exists(coeff_file) || !std::filesystem::exists(csv_file)) {
    return EXIT_FAILURE;
  }

  auto model = dtm2020::Dtm2020Operational::LoadFromFile(coeff_file);
  if (!model) {
    return EXIT_FAILURE;
  }

  std::vector<Row> rows;
  if (!ParseCsv(csv_file, rows) || rows.empty()) {
    return EXIT_FAILURE;
  }

  std::size_t idx = 0;
  for (const auto& row : rows) {
    const auto got = model.value().Evaluate(row.in);
    if (!got) {
      dtm2020::LogError(log, "evaluate failed at row " + std::to_string(idx), got.error());
      return EXIT_FAILURE;
    }

    if (!NearlyEqualRelative(got.value().temperature_k, row.out.temperature_k, 1e-9, 1e-8) ||
        !NearlyEqualRelative(got.value().exospheric_temp_k, row.out.exospheric_temp_k, 1e-9, 1e-8) ||
        !NearlyEqualRelative(got.value().density_g_cm3, row.out.density_g_cm3, 1e-9, 1e-20) ||
        !NearlyEqualRelative(got.value().mean_mol_mass_g, row.out.mean_mol_mass_g, 1e-9, 1e-12) ||
        !NearlyEqualRelative(got.value().d_h_g_cm3, row.out.d_h_g_cm3, 1e-9, 1e-24) ||
        !NearlyEqualRelative(got.value().d_he_g_cm3, row.out.d_he_g_cm3, 1e-9, 1e-24) ||
        !NearlyEqualRelative(got.value().d_o_g_cm3, row.out.d_o_g_cm3, 1e-9, 1e-24) ||
        !NearlyEqualRelative(got.value().d_n2_g_cm3, row.out.d_n2_g_cm3, 1e-9, 1e-24) ||
        !NearlyEqualRelative(got.value().d_o2_g_cm3, row.out.d_o2_g_cm3, 1e-9, 1e-24) ||
        !NearlyEqualRelative(got.value().d_n_g_cm3, row.out.d_n_g_cm3, 1e-9, 1e-24)) {
      dtm2020::Log(log, dtm2020::LogLevel::kError, "mismatch at row " + std::to_string(idx));
      return EXIT_FAILURE;
    }
    ++idx;
  }

  return EXIT_SUCCESS;
}
