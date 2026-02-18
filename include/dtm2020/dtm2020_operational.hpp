#pragma once

// Author: Watson
// Purpose: Public API for DTM2020 operational model evaluation.

#include <array>
#include <filesystem>
#include <optional>
#include <string>

namespace dtm2020 {

enum class ErrorCode {
  kNone,
  kFileOpenFailed,
  kFileParseFailed,
  kInvalidInput,
};

struct Error {
  ErrorCode code{ErrorCode::kNone};
  std::string message{};
};

struct OperationalInputs {
  double altitude_km{};
  double latitude_deg{};
  double longitude_deg{};
  double local_time_h{};
  double day_of_year{};
  double f107{};
  double f107m{};
  double kp_delayed_3h{};
  double kp_mean_24h{};
};

struct Outputs {
  double temperature_k{};
  double exospheric_temp_k{};
  double density_g_cm3{};
  double mean_mol_mass_g{};
  double d_h_g_cm3{};
  double d_he_g_cm3{};
  double d_o_g_cm3{};
  double d_n2_g_cm3{};
  double d_o2_g_cm3{};
  double d_n_g_cm3{};
};

class Dtm2020Operational {
 public:
  struct Options {
    bool emulate_mcm_transition{true};
  };

  static std::optional<Dtm2020Operational> LoadFromFile(
      const std::filesystem::path& coeff_file,
      Error& error);

  static std::optional<Dtm2020Operational> LoadFromFile(
      const std::filesystem::path& coeff_file,
      Error& error,
      Options options);

  Outputs Evaluate(const OperationalInputs& in, Error& error) const;
  float DensityUncertaintyPercent(const OperationalInputs& in) const;

  struct Coefficients {
    static constexpr int kNlatm = 96;
    std::array<float, kNlatm> tt{};
    std::array<float, kNlatm> h{};
    std::array<float, kNlatm> he{};
    std::array<float, kNlatm> o{};
    std::array<float, kNlatm> az2{};
    std::array<float, kNlatm> o2{};
    std::array<float, kNlatm> az{};
    std::array<float, kNlatm> t0{};
    std::array<float, kNlatm> tp{};
  };

 private:
  explicit Dtm2020Operational(Coefficients coeffs, Options options)
      : coeffs_(coeffs), options_(options) {}

  Coefficients coeffs_{};
  Options options_{};
};

}  // namespace dtm2020
