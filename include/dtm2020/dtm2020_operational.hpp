#pragma once

// Author: Watson
// Purpose: Public API for DTM2020 operational model evaluation.

#include <array>
#include <filesystem>
#include <string>

#include "dtm2020/result.hpp"

namespace dtm2020 {

/**
 * @brief Error categories returned by model loading and evaluation.
 */
enum class ErrorCode {
  kNone,
  kFileOpenFailed,
  kFileParseFailed,
  kInvalidInput,
};

/**
 * @brief Error payload with machine-readable code and human-readable message.
 */
struct Error {
  ErrorCode code{ErrorCode::kNone};
  std::string message{};
};

/**
 * @brief Inputs for the operational DTM2020 model (F10.7/Kp forcing).
 */
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

/**
 * @brief Thermospheric outputs returned by DTM2020.
 */
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

/**
 * @brief Operational DTM2020 evaluator backed by a parsed coefficient table.
 */
class Dtm2020Operational {
 public:
  /**
   * @brief Loader options that control behavioral compatibility.
   */
  struct Options {
    bool emulate_mcm_transition{true};
  };

  /**
   * @brief Load operational coefficients from a file.
   * @param coeff_file Path to the DTM2020 operational coefficient file.
   * @return Loaded model on success, or an Error on failure.
   */
  [[nodiscard]] static Result<Dtm2020Operational, Error> LoadFromFile(
      const std::filesystem::path& coeff_file);

  /**
   * @brief Load operational coefficients from a file with explicit options.
   * @param coeff_file Path to the DTM2020 operational coefficient file.
   * @param options Loader/runtime compatibility options.
   * @return Loaded model on success, or an Error on failure.
   */
  [[nodiscard]] static Result<Dtm2020Operational, Error> LoadFromFile(
      const std::filesystem::path& coeff_file,
      Options options);

  /**
   * @brief Evaluate the model for one geophysical state vector.
   * @param in Operational model inputs.
   * @return Outputs on success, or an Error when inputs are invalid or evaluation fails.
   */
  [[nodiscard]] Result<Outputs, Error> Evaluate(const OperationalInputs& in) const;

  /**
   * @brief Estimate 1-sigma density uncertainty in percent for the given inputs.
   * @param in Operational model inputs.
   * @return Estimated uncertainty percentage.
   */
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
