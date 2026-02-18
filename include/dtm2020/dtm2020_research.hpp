#pragma once

// Author: Watson
// Purpose: Optional research model API scaffold (F30/Hp60 path).

#include <array>
#include <filesystem>

#include "dtm2020/dtm2020_operational.hpp"

namespace dtm2020 {

/**
 * @brief Inputs for the research DTM2020 model (F30/ap60 forcing).
 */
struct ResearchInputs {
  double altitude_km{};
  double latitude_deg{};
  double longitude_deg{};
  double local_time_h{};
  double day_of_year{};
  double f30{};
  double f30m{};
  std::array<double, 10> ap60{};
};

/**
 * @brief Research DTM2020 evaluator for the F30/Hp60 driver path.
 */
class Dtm2020Research {
 public:
  /**
   * @brief Load research coefficients from a file.
   * @param coeff_file Path to the DTM2020 research coefficient file.
   * @return Loaded model on success, or an Error on failure.
   */
  [[nodiscard]] static Result<Dtm2020Research, Error> LoadFromFile(
      const std::filesystem::path& coeff_file);

  /**
   * @brief Evaluate the research model for one geophysical state vector.
   * @param in Research model inputs.
   * @return Outputs on success, or an Error when inputs are invalid or evaluation fails.
   */
  [[nodiscard]] Result<Outputs, Error> Evaluate(const ResearchInputs& in) const;

  struct Coefficients {
    static constexpr int kNlatm = 96;
    // 1-based coefficient layout; index 0 is intentionally unused for Fortran parity.
    std::array<float, kNlatm + 1> tt{};
    std::array<float, kNlatm + 1> h{};
    std::array<float, kNlatm + 1> he{};
    std::array<float, kNlatm + 1> o{};
    std::array<float, kNlatm + 1> az2{};
    std::array<float, kNlatm + 1> o2{};
    std::array<float, kNlatm + 1> az{};
    std::array<float, kNlatm + 1> t0{};
    std::array<float, kNlatm + 1> tp{};
  };

 private:
  explicit Dtm2020Research(Coefficients coeffs) : coeffs_(coeffs) {}
  Coefficients coeffs_{};
};

}  // namespace dtm2020
