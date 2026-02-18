#pragma once

// Author: Watson
// Purpose: Optional research model API scaffold (F30/Hp60 path).

#include <array>
#include <filesystem>
#include <optional>

#include "dtm2020/dtm2020_operational.hpp"

namespace dtm2020 {

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

class Dtm2020Research {
 public:
  static std::optional<Dtm2020Research> LoadFromFile(
      const std::filesystem::path& coeff_file,
      Error& error);

  Outputs Evaluate(const ResearchInputs& in, Error& error) const;

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
  explicit Dtm2020Research(Coefficients coeffs) : coeffs_(coeffs) {}
  Coefficients coeffs_{};
};

}  // namespace dtm2020
