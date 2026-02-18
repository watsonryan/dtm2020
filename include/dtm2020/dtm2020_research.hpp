#pragma once

// Author: Watson
// Purpose: Optional research model API scaffold (F30/Hp60 path).

#include <array>

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
  static Dtm2020Research CreateDefault();

  Outputs Evaluate(const ResearchInputs& in, Error& error) const;
};

}  // namespace dtm2020
