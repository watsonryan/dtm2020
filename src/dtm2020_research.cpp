// Author: Watson
// Purpose: Optional research model scaffold with explicit not-yet-implemented status.

#include "dtm2020/dtm2020_research.hpp"

namespace dtm2020 {

Dtm2020Research Dtm2020Research::CreateDefault() {
  return Dtm2020Research{};
}

Outputs Dtm2020Research::Evaluate(const ResearchInputs&, Error& error) const {
  error = {ErrorCode::kInvalidInput,
           "Research model not yet implemented (dtm5/Hp60 path pending)"};
  return {};
}

}  // namespace dtm2020
