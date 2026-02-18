#pragma once

// Author: Watson
// Purpose: Runtime-first coefficient-path resolution for external-data tests.

#include <filesystem>

namespace dtm2020::testutil {

struct CoeffPathConfig {
  std::filesystem::path runtime_override{};
  std::filesystem::path repo_default{};
  std::filesystem::path ci_fallback{};
};

inline std::filesystem::path ResolveCoeffPath(const CoeffPathConfig& config) {
  if (!config.runtime_override.empty() && std::filesystem::exists(config.runtime_override)) {
    return config.runtime_override;
  }
  if (!config.repo_default.empty() && std::filesystem::exists(config.repo_default)) {
    return config.repo_default;
  }
  if (!config.ci_fallback.empty() && std::filesystem::exists(config.ci_fallback)) {
    return config.ci_fallback;
  }
  return {};
}

}  // namespace dtm2020::testutil
