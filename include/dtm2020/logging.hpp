#pragma once

// Author: Watson
// Purpose: Lightweight logging helpers for CLI/tools/tests (library remains log-free).

#include <functional>
#include <iostream>
#include <string>
#include <string_view>

#include "dtm2020/dtm2020_operational.hpp"

namespace dtm2020 {

enum class LogLevel {
  kDebug,
  kInfo,
  kWarn,
  kError,
};

[[nodiscard]] inline std::string_view ToString(LogLevel level) {
  switch (level) {
    case LogLevel::kDebug:
      return "debug";
    case LogLevel::kInfo:
      return "info";
    case LogLevel::kWarn:
      return "warn";
    case LogLevel::kError:
      return "error";
  }
  return "unknown";
}

using LogSink = std::function<void(LogLevel, std::string_view)>;

[[nodiscard]] inline LogSink MakeStderrLogSink() {
  return [](LogLevel level, std::string_view message) {
    std::cerr << "[" << ToString(level) << "] " << message << "\n";
  };
}

inline void Log(LogSink sink, LogLevel level, std::string_view message) {
  if (sink) {
    sink(level, message);
  }
}

inline void LogError(LogSink sink, std::string_view context, const Error& error) {
  if (!sink) {
    return;
  }
  std::string line;
  line.reserve(context.size() + 2 + error.message.size() + error.detail.size() + error.location.size() + 64);
  line.append(context.data(), context.size());
  line.append(": ");
  line.append(FormatError(error));
  sink(LogLevel::kError, line);
}

}  // namespace dtm2020
