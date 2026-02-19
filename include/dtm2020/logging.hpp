/**
 * @file logging.hpp
 * @brief Lightweight logging helpers for tools/tests around the log-free library core.
 */
#pragma once

// Author: Watson
// Purpose: Lightweight logging helpers for CLI/tools/tests (library remains log-free).

#include <functional>
#include <iostream>
#include <string>
#include <string_view>

#include "dtm2020/dtm2020_operational.hpp"

namespace dtm2020 {

/** @brief Log severity levels. */
enum class LogLevel {
  kDebug,
  kInfo,
  kWarn,
  kError,
};

/** @brief Convert log level enum to lowercase token text. */
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

/** @brief Callback signature for log sinks. */
using LogSink = std::function<void(LogLevel, std::string_view)>;

/** @brief Default sink that writes log lines to stderr. */
[[nodiscard]] inline LogSink MakeStderrLogSink() {
  return [](LogLevel level, std::string_view message) {
    std::cerr << "[" << ToString(level) << "] " << message << "\n";
  };
}

/** @brief Emit a log message if a sink is configured. */
inline void Log(LogSink sink, LogLevel level, std::string_view message) {
  if (sink) {
    sink(level, message);
  }
}

/** @brief Emit a formatted `Error` with context at error severity. */
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
