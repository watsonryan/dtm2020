// Author: Watson
// Purpose: Shared error formatting and construction helpers.

#include "dtm2020/dtm2020_operational.hpp"

#include <sstream>
#include <utility>

namespace dtm2020 {

std::string_view ToString(ErrorCode code) {
  switch (code) {
    case ErrorCode::kNone:
      return "none";
    case ErrorCode::kFileOpenFailed:
      return "file_open_failed";
    case ErrorCode::kFileParseFailed:
      return "file_parse_failed";
    case ErrorCode::kInvalidInput:
      return "invalid_input";
  }
  return "unknown";
}

std::string FormatError(const Error& error) {
  std::ostringstream out;
  out << "code=" << ToString(error.code) << " message=\"" << error.message << "\"";
  if (!error.detail.empty()) {
    out << " detail=\"" << error.detail << "\"";
  }
  if (!error.location.empty()) {
    out << " location=\"" << error.location << "\"";
  }
  return out.str();
}

Error MakeError(ErrorCode code, std::string message, std::string detail, std::string location) {
  return Error{code, std::move(message), std::move(detail), std::move(location)};
}

}  // namespace dtm2020
