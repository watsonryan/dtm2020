#pragma once

// Author: Watson
// Purpose: Lightweight C++20 result type for value-or-error returns.

#include <optional>
#include <utility>

namespace dtm2020 {

template <typename T, typename E>
class Result {
 public:
  static Result Ok(T value) { return Result(std::move(value), std::nullopt); }
  static Result Err(E error) { return Result(std::nullopt, std::move(error)); }

  [[nodiscard]] bool has_value() const { return value_.has_value(); }
  [[nodiscard]] explicit operator bool() const { return has_value(); }

  [[nodiscard]] const T& value() const { return *value_; }
  [[nodiscard]] T& value() { return *value_; }

  [[nodiscard]] const E& error() const { return *error_; }
  [[nodiscard]] E& error() { return *error_; }

 private:
  Result(std::optional<T> value, std::optional<E> error)
      : value_(std::move(value)), error_(std::move(error)) {}

  std::optional<T> value_{};
  std::optional<E> error_{};
};

}  // namespace dtm2020
