#pragma once

// Author: Watson
// Purpose: Lightweight C++20 result type for value-or-error returns.

#include <cassert>
#include <utility>
#include <variant>

namespace dtm2020 {

template <typename T, typename E>
class Result {
 public:
  static Result Ok(T value) { return Result(std::move(value)); }
  static Result Err(E error) { return Result(std::move(error)); }

  [[nodiscard]] bool has_value() const { return std::holds_alternative<T>(state_); }
  [[nodiscard]] explicit operator bool() const { return has_value(); }

  [[nodiscard]] const T& value() const {
    assert(has_value());
    return std::get<T>(state_);
  }
  [[nodiscard]] T& value() {
    assert(has_value());
    return std::get<T>(state_);
  }

  [[nodiscard]] const E& error() const {
    assert(!has_value());
    return std::get<E>(state_);
  }
  [[nodiscard]] E& error() {
    assert(!has_value());
    return std::get<E>(state_);
  }

 private:
  explicit Result(T value) : state_(std::move(value)) {}
  explicit Result(E error) : state_(std::move(error)) {}

  std::variant<T, E> state_;
};

}  // namespace dtm2020
