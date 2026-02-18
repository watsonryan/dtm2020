#pragma once

// Author: Watson
// Purpose: Lightweight C++20 result type for value-or-error returns.

#include <cassert>
#include <utility>
#include <variant>

namespace dtm2020 {

/**
 * @brief Lightweight C++20 result container that stores either a value or an error.
 * @tparam T Success payload type.
 * @tparam E Error payload type.
 */
template <typename T, typename E>
class Result {
 public:
  /**
   * @brief Construct a successful result.
   * @param value Success value.
   */
  static Result Ok(T value) { return Result(std::move(value)); }

  /**
   * @brief Construct an error result.
   * @param error Error payload.
   */
  static Result Err(E error) { return Result(std::move(error)); }

  /**
   * @brief Check whether this result contains a success value.
   */
  [[nodiscard]] bool has_value() const { return std::holds_alternative<T>(state_); }

  /**
   * @brief Boolean conversion equivalent to has_value().
   */
  [[nodiscard]] explicit operator bool() const { return has_value(); }

  /**
   * @brief Access the success value.
   * @pre has_value() is true.
   */
  [[nodiscard]] const T& value() const {
    assert(has_value());
    return std::get<T>(state_);
  }

  /**
   * @brief Access the mutable success value.
   * @pre has_value() is true.
   */
  [[nodiscard]] T& value() {
    assert(has_value());
    return std::get<T>(state_);
  }

  /**
   * @brief Access the error payload.
   * @pre has_value() is false.
   */
  [[nodiscard]] const E& error() const {
    assert(!has_value());
    return std::get<E>(state_);
  }

  /**
   * @brief Access the mutable error payload.
   * @pre has_value() is false.
   */
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
