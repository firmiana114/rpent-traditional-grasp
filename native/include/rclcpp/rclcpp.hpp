// Minimal rclcpp compatibility layer for the standalone TRAC-IK build.
#ifndef RPENT_RCLCPP_COMPAT_HPP
#define RPENT_RCLCPP_COMPAT_HPP

#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

namespace rclcpp
{

class Logger
{
};

inline Logger get_logger(const std::string&)
{
  return Logger();
}

class Duration
{
public:
  explicit Duration(double seconds) : seconds_(seconds) {}
  double seconds() const { return seconds_; }

private:
  double seconds_;
};

class Time
{
public:
  Time() : value_(std::chrono::steady_clock::now()) {}
  explicit Time(std::chrono::steady_clock::time_point value) : value_(value) {}

  friend Duration operator-(const Time& first, const Time& second)
  {
    return Duration(std::chrono::duration<double>(first.value_ - second.value_).count());
  }

private:
  std::chrono::steady_clock::time_point value_;
};

class Clock
{
public:
  Time now() const { return Time(std::chrono::steady_clock::now()); }
};

class Node
{
public:
  using SharedPtr = std::shared_ptr<Node>;
  Logger get_logger() const { return Logger(); }
  bool has_parameter(const std::string&) const { return false; }
  std::string declare_parameter(const std::string&, const std::string& value)
  {
    return value;
  }
  void get_parameter(const std::string&, std::string&) const {}
};

inline void logf(const char* level, const char* message)
{
  std::fprintf(stderr, "[trac_ik][%s] %s\n", level, message);
}

template<typename First, typename... Args>
void logf(const char* level, const char* format, First first, Args... args)
{
  std::fprintf(stderr, "[trac_ik][%s] ", level);
  std::fprintf(stderr, format, first, args...);
  std::fputc('\n', stderr);
}

}  // namespace rclcpp

#define RCLCPP_FATAL(logger, ...) rclcpp::logf("FATAL", __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) rclcpp::logf("ERROR", __VA_ARGS__)
#define RCLCPP_WARN_THROTTLE(logger, clock, duration, ...) \
  rclcpp::logf("WARN", __VA_ARGS__)
#define RCLCPP_ERROR_THROTTLE(logger, clock, duration, ...) \
  rclcpp::logf("ERROR", __VA_ARGS__)
#define RCLCPP_DEBUG(logger, ...) do {} while (0)
#define RCLCPP_DEBUG_STREAM(logger, expression) do {} while (0)
#define RCLCPP_FATAL_STREAM(logger, expression) \
  do { std::cerr << "[trac_ik][FATAL] " << expression << std::endl; } while (0)

#endif
