#ifndef SNOWCLEAR_LOGGING_HPP
#define SNOWCLEAR_LOGGING_HPP

// =====================================================================
// Logging — the core's only I/O dependency
// ---------------------------------------------------------------------
// The algorithm sources used to call ROS_INFO / ROS_WARN / ROS_ERROR
// directly, which tied every module to a ROS 1 installation. They now call
// SC_INFO / SC_WARN / SC_ERROR / SC_DEBUG, routed through a process-wide
// sink:
//
//   default      printf-style to stdout (Info/Debug) and stderr (Warn/Error)
//   ROS 2 node   RCLCPP_* on the node's logger, installed via set_log_sink()
//   tests        capture into a buffer to assert on diagnostics
//
// The macros keep the original printf formatting, so every existing message
// string is preserved verbatim — including the ones the Python tooling parses.
// =====================================================================

#include <cstdarg>
#include <functional>
#include <string>

namespace snowclear {

enum class LogLevel { kDebug, kInfo, kWarn, kError };

// Sink signature: (level, fully formatted message, no trailing newline).
using LogSink = std::function<void(LogLevel, const std::string&)>;

// Install a sink. Passing an empty function restores the default sink.
// Thread-safe with respect to log_message().
void set_log_sink(LogSink sink);

// printf-style logging. Prefer the macros below.
void log_message(LogLevel level, const char* fmt, ...)
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((format(printf, 2, 3)))
#endif
    ;

}  // namespace snowclear

#define SC_DEBUG(...) ::snowclear::log_message(::snowclear::LogLevel::kDebug, __VA_ARGS__)
#define SC_INFO(...)  ::snowclear::log_message(::snowclear::LogLevel::kInfo, __VA_ARGS__)
#define SC_WARN(...)  ::snowclear::log_message(::snowclear::LogLevel::kWarn, __VA_ARGS__)
#define SC_ERROR(...) ::snowclear::log_message(::snowclear::LogLevel::kError, __VA_ARGS__)

#endif  // SNOWCLEAR_LOGGING_HPP
