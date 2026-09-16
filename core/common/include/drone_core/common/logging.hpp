#pragma once

#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>

// The autonomy core is deliberately free of any middleware dependency, so it
// cannot reach for a ROS logger. These macros give the core a minimal, always
// available diagnostic channel. By default they write to std::cerr exactly as
// before, so the core standalone (and every unit test) behaves unchanged and
// stays middleware-free.
//
// A host application may install a sink to redirect these diagnostics — the ROS
// node forwards them to RCLCPP (and thus /rosout) so planner/trajgen lines land
// in the flight recording. The sink is pure C++ (a std::function); the core
// never learns ROS exists. Install it ONCE at startup, before the worker/control
// threads spin, so the single read the macros do never races the write.
namespace drone_core {

enum class LogLevel { Info, Error };

using LogSink = std::function<void(LogLevel, const std::string&)>;

namespace detail {
inline void cerrSink(LogLevel level, const std::string& msg) {
  std::cerr << (level == LogLevel::Error ? "[drone_core][error] " : "[drone_core][info] ")
            << msg << std::endl;
}
}  // namespace detail

// The one sink instance (function-local static ⇒ one across all translation
// units). Default: the historical std::cerr behaviour.
inline LogSink& logSink() {
  static LogSink sink = &detail::cerrSink;
  return sink;
}

// Redirect core diagnostics to `sink`. Passing a default-constructed (empty)
// std::function restores std::cerr. Call once at startup (see the note above).
inline void setLogSink(LogSink sink) {
  logSink() = sink ? std::move(sink) : LogSink(&detail::cerrSink);
}

}  // namespace drone_core

#define DRONE_LOG_INFO(msg)                                                     \
  do {                                                                          \
    std::ostringstream _drone_log_os;                                           \
    _drone_log_os << msg;                                                       \
    ::drone_core::logSink()(::drone_core::LogLevel::Info, _drone_log_os.str()); \
  } while (0)

#define DRONE_LOG_ERROR(msg)                                                     \
  do {                                                                           \
    std::ostringstream _drone_log_os;                                            \
    _drone_log_os << msg;                                                        \
    ::drone_core::logSink()(::drone_core::LogLevel::Error, _drone_log_os.str()); \
  } while (0)
