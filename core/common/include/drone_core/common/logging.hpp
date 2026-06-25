#pragma once

#include <iostream>

// The autonomy core is deliberately free of any middleware dependency, so it
// cannot reach for a ROS logger. These macros give the core a minimal, always
// available diagnostic channel that a host application can redirect by
// replacing the stream if it ever needs structured logging.
#define DRONE_LOG_INFO(msg)  (std::cerr << "[drone_core][info] " << msg << std::endl)
#define DRONE_LOG_ERROR(msg) (std::cerr << "[drone_core][error] " << msg << std::endl)
