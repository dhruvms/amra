#ifndef SMPL_CONSOLE_H
#define SMPL_CONSOLE_H

#ifdef SMPL_CONSOLE_ROS
#include "detail/console_ros.h"
#else
#include "detail/console_std.h"
#endif

#endif
