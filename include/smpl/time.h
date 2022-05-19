////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#ifndef SMPL_TIME_H
#define SMPL_TIME_H

// standard includes
#include <time.h>
#include <cstdint>
#include <chrono>
#include <ratio>

// define clock APIs
#define SMPL_CLOCK_ROS_TIME                 0
#define SMPL_CLOCK_ROS_WALLTIME             1
#define SMPL_CLOCK_C                        2
#define SMPL_CLOCK_CHRONO_HIGH_RESOLUTION   3

// select the clock API
#ifndef SMPL_CLOCK_API
#define SMPL_CLOCK_API SMPL_CLOCK_C
#endif

#define SMPL_CLOCK_IS_ROS \
    (SMPL_CLOCK_API == SMPL_CLOCK_ROS_TIME || \
    SMPL_CLOCK_API == SMPL_CLOCK_ROS_WALLTIME)

#if SMPL_CLOCK_IS_ROS
#include <ros/time.h>
#endif

namespace smpl {

// define chrono interface to ros clocks
#if SMPL_CLOCK_IS_ROS
template <class RosTime, class Derived>
class ros_clock_base
{
public:

    typedef std::int64_t rep;
    typedef std::nano period;

    typedef std::chrono::duration<rep, period> duration;

    // duration template arg explicitly required here to prevent name lookup
    // into unknown Derived class at the point of declaration
    typedef std::chrono::time_point<Derived, duration> time_point;

    static constexpr bool is_steady() { return false; }

    static time_point now()
    {
        RosTime n = RosTime::now();
        duration d(1000000000L * (std::int64_t)n.sec + (std::int64_t)n.nsec);
        return time_point(d);
    }
};

class ros_clock : public ros_clock_base<::ros::Time, ros_clock> { };
class ros_wall_clock : public ros_clock_base<::ros::WallTime, ros_wall_clock> { };
#endif

// define chrono interface to c clock
#if SMPL_CLOCK_API == SMPL_CLOCK_C
class c_clock
{
public:

    typedef ::clock_t rep;
    typedef std::ratio<1, CLOCKS_PER_SEC> period;

    typedef std::chrono::duration<rep, period> duration;

    typedef std::chrono::time_point<c_clock> time_point;

    static constexpr bool is_steady() { return true; }

    static time_point now() { return time_point(duration(::clock())); }
};
#endif

// declare selected clock type
#if SMPL_CLOCK_API == SMPL_CLOCK_ROS_TIME
typedef ros_clock clock;
#elif SMPL_CLOCK_API == SMPL_CLOCK_ROS_WALLTIME
typedef ros_wall_clock clock;
#elif SMPL_CLOCK_API == SMPL_CLOCK_C
typedef c_clock clock;
#elif SMPL_CLOCK_API == SMPL_CLOCK_CHRONO_HIGH_RESOLUTION
typedef ::std::chrono::high_resolution_clock clock;
#else
#error "Unrecognized clock API configuration"
#endif

template <class Rep, class Period>
double to_seconds(const std::chrono::duration<Rep, Period>& d)
{
    return std::chrono::duration_cast<std::chrono::duration<double>>(d).count();
}

inline clock::duration to_duration(double seconds)
{
    return std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(seconds));
}

} // namespace smpl

#endif
