////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

#ifndef SMPL_ANGLES_H
#define SMPL_ANGLES_H

// standard includes
#include <cmath>

// system includes
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace smpl {

/// Convert an angle specified in radians to degrees.
constexpr double to_degrees(double rads)
{
    return rads * 180.0 / M_PI;
}

/// Convert an angle specified in degrees to radians.
constexpr double to_radians(double degs)
{
    return degs * M_PI / 180.0;
}

/// Normalize an angle into the range [-pi, pi].
inline double normalize_angle(double angle)
{
    // normalize to [-2*pi, 2*pi] range
    if (std::fabs(angle) > 2.0 * M_PI) {
        angle = std::fmod(angle, 2.0 * M_PI);
    }

    if (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    return angle;
}

inline double normalize_angle_positive(double angle)
{
    angle = normalize_angle(angle);
    if (angle < 0.0) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

/// Return the shortest signed difference between two angles.
inline double shortest_angle_diff(double af, double ai)
{
    return normalize_angle(af - ai);
}

/// Return the shortest distance between two angles.
inline double shortest_angle_dist(double af, double ai)
{
    return std::fabs(shortest_angle_diff(af, ai));
}

inline double minor_arc_diff(double af, double ai)
{
    return shortest_angle_diff(af, ai);
}

inline double major_arc_diff(double af, double ai)
{
    double diff = shortest_angle_diff(af, ai);
    return -1.0 * std::copysign(1.0, diff) * (2.0 * M_PI - std::fabs(diff));
}

inline double minor_arc_dist(double af, double ai)
{
    return std::fabs(minor_arc_diff(af, ai));
}

inline double major_arc_dist(double af, double ai)
{
    return std::fabs(major_arc_diff(af, ai));
}

/// Return the closest angle equivalent to af that is numerically greater than
/// ai.
inline double unwind(double ai, double af)
{
    //2.0 * M_PI * std::floor((af - ai) / (2.0 * M_PI));
    af = std::remainder(af - ai, 2.0 * M_PI);
    if (af < ai) {
        af += 2.0 * M_PI;
    }
    return af;
}

template <typename T>
void get_euler_zyx(const Eigen::Matrix<T, 3, 3>& rot, T& y, T& p, T& r)
{
    y = std::atan2(rot(1, 0), rot(0, 0));
    p  = std::atan2(-rot(2, 0), std::sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2)));
    r = std::atan2(rot(2, 1), rot(2, 2));
}

template <typename T>
void get_euler_zyx(const Eigen::Quaternion<T>& rot, T& y, T& p, T& r)
{
    Eigen::Matrix<T, 3, 3> R(rot);
    get_euler_zyx(R, y, p, r);
}

template <typename T>
void from_euler_zyx(T y, T p, T r, Eigen::Matrix<T, 3, 3>& rot)
{
    rot = Eigen::AngleAxis<T>(y, Eigen::Matrix<T, 3, 1>::UnitZ()) *
        Eigen::AngleAxis<T>(p, Eigen::Matrix<T, 3, 1>::UnitY()) *
        Eigen::AngleAxis<T>(r, Eigen::Matrix<T, 3, 1>::UnitX());
}

template <typename T>
void from_euler_zyx(T y, T p, T r, Eigen::Quaternion<T>& q)
{
    Eigen::Matrix<T, 3, 3> R;
    from_euler_zyx(y, p, r, R);
    q = Eigen::Quaternion<T>(R);
}

template <typename T>
void normalize_euler_zyx(T& y, T& p, T& r)
{
    Eigen::Matrix<T, 3, 3> rot;
    from_euler_zyx(y, p, r, rot);
    get_euler_zyx(rot, y, p, r);
}

template <typename T>
void normalize_euler_zyx(T* angles) // in order r, p, y
{
    Eigen::Matrix<T, 3, 3> rot;
    from_euler_zyx(angles[2], angles[1], angles[0], rot);
    get_euler_zyx(rot, angles[2], angles[1], angles[0]);
}

template <typename T>
auto get_nearest_planar_rotation(const Eigen::Quaternion<T>& q) -> T
{
    auto s_squared = 1.0 - (q.w() * q.w());
    // end the long chain of copypasta here that seems to have originated from
    // BULLET
    if (s_squared < 10.0 * std::numeric_limits<T>::epsilon()) {
        return 0.0;
    } else {
        double s = 1.0 / sqrt(s_squared);
        return (2.0 * acos(q.w())) * (q.z() * s);
    }
}

namespace angles { // keep this around for compatibility

constexpr double to_degrees(double rads) { return ::smpl::to_degrees(rads); }
constexpr double to_radians(double degs) { return ::smpl::to_radians(degs); }

inline double normalize_angle(double angle)
{
    return ::smpl::normalize_angle(angle);
}

inline double normalize_angle_positive(double angle)
{
    return ::smpl::normalize_angle_positive(angle);
}

inline double shortest_angle_diff(double af, double ai)
{
    return ::smpl::shortest_angle_diff(af, ai);
}

inline double shortest_angle_dist(double af, double ai)
{
    return ::smpl::shortest_angle_dist(af, ai);
}

inline double minor_arc_diff(double af, double ai)
{
    return ::smpl::minor_arc_diff(af, ai);
}

inline double major_arc_diff(double af, double ai)
{
    return ::smpl::major_arc_diff(af, ai);
}

inline double minor_arc_dist(double af, double ai)
{
    return ::smpl::minor_arc_dist(af, ai);
}

inline double major_arc_dist(double af, double ai)
{
    return ::smpl::major_arc_dist(af, ai);
}

inline double unwind(double ai, double af)
{
    return ::smpl::unwind(ai, af);
}

template <typename T>
void get_euler_zyx(const Eigen::Matrix<T, 3, 3>& rot, T& y, T& p, T& r)
{
    return ::smpl::get_euler_zyx(rot, y, p, r);
}

template <typename T>
void get_euler_zyx(const Eigen::Quaternion<T>& rot, T& y, T& p, T& r)
{
    return ::smpl::get_euler_zyx(rot, y, p, r);
}

template <typename T>
void from_euler_zyx(T y, T p, T r, Eigen::Matrix<T, 3, 3>& rot)
{
    return ::smpl::from_euler_zyx(y, p, r, rot);
}

template <typename T>
void from_euler_zyx(T y, T p, T r, Eigen::Quaternion<T>& q)
{
    return ::smpl::from_euler_zyx(y, p, r, q);
}

template <typename T>
void normalize_euler_zyx(T& y, T& p, T& r)
{
    return ::smpl::normalize_euler_zyx(y, p, r);
}

template <typename T>
void normalize_euler_zyx(T* angles)
{
    return ::smpl::normalize_euler_zyx(angles);
}

template <typename T>
T get_nearest_planar_rotation(const Eigen::Quaternion<T>& q)
{
    return ::smpl::get_nearest_planar_rotation(q);
}

} // namespace angles

} // namespace smpl

#endif

