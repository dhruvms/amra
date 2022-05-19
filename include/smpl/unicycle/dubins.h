#ifndef SMPL_DUBINS_H
#define SMPL_DUBINS_H

// standard includes
#include <ostream>
#include <vector>

// project includes
#include <smpl/spatial.h>
#include <smpl/unicycle/pose_2d.h>

namespace smpl {

enum class AngleDir { CW, CCW };

struct DubinsMotion
{
    Pose2D      start;  ///< The start (x, y, yaw) pose
    Pose2D      goal;   ///< The goal (x, y, yaw) pose
    double      radius; ///< The radius, in meters, of both turns
    double      arc1;   ///< Length, in radians, of the first turn
    double      arc2;   ///< Length, in radians, of the second turn
    AngleDir    dir1;   ///< The direction of the first turn
    AngleDir    dir2;   ///< The direction of the second turn

    DubinsMotion() = default;

    DubinsMotion(
        const Pose2D& start, const Pose2D& goal, double radius_m,
        double arc1, double arc2,
        AngleDir dir1, AngleDir dir2);

    // Return the point around which the dubins path pivots during the first
    // turn.
    auto pivot1() const -> Vector2;

    // Return the point around which the dubins path pivots during the second
    // turn.
    auto pivot2() const -> Vector2;

    // Return the pose at the start of the straight-line segment.
    auto straight_start() const -> Pose2D;

    // Return the pose at the end of the straight-line segment.
    auto straight_end() const -> Pose2D;

    // Return the time parameter at the start of the dubins path (always 0.0).
    auto t0() const -> double;

    // Return the time parameter at the start of the straight-line segment.
    auto t1() const -> double;

    // Return the time parameter at the end of the straight-line segment.
    auto t2() const -> double;

    // Return the time parameter at the end of the dubins path (always 1.0).
    auto t3() const -> double;

    // Return the linear length of the dubins path.
    auto length() const -> double;

    // Sample the dubins path at a given time.
    auto operator()(double t) const -> Pose2D;
};

int MakeDubinsPaths(
    const Pose2D& start,
    const Pose2D& goal,
    double radius,
    DubinsMotion motions[6]);

} // namespace smpl

#endif
