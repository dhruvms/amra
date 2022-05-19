#ifndef SMPL_UNICYCLE_H
#define SMPL_UNICYCLE_H

#include <cmath>
#include <cstdlib>
#include <limits>

#include <smpl/unicycle/pose_2d.h>

namespace smpl {

struct UnicycleMotion
{
    Pose2D start = Pose2D{ 0.0, 0.0, 0.0 };
    Pose2D goal = Pose2D{ 0.0, 0.0, 0.0 };

    // determined by geometry alone
    double l = 0.0; // length of straight line segment
    double r = 0.0; // turning radius

    // derived from l and r, normalized so that 0 <= t <= 1
    double w = 0.0; // angular velocity
    double v = 0.0; // linear velocity

    // time to end of the straight line segment
    double tl = 0.0;

    // whether the solver could find a valid solution
    //
    // invalid solutions occur when either:
    // (1) the start and goal heading are the same but the goal position is not
    //     inline with the start heading
    // (2) the start and goal heading are different but the goal position is
    //     inline with the start heading so that a turn of radius 0
    //     (turn-in-place) is required to meet the final orientation
    bool valid = false;

    auto operator()(double t) const -> Pose2D;
    auto length() const -> double;
    bool is_valid() const;
    auto at(double t) const -> Pose2D;
};

auto MakeUnicycleMotion(
    double start_x, double start_y, double start_theta,
    double goal_x, double goal_y, double goal_theta,
    double eps = std::numeric_limits<double>::epsilon())
    -> UnicycleMotion;

auto MakeUnicycleMotion(
    const Pose2D& start,
    const Pose2D& goal,
    double eps = std::numeric_limits<double>::epsilon())
    -> UnicycleMotion;

} // namespace smpl

#endif
