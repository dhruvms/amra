#include <smpl/unicycle/unicycle.h>

// project includes
#include <smpl/angles.h>
#include <smpl/spatial.h>

namespace smpl {

// from http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257, specific to 2x2 matrices
static
inline auto pinv(const Matrix2& a, double eps) -> Matrix2
{
    // see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method

    // SVD
    Eigen::JacobiSVD<Matrix2> svdA(
            a,
            Eigen::ComputeFullU | Eigen::ComputeFullV);
    Vector2 vSingular = svdA.singularValues();

    // Build a diagonal matrix with the Inverted Singular values
    // The pseudo inverted singular matrix is easy to compute :
    // is formed by replacing every nonzero entry by its reciprocal (inversing).
    Vector2 vPseudoInvertedSingular(svdA.matrixV().cols(), 1);

    for (int iRow = 0; iRow < vSingular.rows(); iRow++) {
        if (std::fabs(vSingular(iRow)) <= eps) { // TODO : Put epsilon in parameter
            vPseudoInvertedSingular(iRow, 0) = 0.0;
        } else {
            vPseudoInvertedSingular(iRow, 0) = 1.0 / vSingular(iRow);
        }
    }

    // A little optimization here
    Matrix2 mAdjointU = svdA.matrixU().adjoint().block(
            0, 0, vSingular.rows(), svdA.matrixU().adjoint().cols());

    // Pseudo-Inversion : V * S * U'
    return (svdA.matrixV() * vPseudoInvertedSingular.asDiagonal()) * mAdjointU;
}

static inline bool AlmostEquals(double lhs, double rhs, double eps)
{
    return std::fabs(lhs - rhs) < eps;
}

auto UnicycleMotion::operator()(double t) const -> Pose2D
{
    return at(t);
}

auto UnicycleMotion::length() const -> double
{
    auto arc_len = r * std::fabs(goal.theta - start.theta); //w * (1.0 - tl);
    return std::fabs(l) + std::fabs(arc_len);
}

bool UnicycleMotion::is_valid() const { return valid; }

auto UnicycleMotion::at(double t) const -> Pose2D
{
    if (t <= tl) {
        auto x = start.x + v * t * std::cos(start.theta);
        auto y = start.y + v * t * std::sin(start.theta);
        auto theta = start.theta;
        return Pose2D{ x, y, theta };
    } else {
        auto x =
                start.x
                + l * std::cos(start.theta)
                + r * std::sin(w * (t - tl) + start.theta)
                - r * std::sin(start.theta);
        auto y =
                start.y
                + l * std::sin(start.theta)
                - r * std::cos(w * (t - tl) + start.theta)
                + r * std::cos(start.theta);
        auto theta = start.theta + w * (t - tl);
        return Pose2D{ x, y, theta };
    }
}

auto MakeUnicycleMotion(
    double start_x, double start_y, double start_theta,
    double goal_x, double goal_y, double goal_theta,
    double eps)
    -> UnicycleMotion
{
    UnicycleMotion motion;

    motion.start = Pose2D{ start_x, start_y, start_theta };
    motion.goal = Pose2D{ goal_x, goal_y, goal_theta };

    motion.valid = true;

    auto dx = goal_x - start_x;
    auto dy = goal_y - start_y;
    auto dtheta = angles::shortest_angle_diff(goal_theta, start_theta);

    auto sstheta = std::sin(start_theta);
    auto cstheta = std::cos(start_theta);
    auto sgtheta = std::sin(goal_theta);
    auto cgtheta = std::cos(goal_theta);

    Matrix2 R;
    R(0, 0) = cstheta;
    R(0, 1) = -sstheta + sgtheta;
    R(1, 0) = sstheta;
    R(1, 1) = cstheta - cgtheta;

    Matrix2 Rpinv = pinv(R, eps);

    Vector2 S = Rpinv * Vector2(dx, dy);

    motion.l = S(0);
    motion.r = S(1);

    auto lmag = std::fabs(motion.l);
    auto rmag = std::fabs(motion.r);

    if (rmag < eps) {
        motion.r = 0.0;
        motion.w = std::numeric_limits<double>::infinity();
        motion.v = motion.l;
        motion.tl = 1.0;

        if (std::fabs(dtheta) < eps &&
            angles::shortest_angle_dist(start_theta, std::atan2(dy, dx)) < eps)
        {
            // no rotation and linear motion is headed towards the goal
            motion.valid = true;
        } else {
            // can't reach the goal position/rotation, or requires final
            // turn-in-place
            motion.valid = false;
        }

        return motion;
    }

    // Rearrange these to obtain an expression for w in terms of l, r, and dtheta
    //   w = (goal_theta - start_theta) / (1 - tl)
    //   t_l = l / v
    //   r = v / w
    // the angular velocity w should have the same sign as the radius.
    // TODO: What is the meaning behind |l| instead of just l here? It seems to
    // work.
    if (motion.r * dtheta < 0.0) {
        motion.w = std::copysign(2.0 * M_PI - std::fabs(dtheta), motion.r) + (lmag / motion.r);
    } else {
        motion.w = std::copysign(dtheta, motion.r) + (lmag / motion.r);
    }

    motion.v = std::copysign(motion.r * motion.w, motion.l);

    motion.tl = motion.l / motion.v; // time to arc segment

    return motion;
}

auto MakeUnicycleMotion(const Pose2D& start, const Pose2D& goal, double eps)
    -> UnicycleMotion
{
    return MakeUnicycleMotion(
            start.x, start.y, start.theta,
            goal.x, goal.y, goal.theta,
            eps);
}

} // namespace smpl
