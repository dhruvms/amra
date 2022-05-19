#include <smpl/unicycle/dubins.h>

// standard includes
#include <math.h>
#include <stdlib.h>
#include <limits>

// project includes
#include <smpl/angles.h>
#include <smpl/spatial.h>

namespace smpl {

struct Circle2d
{
    Vector2 center;
    double  radius;
};

struct DirectionalCircle2d
{
    Circle2d    circle;
    int         direction; // -1 for left, 1 for right

    bool left() const { return direction == -1; }
    bool right() const { return direction == 1; }
};

struct Segment2d
{
    Vector2 a;
    Vector2 b;
};

static
auto signd(double d) -> double
{
    if (d > 0)          return 1.0;
    else if (d < 0.0)   return -1.0;
    else                return 0.0;
}

static
auto interp_angle(double aa, double ab, double t, AngleDir dir) -> double
{
    auto sadist = smpl::shortest_angle_diff(ab, aa);

    double af;
    if ((sadist < 0 && dir == AngleDir::CW) ||
            (sadist > 0 && dir == AngleDir::CCW))
    {
        af = aa + t * sadist;
    } else {
        af = aa + t * -signd(sadist) * (2.0 * M_PI - fabs(sadist));
    }

    return smpl::normalize_angle_positive(af);
}

static
auto rotate(const Vector2& v, double theta) -> Vector2
{
    auto newvx = v.x() * cos(theta) - v.y() * sin(theta);
    auto newvy = v.x() * sin(theta) + v.y() * cos(theta);
    return Vector2(newvx, newvy);
}

static
auto heading(const Vector2& v) -> double
{
    return atan2(v.y(), v.x());
}

static
auto MakeHeadingVector(double th) -> Vector2
{
    return Vector2(cos(th), sin(th));
}

static
auto rotate90cw(const Vector2& v) -> Vector2
{
    return Vector2(v.y(), -v.x());
}

static
auto rotate90ccw(const Vector2& v) -> Vector2
{
    return Vector2(-v.y(), v.x());
}

static
auto dist(const Vector2& u, const Vector2& v) -> double
{
    return (u - v).norm();
}

static
auto dist_sqrd(const Vector2& u, const Vector2& v) -> double
{
    return (u - v).squaredNorm();
}

static
auto ComputeArcLength(
    const DirectionalCircle2d& circle,
    double start_angle,
    double end_angle)
    -> double
{
    auto start = Vector2(
            circle.circle.center.x() + circle.circle.radius * cos(start_angle),
            circle.circle.center.y() + circle.circle.radius * sin(start_angle));
    auto end = Vector2(
            circle.circle.center.x() + circle.circle.radius * cos(end_angle),
            circle.circle.center.y() + circle.circle.radius * sin(end_angle));

    auto v1 = Vector2(start - circle.circle.center);
    auto v2 = Vector2(end - circle.circle.center);
    auto theta = heading(v2) - heading(v1);
    if (theta < 0 && circle.left()) {
        theta += 2 * M_PI;
    } else if (theta > 0 && circle.right()) {
        theta -= 2 * M_PI;
    }
    return fabs(theta * circle.circle.radius);
}

// Return descriptions of the turn-left and turn-right directional circles from
// a pose and turning radius.
static
void ComputeTurningCircles(
    const Pose2D& pose,
    double turning_radius_m,
    DirectionalCircle2d& left_turn,
    DirectionalCircle2d& right_turn)
{
    auto heading = Vector2(cos(pose.theta), sin(pose.theta));
    heading.normalize();
    heading = rotate(heading, M_PI / 2.0);

    left_turn.circle.center = pos(pose) + turning_radius_m * heading;
    left_turn.circle.radius = turning_radius_m;
    left_turn.direction = -1;

    heading = rotate(heading, M_PI);
    right_turn.circle.center = pos(pose) + turning_radius_m * heading;
    right_turn.circle.radius = turning_radius_m;
    right_turn.direction = 1;
}

static
auto ComputeInnerTangent(
    const DirectionalCircle2d& dc1,
    const DirectionalCircle2d& dc2)
    -> Segment2d
{
    auto& c1 = dc1.circle;
    auto& c2 = dc2.circle;

    auto& p1 = c1.center;
    auto& p2 = c2.center;
    auto r1 = c1.radius;
    auto r2 = c2.radius;

    auto v1 = Vector2(p2 - p1);
    auto D = v1.norm();

    // create new circle at the midpoint between two input circles whose
    // circumference touches the centers of both input circle_centers
    auto p3 = Vector2(0.5 * (p1 + p2));
    auto r3 = 0.5 * D;

    auto tangent = Segment2d();

    if (D < r1 + r2) {
        tangent.a = Vector2(
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN());
        tangent.b = Vector2(
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN());
        return tangent;
    }

    auto& p4 = p1;
    auto r4 = r1 + r2;

    // Find the angle v1 would need to be rotated to point towards the upper tangent point
    auto gamma = acos(r4 / (2*r3)); // r3*r3 = r3*r3 + r4*r4 - 2*r3*r4*cos(gamma) from law of cosines

    auto v2 = Vector2(v1.normalized());
    v2 = rotate(v2, gamma);

    auto upper_inner_tangent = Segment2d();
    auto lower_inner_tangent = Segment2d();

    // Find tangent point on the first circle
    auto pt = Vector2(p1 + r4 * v2);
    upper_inner_tangent.a = p1 + r1 * v2;

    // Find the tangent point on the second circle
    auto v4 = Vector2(p2 - pt);
    upper_inner_tangent.b = upper_inner_tangent.a + v4;

    // Repeat the last couple steps to find the lower inner tangent
    v2 = v1.normalized();
    v2 = rotate(v2, -gamma);
    auto lpt = Vector2(p1 + r4 * v2);
    lower_inner_tangent.a = p1 + r1 * v2;
    v4 = p2 - lpt;
    lower_inner_tangent.b = lower_inner_tangent.a + v4;

    if (dc1.right()) {
        tangent = upper_inner_tangent;
    } else {
        tangent = lower_inner_tangent;
    }

    return tangent;
}

static
auto ComputeOuterTangent(
    const DirectionalCircle2d& dc1,
    const DirectionalCircle2d& dc2)
    -> Segment2d
{
    auto& c1 = dc1.circle;
    auto& c2 = dc2.circle;

    auto& p1 = c1.radius > c2.radius ? c1.center : c2.center;
    auto& p2 = c1.radius > c2.radius ? c2.center : c1.center;
    auto r1 = c1.radius > c2.radius ? c1.radius : c2.radius;
    auto r2 = c1.radius > c2.radius ? c2.radius : c1.radius;

    auto v1 = Vector2(p2 - p1);

    auto D = v1.norm();

    auto p3 = Vector2(0.5 * (p1 + p1));
    auto r3 = 0.5 * D;

    auto& p4 = p1;
    auto r4 = r1 - r2;

    auto gamma = acos(r4 / (2 * r3));

    auto v2 = Vector2(v1.normalized());
    v2 = rotate(v2, gamma);

    auto upper_outer_tangent = Segment2d();
    auto lower_outer_tangent = Segment2d();

    auto pt = Vector2(p1 + r4 * v2);
    upper_outer_tangent.a = p1 + r1 * v2;

    auto v4 = Vector2(p2 - pt);
    upper_outer_tangent.b = upper_outer_tangent.a + v4;

    v2 = v1.normalized();
    v2 = rotate(v2, -gamma);
    auto lpt = Vector2(p1 + r4 * v2);
    lower_outer_tangent.a = p1 + r1 * v2;
    v4 = p2 - lpt;
    lower_outer_tangent.b = lower_outer_tangent.a + v4;

    auto tangent = Segment2d();

    if (dc1.right()) {
        tangent = c1.radius > c2.radius ? upper_outer_tangent : lower_outer_tangent;
    } else {
        tangent = c1.radius > c2.radius ? lower_outer_tangent : upper_outer_tangent;
    }

    if (c1.radius <= c2.radius) {
        std::swap(tangent.a, tangent.b);
    }

    return tangent;
}

static
auto ConstructRRPath(
    const Pose2D& start,
    const Pose2D& goal,
    double radius,
    const DirectionalCircle2d& start_circle,
    const DirectionalCircle2d& goal_circle,
    const Segment2d& tangent)
    -> DubinsMotion
{
    auto rr_path = DubinsMotion();
    rr_path.start = start;
    rr_path.goal = goal;
    rr_path.radius = radius;
    rr_path.dir1 = AngleDir::CW;
    rr_path.dir2 = AngleDir::CW;

    auto v1 = Vector2(pos(start) - start_circle.circle.center);
    auto start_angle = heading(v1);

    auto v2 = Vector2(tangent.a - start_circle.circle.center);
    auto end_angle = heading(v2);

    rr_path.arc1 =
            ComputeArcLength(start_circle, start_angle, end_angle) / radius;

    v1 = tangent.b - goal_circle.circle.center;
    start_angle = heading(v1);

    v2 = pos(goal) - goal_circle.circle.center;
    end_angle = heading(v2);

    rr_path.arc2 =
            ComputeArcLength(goal_circle, start_angle, end_angle) / radius;
    return rr_path;
}

static
auto ConstructLLPath(
    const Pose2D& start,
    const Pose2D& goal,
    double radius,
    const DirectionalCircle2d& start_circle,
    const DirectionalCircle2d& goal_circle,
    const Segment2d& tangent)
    -> DubinsMotion
{
    auto ll_path = DubinsMotion();
    ll_path.start = start;
    ll_path.goal = goal;
    ll_path.radius = radius;
    ll_path.dir1 = AngleDir::CCW;
    ll_path.dir2 = AngleDir::CCW;

    auto v1 = Vector2(pos(start) - start_circle.circle.center);
    auto start_angle = heading(v1);

    auto v2 = Vector2(tangent.a - start_circle.circle.center);
    auto end_angle = heading(v2);

    ll_path.arc1 =
            ComputeArcLength(start_circle, start_angle, end_angle) / radius;

    v1 = tangent.b - goal_circle.circle.center;
    start_angle = heading(v1);

    v2 = pos(goal) - goal_circle.circle.center;
    end_angle = heading(v2);

    ll_path.arc2 =
            ComputeArcLength(goal_circle, start_angle, end_angle) / radius;
    return ll_path;
}

static
auto ConstructLRPath(
    const Pose2D& start,
    const Pose2D& goal,
    double radius,
    const DirectionalCircle2d& start_circle,
    const DirectionalCircle2d& goal_circle,
    const Segment2d& tangent)
    -> DubinsMotion
{
    auto lr_path = DubinsMotion();
    lr_path.start = start;
    lr_path.goal = goal;
    lr_path.radius = radius;
    lr_path.dir1 = AngleDir::CCW;
    lr_path.dir2 = AngleDir::CW;

    auto v1 = Vector2(pos(start) - start_circle.circle.center);
    auto start_angle = heading(v1);

    auto v2 = Vector2(tangent.a - start_circle.circle.center);
    auto end_angle = heading(v2);

    lr_path.arc1 =
            ComputeArcLength(start_circle, start_angle, end_angle) / radius;

    v1 = tangent.b - goal_circle.circle.center;
    start_angle = heading(v1);

    v2 = pos(goal) - goal_circle.circle.center;
    end_angle = heading(v2);

    lr_path.arc2 =
            ComputeArcLength(goal_circle, start_angle, end_angle) / radius;
    return lr_path;
}

static
auto ConstructRLPath(
    const Pose2D& start,
    const Pose2D& goal,
    double radius,
    const DirectionalCircle2d& start_circle,
    const DirectionalCircle2d& goal_circle,
    const Segment2d& tangent)
    -> DubinsMotion
{
    auto rl_path = DubinsMotion();
    rl_path.start = start;
    rl_path.goal = goal;
    rl_path.radius = radius;
    rl_path.dir1 = AngleDir::CW;
    rl_path.dir2 = AngleDir::CCW;

    auto v1 = Vector2(pos(start) - start_circle.circle.center);
    auto start_angle = heading(v1);

    auto v2 = Vector2(tangent.a - start_circle.circle.center);
    auto end_angle = heading(v2);

    rl_path.arc1 =
            ComputeArcLength(start_circle, start_angle, end_angle) / radius;

    v1 = tangent.b - goal_circle.circle.center;
    start_angle = heading(v1);

    v2 = pos(goal) - goal_circle.circle.center;
    end_angle = heading(v2);

    rl_path.arc2 =
            ComputeArcLength(goal_circle, start_angle, end_angle) / radius;
    return rl_path;
}

static
auto ConstructRLRPath(
    const Pose2D& start,
    const Pose2D& goal,
    double radius,
    const DirectionalCircle2d& start_circle,
    const DirectionalCircle2d& goal_circle,
    const Segment2d& tangent)
    -> DubinsMotion
{
#if 0
    // check for RLR trajectory
    const Circle2d& c1 = start_circle.circle;
    const Circle2d& c2 = goal_circle.circle;

    Vector2 v1 = c2.center - c1.center;
    double D = v1.norm();
    double gamma = acos(D / (4 * radius));
    double theta = heading(v1) + gamma;
    Vector2 v2(1.0, 0.0);// = v1;
    v2.normalize();

    rotate(v2, theta); //gamma); //theta);

    Vector2 p3 = c1.center + 2 * radius * v2;
    Vector2 pt1 = p3 + (c1.center - p3).normalized() * radius;
    Vector2 pt2 = p3 + (c2.center - p3).normalized() * radius;

    v1 = start.pos - c1.center;
    double start_angle1 = heading(v1);
    v1 = pt1 - c1.center;
    double end_angle1 = heading(v1);
    std::vector<Pose2D> turn1 = GenerateTurnPath(
            start_circle,
            start_angle1,
            end_angle1,
            interp_res);

    v1 = pt1 - p3;
    double start_angle2 = heading(v1);
    v1 = pt2 - p3;
    double end_angle2 = heading(v1);
    DirectionalCircle2d midturncircle = { { p3, radius }, -1 };
    std::vector<Pose2D> turn2 = GenerateTurnPath(
            midturncircle,
            start_angle2,
            end_angle2,
            interp_res);

    v1 = pt2 - c2.center;
    double start_angle3 = heading(v1);
    v1 = goal.pos - c2.center;
    double end_angle3 = heading(v1);
    std::vector<Pose2D> turn3 = GenerateTurnPath(
            goal_circle,
            start_angle3,
            end_angle3,
            interp_res);

    turn1.insert(turn1.end(), turn2.begin(), turn2.end());
    turn1.insert(turn1.end(), turn3.begin(), turn3.end());

    paths.push_back(turn1);
#else
    return DubinsMotion();
#endif
}

static
auto ConstructLRLPath(
    const Pose2D& start,
    const Pose2D& goal,
    double radius,
    const DirectionalCircle2d& start_circle,
    const DirectionalCircle2d& goal_circle,
    const Segment2d& tangent)
    -> DubinsMotion
{
#if 0
    // check for LRL trajectory
    const Circle2d& c1 = start_circle.circle;
    const Circle2d& c2 = goal_circle.circle;

    VectorVector2r - c1.center;
    double D = v1.norm();
    double gamma = acos(D / (4 * radius));
    double theta = heading(v1) + gamma;
    Vector2 v2 = v1;
    v2.normalize();

    rotate(v2, -gamma); //theta);

    Vector2 p3 = c1.center + 2 * radius * v2;
    Vector2 pt1 = p3 + (c1.center - p3).normalized() * radius;
    Vector2 pt2 = p3 + (c2.center - p3).normalized() * radius;

    v1 = start.pos - c1.center;
    double start_angle1 = heading(v1);
    v1 = pt1 - c1.center;
    double end_angle1 = heading(v1);
    std::vector<Pose2D> turn1 = GenerateTurnPath(
            start_circle,
            start_angle1,
            end_angle1,
            interp_res);

    v1 = pt1 - p3;
    double start_angle2 = heading(v1);
    v1 = pt2 - p3;
    double end_angle2 = heading(v1);
    DirectionalCircle2d midturncircle = { { p3, radius }, 1 };
    std::vector<Pose2D> turn2 = GenerateTurnPath(
            midturncircle,
            start_angle2,
            end_angle2,
            interp_res);

    v1 = pt2 - c2.center;
    double start_angle3 = heading(v1);
    v1 = goal.pos - c2.center;
    double end_angle3 = heading(v1);
    std::vector<Pose2D> turn3 = GenerateTurnPath(
            goal_circle,
            start_angle3,
            end_angle3,
            interp_res);

    turn1.insert(turn1.end(), turn2.begin(), turn2.end());
    turn1.insert(turn1.end(), turn3.begin(), turn3.end());

    paths.push_back(turn1);
#else
    return DubinsMotion();
#endif
}

static
auto GenerateTurnPath(
    const DirectionalCircle2d& circle,
    double start_angle,
    double end_angle,
    double interp_res)
    -> std::vector<Pose2D>
{
    auto path = std::vector<Pose2D>();
    auto angle = start_angle;
    path.push_back({
            circle.circle.center.x() + circle.circle.radius * cos(start_angle),
            circle.circle.center.y() + circle.circle.radius * sin(start_angle),
            circle.right() ? start_angle - M_PI / 2.0 : start_angle + M_PI / 2.0
    });

    auto arc_length = ComputeArcLength(circle, start_angle, end_angle);

    auto num_interm_points = (int)floor(arc_length / interp_res);

    path.reserve(num_interm_points + 2);

    auto angle_res = interp_res / circle.circle.radius;
    if (circle.right()) {
        // clockwise
        for (auto i = 0; i < num_interm_points; ++i) {
            angle -= angle_res;
            path.push_back({
                    circle.circle.center.x() + circle.circle.radius * cos(angle),
                    circle.circle.center.y() + circle.circle.radius * sin(angle),
                    angle - M_PI / 2.0
            });
        }
    } else {
        // counter-clockwise
        for (auto i = 0; i < num_interm_points; ++i) {
            angle += angle_res;
            path.push_back({
                    circle.circle.center.x() + circle.circle.radius * cos(angle),
                    circle.circle.center.y() + circle.circle.radius * sin(angle),
                    angle + M_PI / 2.0
            });
        }
    }

    path.push_back({
            circle.circle.center.x() + circle.circle.radius * cos(end_angle),
            circle.circle.center.y() + circle.circle.radius * sin(end_angle),
            circle.right() ? end_angle - M_PI / 2.0 : end_angle + M_PI / 2.0
    });

    return path;
}

static
auto GenerateStraightPath(
    const Vector2& start,
    const Vector2& end,
    double interp_res)
    -> std::vector<Pose2D>
{
    auto v1 = Vector2(end - start);
    auto angle = heading(v1);
    return {
        Pose2D{ start.x(),  start.y(),  angle },
        Pose2D{ end.x(),    end.y(),    angle },
    };
}

///////////////
// Interface //
///////////////

DubinsMotion::DubinsMotion(
    const Pose2D& start,
    const Pose2D& goal,
    double radius,
    double arc1,
    double arc2,
    AngleDir dir1,
    AngleDir dir2)
{
    this->start = start;
    this->goal = goal;
    this->radius = radius;
    this->arc1 = arc1;
    this->arc2 = arc2;
    this->dir1 = dir1;
    this->dir2 = dir2;
}

auto DubinsMotion::pivot1() const -> Vector2
{
    auto hv = MakeHeadingVector(start.theta);
    if (dir1 == AngleDir::CW) {
        hv = rotate90cw(hv);
    } else if (dir1 == AngleDir::CCW) {
        hv = rotate90ccw(hv);
    }

    return pos(start) + hv * radius;
}

auto DubinsMotion::pivot2() const -> Vector2
{
    auto hv = MakeHeadingVector(goal.theta);
    if (dir2 == AngleDir::CW) {
        hv = rotate90cw(hv);
    } else if (dir2 == AngleDir::CCW) {
        hv = rotate90ccw(hv);
    }
    return pos(goal) + hv * radius;
}

auto DubinsMotion::straight_start() const -> Pose2D
{
    auto piv1 = pivot1();
    auto h2s = Vector2(pos(start) - piv1);
    auto dir = (dir1 == AngleDir::CCW ? 1.0 : -1.0);
    return Pose2D(piv1 + rotate(h2s, dir * arc1), start.theta + dir * arc1);
}

auto DubinsMotion::straight_end() const -> Pose2D
{
    auto piv2 = pivot2();
    auto h2g = Vector2(pos(goal) - piv2);
    auto dir = (dir2 == AngleDir::CCW ? 1.0 : -1.0);
    return Pose2D(piv2 + rotate(h2g, -dir * arc2), goal.theta - dir * arc2);
}

auto DubinsMotion::t0() const -> double
{
    return 0.0;
}

auto DubinsMotion::t1() const -> double
{
    auto len = length();
    if (len == 0.0) {
        return 0.0;
    }
    auto arc1len = arc1 * radius;
    return arc1len / len;
}

auto DubinsMotion::t2() const -> double
{
    auto len = length();
    if (len == 0.0) {
        return 0.0;
    }
    auto arc2len = arc2 * radius;
    return (len - arc2len) / len;
}

auto DubinsMotion::t3() const -> double
{
    return 1.0;
}

auto DubinsMotion::length() const -> double
{
    return arc1 * radius +
            dist(pos(straight_start()), pos(straight_end())) +
            arc2 * radius;
}

static
auto interp(Pose2D src, Pose2D dst, double a) -> Pose2D
{
    return Pose2D{
        (1.0 - a) * src.x + a * dst.x,
        (1.0 - a) * src.y + a * dst.y,
        src.theta + a * shortest_angle_diff(dst.theta, src.theta)
    };
}

auto DubinsMotion::operator()(double t) const -> Pose2D
{
    if (t <= 0.0) {
        return start;
    }

    if (t >= 1.0) {
        return goal;
    }

    auto t_1 = t1();
    auto t_2 = t2();
    auto piv1 = pivot1();
    auto piv2 = pivot2();

    if (t < t_1) {
        // on start turn
        auto a = t / t_1;
        auto h2s = Vector2(pos(start) - piv1);
        auto dir = (dir1 == AngleDir::CCW ? 1.0 : -1.0);
        return Pose2D(piv1 + rotate(h2s, dir * a * arc1),
                interp_angle(start.theta, start.theta + dir * arc1, a, dir1));
    } else if (t > t_2) {
        // on goal turn
        auto a = (t - t_2) / (1.0 - t_2);
        auto h2g = Vector2(pos(goal) - piv2);
        auto dir = (dir2 == AngleDir::CCW ? 1.0 : -1.0);
        return Pose2D(piv2 + rotate(h2g, -dir * (1.0 - a) * arc2),
                interp_angle(goal.theta - dir * arc2, goal.theta, a, dir2));
    } else {
        // on straight segment
        auto a = (t - t_1) / (t_2 - t_1);
        return Pose2D(interp(straight_start(), straight_end(), a));
    }
}

// TODO: convert from returning vector of solutions to fixed array + number
// of valid solutions
int MakeDubinsPaths(
    const Pose2D& start,
    const Pose2D& goal,
    double radius,
    DubinsMotion motions[6])
{
    // compute both pairs of turning circles
    auto start_circle_l = DirectionalCircle2d();
    auto start_circle_r = DirectionalCircle2d();
    auto goal_circle_l = DirectionalCircle2d();
    auto goal_circle_r = DirectionalCircle2d();
    ComputeTurningCircles(start, radius, start_circle_l, start_circle_r);
    ComputeTurningCircles(goal, radius, goal_circle_l, goal_circle_r);

    // TODO: variant that returns geometric debug information (turning circles)

    auto ll_tangent = ComputeOuterTangent(start_circle_l, goal_circle_l);
    auto rr_tangent = ComputeOuterTangent(start_circle_r, goal_circle_r);
    auto lr_tangent = ComputeInnerTangent(start_circle_l, goal_circle_r);
    auto rl_tangent = ComputeInnerTangent(start_circle_r, goal_circle_l);

    auto count = 0;

    if (rr_tangent.a.x() == rr_tangent.a.x()) {
        motions[count++] = ConstructRRPath(
                start, goal, radius, start_circle_r, goal_circle_r, rr_tangent);
    }

    if (ll_tangent.a.x() == ll_tangent.a.x()) {
        motions[count++] = ConstructLLPath(
                start, goal, radius, start_circle_l, goal_circle_l, ll_tangent);
    }

    if (lr_tangent.a.x() == lr_tangent.a.x()) {
        motions[count++] = ConstructLRPath(
                start, goal, radius, start_circle_l, goal_circle_r, lr_tangent);
    }

    if (rl_tangent.a.x() == rl_tangent.a.x()) {
        motions[count++] = ConstructRLPath(
                start, goal, radius, start_circle_r, goal_circle_l, rl_tangent);
    }

//    if ((start_circle_l.circle.center - goal_circle_l.circle.center).norm() < 4 * radius) {
//        motions[count++] = ConstructLRLPath();
//    }
//
//    if ((start_circle_r.circle.center - goal_circle_r.circle.center).norm() < 4 * radius) {
//        motions[count++] = ConstructRLRPath();
//    }

    return count;
}

} // namespace smpl
