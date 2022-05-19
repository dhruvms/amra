#ifndef SMPL_POSE_2D_H
#define SMPL_POSE_2D_H

// system includes
#include <smpl/spatial.h>

namespace smpl {

struct Pose2D
{
    union {
        double data[3];

        struct {
            double x;
            double y;
            double theta;
        };
    };

    Pose2D() = default;

    Pose2D(double x, double y, double theta)
    {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }

    Pose2D(const Vector2& v, double theta)
    {
        this->x = v.x();
        this->y = v.y();
        this->theta = theta;
    }
};

inline auto pos(Pose2D pose) -> Vector2
{
    return Vector2(pose.x, pose.y);
}

} // namespace smpl

#endif

