#ifndef SMPL_SPATIAL_H
#define SMPL_SPATIAL_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace smpl {

// Aliases for the most commonly used types from within Eigen to support brevity
// and, optimistically, ease a possible transition to optional compilation using
// 32-bit floating-point across the board rather than templating the entire
// library

//////////////////
// Matrix Types //
//////////////////

template <
    class Scalar,
    int Rows,
    int Cols,
    int Options = 0,
    int MaxRows = Rows,
    int MaxCols = Cols>
using Matrix = Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>;

using Vector2 = Eigen::Vector2d;
using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;

using RowVector2 = Eigen::RowVector2d;
using RowVector3 = Eigen::RowVector3d;
using RowVector4 = Eigen::RowVector4d;

using Matrix2 = Eigen::Matrix2d;
using Matrix3 = Eigen::Matrix3d;
using Matrix4 = Eigen::Matrix4d;

//////////////////////////////////
// Spatial Transformation Types //
//////////////////////////////////

using Eigen::Affine;
using Eigen::AffineCompact;
using Eigen::Isometry;
using Eigen::Projective;

template <class Scalar, int Dim, int Mode, int Options = 0>
using Transform = Eigen::Transform<Scalar, Dim, Mode, Options>;

using Projective2 = Eigen::Projective2d;
using Affine2 = Eigen::Affine2d;
using Affine2Compact = Eigen::AffineCompact2d;
using Translation2 = Eigen::Translation2d;
using AlignedScaling2 = Eigen::AlignedScaling2d;
using Rotation2D = Eigen::Rotation2Dd;

using Projective3 = Eigen::Projective3d;
using Affine3 = Eigen::Affine3d;
using Affine3Compact = Eigen::AffineCompact3d;
using Translation3 = Eigen::Translation3d;
using AlignedScaling3 = Eigen::AlignedScaling3d;
using AngleAxis = Eigen::AngleAxisd;
using Quaternion = Eigen::Quaterniond;

using Eigen::Scaling;

inline auto MakeAffine2(double x, double y) -> Affine2
{
    return Affine2(Translation2(x, y));
}

inline auto MakeAffine2(double x, double y, double theta) -> Affine2
{
    return Affine2(Translation2(x, y) * Rotation2D(theta));
}

inline auto MakeAffine(double x, double y, double z) -> Affine3
{
    return Affine3(Translation3(x, y, z));
}

inline auto MakeAffine(
    double x, double y, double z,
    double Y)
    -> Affine3
{
    return Translation3(x, y, z) * AngleAxis(Y, Vector3::UnitZ());
}

inline auto MakeAffine(
    double x, double y, double z,
    double Y, double P)
    -> Affine3
{
    return Translation3(x, y, z) *
            AngleAxis(Y, Vector3::UnitZ()) *
            AngleAxis(P, Vector3::UnitY());
}

inline auto MakeAffine(
    double x, double y, double z,
    double Y, double P, double R)
    -> Affine3
{
    return Translation3(x, y, z) *
            AngleAxis(Y, Vector3::UnitZ()) *
            AngleAxis(P, Vector3::UnitY()) *
            AngleAxis(R, Vector3::UnitX());
}

} // namespace smpl

#endif
