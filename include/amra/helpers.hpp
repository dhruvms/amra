#ifndef HELPERS_HPP
#define HELPERS_HPP

// project includes

// system includes
#include <smpl/angles.h>

// standard includes
#include <sstream>

namespace AMRA
{

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (X*YSIZE + Y)
#define CONTXY2DISC(X, CELLSIZE) (int)std::round((X / CELLSIZE))

inline
void reset(std::stringstream& ss)
{
	ss.str("");
	ss.clear();
}

//------------------------------------------------------------------------------
// From smpl/angles.h:
// namespace smpl {
//
//   Convert an angle specified in radians to degrees:
//     constexpr double to_degrees(double rads);
//
//   Convert an angle specified in degrees to radians:
//     constexpr double to_radians(double degs);
//
//   Normalize an angle into the range [-pi, pi]:
//     inline double normalize_angle(double angle);
//
//   Normalize an angle into the range [0, pi]:
//     inline double normalize_angle_positive(double angle);
//
//   Return the shortest signed difference between two angles:
//     inline double shortest_angle_diff(double af, double ai);
//
//   Return the shortest distance between two angles:
//     inline double shortest_angle_dist(double af, double ai);
//
//     inline double minor_arc_diff(double af, double ai);
//     inline double major_arc_diff(double af, double ai);
//     inline double minor_arc_dist(double af, double ai);
//     inline double major_arc_dist(double af, double ai);
//
//   Return the closest angle equivalent to af that is numerically greater
//   than ai:
//     inline double unwind(double ai, double af);
// }
//------------------------------------------------------------------------------

/// Motion primitives resources/mprim/mhi_3m_9m.mprim are calculated for 12 angles.
const int DEFAULT_NUM_ANGLES = 12;

// converts continuous (radians) version of angle into discrete
// maps 0->0, [delta/2, 3/2*delta)->1, [3/2*delta, 5/2*delta)->2, ...
inline int ContToDiscTheta(double theta, int numAngles) {
	double delta = 2 * M_PI / numAngles;
	return (int)(smpl::normalize_angle_positive(theta + delta / 2) / delta);
}

inline int ContToDiscTheta(double theta) {
 	return ContToDiscTheta(theta, DEFAULT_NUM_ANGLES);
}

inline double DiscToContTheta(int theta, int numAngles) {
	double delta = 2 * M_PI / numAngles;
	return theta * delta;
}

inline double DiscToContTheta(int theta) {
 	return DiscToContTheta(theta, DEFAULT_NUM_ANGLES);
}
}  // namespace AMRA

#endif  // HELPERS_HPP
