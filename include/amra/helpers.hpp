#ifndef HELPERS_HPP
#define HELPERS_HPP

// project includes

// system includes
#include <smpl/time.h>

// standard includes
#include <sstream>
#include <sys/stat.h>
#include <chrono>
#include <cmath>

namespace AMRA
{

static double GetTime()
{
	using namespace smpl;
	return to_seconds(clock::now().time_since_epoch());
}

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (X*YSIZE + Y)

inline
void reset(std::stringstream& ss)
{
	ss.str("");
	ss.clear();
}

template <typename T>
inline
int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

const int DEFAULT_NUM_ANGLES = 12;

inline double DiscToContTheta(int theta, int numAngles) {
	double delta = 2 * M_PI / numAngles;
	return theta * delta;
}

inline double DiscToContTheta(int theta) {
 	return DiscToContTheta(theta, DEFAULT_NUM_ANGLES);
}

inline
bool FileExists(const std::string& filename)
{
	struct stat buf;
	if (stat(filename.c_str(), &buf) != -1)
	{
		return true;
	}
	return false;
}

}  // namespace AMRA

#endif  // HELPERS_HPP
