#ifndef HELPERS_HPP
#define HELPERS_HPP

// project includes

// system includes

// standard includes
#include <sstream>

namespace AMRA
{

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

}  // namespace AMRA

#endif  // HELPERS_HPP
