#ifndef HELPERS_HPP
#define HELPERS_HPP

// project includes

// system includes

// standard includes
#include <sstream>

namespace AMRA
{

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (X*YSIZE + Y)

void reset(std::stringstream& ss)
{
	ss.str("");
	ss.clear();
}

}  // namespace AMRA

#endif  // HELPERS_HPP
