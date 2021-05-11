#ifndef TYPES_HPP
#define TYPES_HPP

// standard includes
#include <iostream>

namespace AMRA
{

typedef int* Map_t;

struct Resolution
{
	enum Level
	{
		Invalid = -1,
		//  reserved for ANCHOR = 0;
		ANCHOR = 0,
		HIGH = 1,
		MID = 2,
		LOW = 3
	};
};

struct MapState
{
	int d1, d2;
	Resolution::Level level;
};

inline
bool operator==(const MapState& a, const MapState& b)
{
	return (
		a.d1 == b.d1 &&
		a.d2 == b.d2
	);
}

inline
std::ostream& operator<<(std::ostream& out, const MapState& state)
{
	return out << state.d1 << ',' << state.d2 << std::endl;
}

} // namespace AMRA

#endif  // TYPES_HPP
