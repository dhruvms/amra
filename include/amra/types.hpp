#ifndef TYPES_HPP
#define TYPES_HPP

// standard includes
#include <iostream>
#include <vector>

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

class Search
{
public:
	virtual int set_start(int start_id) = 0;
	virtual int set_goal(int goal_id) = 0;
	virtual void set_max_planning_time(double max_planning_time_ms) = 0;
	virtual int get_n_expands() const = 0;
	virtual void reset() = 0;

	virtual int replan(
		std::vector<int>* solution_path, int* solution_cost) = 0;
};

} // namespace AMRA

#endif  // TYPES_HPP
