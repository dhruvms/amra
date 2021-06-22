#ifndef TYPES_HPP
#define TYPES_HPP

// standard includes
#include <iostream>
#include <vector>
#include <map>
#include <memory>

namespace AMRA
{

typedef int* MAP_t;

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
	std::vector<int> coord;
	std::vector<double> state;
	Resolution::Level level;
};
typedef std::map<int, std::vector<MapState*> > EXPANDS_t;

inline
bool operator==(const MapState& a, const MapState& b)
{
	return (a.coord == b.coord);
}

inline
std::ostream& operator<<(std::ostream& out, const MapState& s)
{
	for (size_t i = 0; i < s.coord.size(); ++i)
	{
		out << s.coord.at(i);
		if (i < s.coord.size()-1) {
			out << ", ";
		}
	}
	out << std::endl;
	return out;
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

class Environment
{
public:
	virtual void CreateSearch() = 0;
	virtual bool Plan(bool save=false) = 0;

	virtual void GetSuccs(
		int state_id,
		Resolution::Level level,
		std::vector<int>* succs,
		std::vector<unsigned int>* costs) = 0;
	virtual bool IsGoal(const int& id) = 0;

	virtual void SaveExpansions(
		int iter, double w1, double w2,
		const std::vector<int>& curr_solution) = 0;

	int GetStartID() const { return m_start_id; };
	int GetGoalID() const { return m_goal_id; };

	virtual void GetStart(MapState& start) = 0;
	virtual void GetGoal(MapState& goal) = 0;
	virtual void GetStateFromID(const int& id, MapState& state) = 0;

	virtual Resolution::Level GetResLevel(const int& state_id) = 0;

protected:
	std::unique_ptr<Search> m_search;

	int m_start_id, m_goal_id, m_expansions = 0;
};

} // namespace AMRA

#endif  // TYPES_HPP
