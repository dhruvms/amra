// project includes
#include <amra/heuristic.hpp>
#include <amra/constants.hpp>

// system includes

// standard includes
#include <cmath>
#include <cassert>

namespace AMRA
{

unsigned int EuclideanDist::GetGoalHeuristic(int state_id)
{
	MapState state, goal;
	m_space->GetGoal(goal);
	m_space->GetStateFromID(state_id, state);

	assert(state.coord.size() == goal.coord.size());

	double dist = 0.0;
	for (size_t i = 0; i < state.coord.size(); ++i) {
		dist += std::pow(state.coord.at(i) - goal.coord.at(i), 2);
	}
	dist = std::sqrt(dist);
	return (dist * COST_MULT);
}

unsigned int EuclideanDist::GetStartHeuristic(int state_id)
{
	MapState state, start;
	m_space->GetStart(start);
	m_space->GetStateFromID(state_id, state);

	assert(state.coord.size() == start.coord.size());

	double dist = 0.0;
	for (size_t i = 0; i < state.coord.size(); ++i) {
		dist += std::pow(state.coord.at(i) - start.coord.at(i), 2);
	}
	dist = std::sqrt(dist);
	return (dist * COST_MULT);
}

unsigned int EuclideanDist::GetFromToHeuristic(int from_id, int to_id)
{
	MapState from, to;
	m_space->GetStateFromID(from_id, from);
	m_space->GetStateFromID(to_id, to);

	assert(from.coord.size() == to.coord.size());

	double dist = 0.0;
	for (size_t i = 0; i < from.coord.size(); ++i) {
		dist += std::pow(from.coord.at(i) - to.coord.at(i), 2);
	}
	dist = std::sqrt(dist);
	return (dist * COST_MULT);
}

unsigned int ManhattanDist::GetGoalHeuristic(int state_id)
{
	MapState state, goal;
	m_space->GetGoal(goal);
	m_space->GetStateFromID(state_id, state);

	assert(state.coord.size() == goal.coord.size());

	double dist = 0.0;
	for (size_t i = 0; i < state.coord.size(); ++i) {
		dist += std::abs(state.coord.at(i) - goal.coord.at(i));
	}
	return (dist * COST_MULT);
}

unsigned int ManhattanDist::GetStartHeuristic(int state_id)
{
	MapState state, start;
	m_space->GetStart(start);
	m_space->GetStateFromID(state_id, state);

	assert(state.coord.size() == start.coord.size());

	double dist = 0.0;
	for (size_t i = 0; i < state.coord.size(); ++i) {
		dist += std::abs(state.coord.at(i) - start.coord.at(i));
	}
	return (dist * COST_MULT);
}

unsigned int ManhattanDist::GetFromToHeuristic(int from_id, int to_id)
{
	MapState from, to;
	m_space->GetStateFromID(from_id, from);
	m_space->GetStateFromID(to_id, to);

	assert(from.coord.size() == to.coord.size());

	double dist = 0.0;
	for (size_t i = 0; i < from.coord.size(); ++i) {
		dist += std::abs(from.coord.at(i) - to.coord.at(i));
	}
	return (dist * COST_MULT);
}

}  // namespace AMRA
