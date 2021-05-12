// project includes
#include <amra/heuristic.hpp>
#include <amra/constants.hpp>

// system includes

// standard includes

namespace AMRA
{

unsigned int EuclideanDist::GetGoalHeuristic(int state_id)
{
	MapState state, goal;
	m_space->GetGoal(goal);
	m_space->GetStateFromID(state_id, state);

	double dist_sq = std::sqrt(std::pow(state.d1 - goal.d1, 2) + std::pow(state.d2 - goal.d2, 2));
	return (dist_sq * COST_MULT);
}

unsigned int EuclideanDist::GetStartHeuristic(int state_id)
{
	MapState state, start;
	m_space->GetStart(start);
	m_space->GetStateFromID(state_id, state);

	double dist_sq = std::sqrt(std::pow(state.d1 - start.d1, 2) + std::pow(state.d2 - start.d2, 2));
	return (dist_sq * COST_MULT);
}

unsigned int EuclideanDist::GetFromToHeuristic(int from_id, int to_id)
{
	MapState from, to;
	m_space->GetStateFromID(from_id, from);
	m_space->GetStateFromID(to_id, to);

	double dist_sq = std::sqrt(std::pow(from.d1 - to.d1, 2) + std::pow(from.d2 - to.d2, 2));
	return (dist_sq * COST_MULT);
}

}  // namespace CMUPlanner
