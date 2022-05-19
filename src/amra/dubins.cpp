// project includes
#include <amra/dubins.hpp>
#include <amra/constants.hpp>
#include <amra/helpers.hpp>

// system includes

// standard includes

namespace AMRA
{

unsigned int Dubins::GetGoalHeuristic(int state_id)
{
	MapState state, goal;
	m_space->GetGoal(goal);
	m_space->GetStateFromID(state_id, state);

	assert(state.coord.size() == goal.coord.size());

	dubins_dist(state, goal);
	int dubins_time_ms = (m_dist / MAX_VEL) * 1000;

	return dubins_time_ms;
}

unsigned int Dubins::GetStartHeuristic(int state_id)
{
	MapState state, start;
	m_space->GetStart(start);
	m_space->GetStateFromID(state_id, state);

	assert(state.coord.size() == start.coord.size());

	dubins_dist(start, state);
	int dubins_time_ms = (m_dist / MAX_VEL) * 1000;

	return dubins_time_ms;
}

unsigned int Dubins::GetFromToHeuristic(int from_id, int to_id)
{
	MapState from, to;
	m_space->GetStateFromID(from_id, from);
	m_space->GetStateFromID(to_id, to);

	assert(from.coord.size() == to.coord.size());

	dubins_dist(from, to);
	int dubins_time_ms = (m_dist / MAX_VEL) * 1000;

	return dubins_time_ms;
}

void Dubins::dubins_dist(const MapState& from, const MapState& to)
{
	m_dist = std::numeric_limits<int>::max();

	int from_theta = from.coord.size() == 2 ? 0 : from.coord.at(2);
	int to_theta = to.coord.size() == 2 ? 0 : to.coord.at(2);
	int num_paths = smpl::MakeDubinsPaths(
		smpl::Pose2D(from.coord.at(0), from.coord.at(1), DiscToContTheta(from_theta)),
		smpl::Pose2D(to.coord.at(0), to.coord.at(1), DiscToContTheta(to_theta)),
		TURNING_RADIUS / 1.0, m_motions);

	for (int i = 0; i < num_paths; ++i)
	{
		if (m_motions[i].length() < m_dist) {
			m_dist = m_motions[i].length();
		}
	}
}

}  // namespace AMRA
