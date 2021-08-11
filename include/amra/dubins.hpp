#ifndef DUBINS_HPP
#define DUBINS_HPP

// project includes
#include <amra/types.hpp>
#include <amra/heuristic.hpp>

// system includes
#include <smpl/unicycle/dubins.h>

// standard includes

namespace AMRA
{

class Dubins : public Heuristic
{
public:
	Dubins(Environment* space) : Heuristic(space) {};

	unsigned int GetGoalHeuristic(int state_id) override;
	unsigned int GetStartHeuristic(int state_id) override;
	unsigned int GetFromToHeuristic(int from_id, int to_id) override;

private:
	smpl::DubinsMotion m_motions[6];
	int m_dist;

	void dubins_dist(const MapState& from, const MapState& to);
};

}  // namespace AMRA

#endif  // DUBINS_HPP
