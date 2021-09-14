#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP

// project includes
#include <amra/types.hpp>

// system includes

// standard includes

namespace AMRA
{

class Heuristic
{
public:
	Heuristic(Environment* space) : m_space(space) {};
	virtual ~Heuristic(){};

	virtual void Init(const DiscState& robot, const DiscState& goal) {};

	virtual unsigned int GetGoalHeuristic(int state_id) = 0;
	virtual unsigned int GetStartHeuristic(int state_id) = 0;
	virtual unsigned int GetFromToHeuristic(int from_id, int to_id) = 0;

protected:
	Environment* m_space = nullptr;
};

class EuclideanDist : public Heuristic
{
public:
	EuclideanDist(Environment* space) : Heuristic(space) {};

	unsigned int GetGoalHeuristic(int state_id) override;
	unsigned int GetStartHeuristic(int state_id) override;
	unsigned int GetFromToHeuristic(int from_id, int to_id) override;
};

class ManhattanDist : public Heuristic
{
public:
	ManhattanDist(Environment* space) : Heuristic(space) {};

	unsigned int GetGoalHeuristic(int state_id) override;
	unsigned int GetStartHeuristic(int state_id) override;
	unsigned int GetFromToHeuristic(int from_id, int to_id) override;
};

}  // namespace AMRA

#endif  // HEURISTIC_HPP
