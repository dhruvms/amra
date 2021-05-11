#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP

// project includes
#include <amra/environment.hpp>

// system includes

// standard includes

namespace AMRA
{

class Heuristic
{
public:
	Heuristic(Environment* space) : m_space(space) {};
	virtual ~Heuristic(){};

	virtual int GetGoalHeuristic(int state_id) = 0;
	virtual int GetStartHeuristic(int state_id) = 0;
	virtual int GetFromToHeuristic(int from_id, int to_id) = 0;

protected:
	Environment* m_space = nullptr;
};

class EuclideanDistSq : public Heuristic
{
public:
	EuclideanDistSq(Environment* space) : Heuristic(space) {};

	int GetGoalHeuristic(int state_id) override;
	int GetStartHeuristic(int state_id) override;
	int GetFromToHeuristic(int from_id, int to_id) override;
};

}  // namespace AMRA

#endif  // HEURISTIC_HPP
