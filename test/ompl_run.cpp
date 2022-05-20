#include <amra/movingai.hpp>
#include <amra/helpers.hpp>
#include <amra/CostConvergenceTerminationCondition.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/config.h>

#include <vector>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class MovingAIOMPL
{ public:
	MovingAIOMPL(const std::string& mapname) : m_mapname(mapname)
	{
		// MovingAI parser
		m_map = std::make_unique<AMRA::MovingAI>(mapname);

		////////////////
		// OMPL setup //
		////////////////

		// Construct the robot state space in which we're planning, a subset of R^2.
		auto space(std::make_shared<ob::RealVectorStateSpace>());
		space->addDimension(0.0, m_map->GetW());
		space->addDimension(0.0, m_map->GetH());
		m_d1_lim = m_map->GetW() - 1;
		m_d2_lim = m_map->GetH() - 1;

		// Construct a space information instance for this state space
		m_si = std::make_shared<ob::SpaceInformation>(space);
		// Set the function used to check which states in the space are valid
		m_si->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });

		m_si->setup();
		m_si->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
	}

	bool plan(double runTime, int d1s=-1, int d2s=-1, int d1g=-1, int d2g=-1)
	{
		if (!m_si) {
			return false;
		}

		if (d1s < 0 || d2s < 0)
		{
			// get random start
			m_map->GetRandomState(d1s, d2s);
		}

		if (d1g < 0 || d2g < 0)
		{
			// get random goal
			do {
				m_map->GetRandomState(d1g, d2g);
			}
			while (d1g == d1s && d2g == d2s);
		}

		ob::ScopedState<> start(m_si->getStateSpace());
		start[0] = d1s;
		start[1] = d2s;
		ob::ScopedState<> goal(m_si->getStateSpace());
		goal[0] = d1g;
		goal[1] = d2g;

		// Create a problem instance
		m_pdef = std::make_shared<ob::ProblemDefinition>(m_si);
		// Set the start and goal states
		m_pdef->setStartAndGoalStates(start, goal);
		// Set the optimization objective
		m_pdef->setOptimizationObjective(m_obj);

		// Construct the optimal planner
		m_planner = std::make_shared<og::RRTstar>(m_si);

		// Create the optimization objective
		m_obj = std::make_shared<ob::PathLengthOptimizationObjective>(m_si);

		// Set the problem instance for our planner to solve
		m_planner->setProblemDefinition(m_pdef);
		m_planner->setup();

		// attempt to solve the planning problem in the given runtime
		double start_time = AMRA::GetTime();
		// ob::PlannerStatus solved = m_planner->solve(ob::plannerOrTerminationCondition(ob::CostConvergenceTerminationCondition(m_pdef, 1, 1.0), ob::timedPlannerTerminationCondition(runTime)));
		ob::PlannerStatus solved = m_planner->solve(ob::plannerOrTerminationCondition(ob::CostConvergenceTerminationCondition(m_pdef), ob::timedPlannerTerminationCondition(runTime)));
		double t_i = AMRA::GetTime() - start_time;

		if (solved)
		{
			ob::PlannerData data(m_si);
			m_planner->getPlannerData(data);
			std::cout << "{" << data.numVertices() << ", " << t_i << ", " << std::floor(m_pdef->getSolutionPath()->length() * 1000) << "}" << std::endl;

			return true;
		}
		else {
			std::cout << "No solution found." << std::endl;
			return false;
		}
	}

	void printSolution()
	{
		auto p = std::static_pointer_cast<og::PathGeometric>(m_pdef->getSolutionPath());
		p->interpolate();
		for (std::size_t i = 0; i < p->getStateCount(); ++i)
		{
			const double d1 = std::min((double)m_d1_lim, p->getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
			const double d2 =
				std::min((double)m_d2_lim, p->getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
			printf("(%f, %f)\n", d1, d2);
		}
	}

private:
	std::string m_mapname;
	std::unique_ptr<AMRA::MovingAI> m_map;
	int m_d1_lim, m_d2_lim;

	ob::SpaceInformationPtr m_si;
	ob::PlannerPtr m_planner;
	ob::OptimizationObjectivePtr m_obj;
	ob::ProblemDefinitionPtr m_pdef;

	bool isStateValid(const ob::State *state) const
	{
		const int d1 = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], m_d1_lim);
		const int d2 = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], m_d2_lim);

		return m_map->IsTraversible(d1, d2);
	}

 };

 int main(int argc, char** argv)
 {
	std::string mapfile(argv[1]);

	// // run RRT* planning for fixed starts and goals
	// std::vector<std::vector<int> > starts, goals;
	// starts = { ... };
	// goals = { ... };
	// for (int i = 0; i < starts.size(); ++i)
	// {
	// 	MovingAIOMPL env(mapfile);
	// 	env.plan(starts[i][0], starts[i][1], goals[i][0], goals[i][1], 5.0);
	// }

	// run 2D gridworld planning for random starts and goals
	for (int i = 0; i < 1; ++i)
	{
		MovingAIOMPL env(mapfile);
		env.plan(5.0);
		env.printSolution();
	}

	return 0;
 }
