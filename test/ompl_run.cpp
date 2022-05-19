#include <amra/movingai.hpp>
#include <amra/CostConvergenceTerminationCondition.h>
#include <amra/helpers.hpp>

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

	bool plan(unsigned int d1s, unsigned int d2s, unsigned int d1g, unsigned int d2g, double runTime)
	{
		if (!m_si) {
			return false;
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
			const int d1 = std::min(m_d1_lim, (int)p->getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
			const int d2 =
				std::min(m_d2_lim, (int)p->getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
			printf("(%d, %d)\n", d1, d2);
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
	std::vector<std::vector<int> > starts, goals;

	starts = {
{840, 357},
{420, 168},
{777, 819},
{840, 777},
{483, 147},
{588, 630},
{567, 672},
{294, 966},
{441, 252},
{441, 210},
{882, 567},
{189, 189},
{504, 924},
{189, 126},
{630, 252},
{609, 672},
{420, 63},
{651, 84},
{147, 315},
{252, 231},
{273, 966},
{756, 882},
{462, 861},
{798, 504},
{609, 252},
{525, 1008},
{672, 42},
{231, 819},
{819, 756},
{399, 693},
{462, 714},
{651, 462},
{336, 504},
{84, 651},
{714, 798},
{147, 714},
{210, 525},
{399, 672},
{651, 252},
{672, 105},
{588, 714},
{399, 63},
{294, 1008},
{735, 84},
{798, 588},
{735, 21},
{231, 966},
{819, 525},
{420, 63},
{357, 63},
{336, 1008},
{273, 819},
{735, 84},
{147, 483},
{231, 588},
{819, 588},
{399, 168},
{273, 273},
{378, 147},
{126, 609},
{294, 966},
{462, 756},
{126, 483},
{756, 420},
{798, 357},
{693, 231},
{462, 672},
{693, 210},
{819, 588},
{462, 105},
{525, 105},
{546, 819},
{525, 819},
{882, 567},
{567, 882},
{714, 798},
{273, 609},
{273, 546},
{483, 945},
{189, 105},
{168, 714},
{462, 168},
{462, 756},
{147, 651},
{441, 105},
{861, 420},
{672, 42},
{273, 903},
{441, 714},
{756, 210},
{861, 735},
{252, 651},
{420, 147},
{126, 504},
{147, 210},
{735, 819},
{714, 189},
{777, 483},
{651, 189},
{504, 315},
	};
	goals = {
{756, 777},
{840, 819},
{735, 483},
{714, 273},
{231, 693},
{168, 147},
{252, 546},
{189, 168},
{672, 210},
{756, 798},
{777, 462},
{252, 987},
{714, 63},
{819, 441},
{252, 1008},
{294, 357},
{756, 126},
{609, 252},
{147, 378},
{189, 168},
{588, 714},
{399, 189},
{168, 357},
{609, 336},
{273, 567},
{273, 525},
{588, 357},
{294, 966},
{273, 819},
{651, 798},
{357, 819},
{168, 588},
{483, 945},
{378, 756},
{399, 693},
{399, 63},
{483, 840},
{420, 252},
{525, 609},
{525, 819},
{147, 483},
{630, 147},
{672, 441},
{147, 210},
{336, 840},
{168, 210},
{126, 294},
{504, 651},
{630, 189},
{630, 441},
{210, 819},
{378, 714},
{189, 168},
{693, 168},
{735, 819},
{147, 693},
{735, 840},
{588, 525},
{483, 63},
{567, 861},
{861, 693},
{357, 42},
{567, 357},
{588, 924},
{861, 714},
{588, 357},
{357, 672},
{315, 987},
{546, 609},
{630, 252},
{588, 945},
{567, 336},
{861, 840},
{861, 378},
{210, 567},
{819, 315},
{651, 126},
{252, 273},
{777, 336},
{315, 777},
{651, 735},
{378, 525},
{546, 378},
{504, 42},
{735, 798},
{315, 336},
{231, 336},
{693, 147},
{441, 882},
{714, 84},
{252, 903},
{651, 126},
{861, 693},
{630, 441},
{588, 714},
{756, 777},
{798, 483},
{567, 903},
{819, 420},
{399, 210},
			};

	for (int i = 0; i < starts.size(); ++i)
	{
		MovingAIOMPL env(mapfile);
		env.plan(starts[i][0], starts[i][1], goals[i][0], goals[i][1], 5.0);
	}

	return 0;
 }
