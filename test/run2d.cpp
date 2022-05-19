// project includes
#include <amra/grid2d.hpp>

// system includes

// standard includes
#include <vector>

using namespace AMRA;

int main(int argc, char** argv)
{
	std::string mapfile(argv[1]);

	// // run 2D gridworld planning for fixed starts and goals
	// std::vector<std::vector<int> > starts, goals;
	// starts = { {28, 12} };
	// goals = { {5, 45} };
	// for (int i = 0; i < starts.size(); ++i)
	// {
	// 	Grid2D grid(mapfile);
	// 	grid.CreateSearch();
	// 	// grid.CreateARAStarSearch();
	// 	grid.SetStart(starts[i][0], starts[i][1]);
	// 	grid.SetGoal(goals[i][0], goals[i][1]);
	// 	grid.Plan(true);
	// }

	// run 2D gridworld planning for random starts and goals
	for (int i = 0; i < 1; ++i)
	{
		Grid2D grid(mapfile);
		grid.CreateSearch();
		grid.Plan(true);
	}
}
