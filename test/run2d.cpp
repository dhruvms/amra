// project includes
#include <amra/grid2d.hpp>

// system includes

// standard includes
#include <vector>

using namespace AMRA;

int main(int argc, char** argv)
{
	std::string mapfile(argv[1]);
// 	std::vector<std::vector<int> > starts, goals;

// 	starts = { ... 	};
// 	goals = { ... };

// 	for (int i = 0; i < starts.size(); ++i)
// 	{
// 		Grid2D grid(mapfile);
// 		grid.CreateSearch();
// 		// grid.CreateARAStarSearch();
// 		grid.SetStart(starts[i][0], starts[i][1]);
// 		grid.SetGoal(goals[i][0], goals[i][1]);
// 		grid.Plan(true);
// 	}

	for (int i = 0; i < 100; ++i)
	{
		Grid2D grid(mapfile);
		grid.CreateSearch();
		grid.Plan(true);
	}
}
