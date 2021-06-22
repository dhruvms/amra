// project includes
#include <amra/grid2d.hpp>

// system includes

// standard includes
#include <vector>

using namespace AMRA;

int main(int argc, char** argv)
{
	std::string mapfile(argv[1]);
	Grid2D grid(mapfile);
	grid.CreateSearch();
	grid.SetStart(28, 20);
	grid.SetGoal(15, 45);
	// grid.CreateWAStarSearch(1.0);
	grid.Plan();
}
