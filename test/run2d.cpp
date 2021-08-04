// project includes
#include <amra/grid2d.hpp>

// system includes

// standard includes
#include <vector>

using namespace AMRA;

int main(int argc, char** argv)
{
	std::string mapfile(argv[1]);
	// std::vector<std::vector<int> > starts, goals;

	// starts = {
	// 			{142, 77},
	// 			{136, 60},
	// 			{49, 97},
	// 			{42, 42},
	// 			{85, 176},
	// 			{68, 144},
	// 			{157, 166},
	// 			{35, 28},
	// 			{71, 67},
	// 			{109, 108},
	// 			{79, 137},
	// 			{162, 156},
	// 			{142, 37},
	// 			{158, 149},
	// 			{127, 26},
	// 			{38, 171},
	// 			{52, 64},
	// 			{81, 40},
	// 			{56, 125},
	// 			{88, 143},
	// 		};
	// goals = {
	// 			{162, 157},
	// 			{14, 93},
	// 			{155, 88},
	// 			{140, 83},
	// 			{153, 114},
	// 			{164, 69},
	// 			{177, 88},
	// 			{106, 32},
	// 			{135, 124},
	// 			{44, 141},
	// 			{161, 164},
	// 			{46, 163},
	// 			{93, 26},
	// 			{45, 94},
	// 			{38, 138},
	// 			{124, 176},
	// 			{92, 12},
	// 			{76, 52},
	// 			{105, 178},
	// 			{46, 120},
	// 		};

	// for (int i = 0; i < starts.size(); ++i)
	// {
	// 	Grid2D grid(mapfile);
	// 	grid.CreateSearch();
	// 	// grid.CreateWAStarSearch(1.0);
	// 	grid.SetStart(starts[i][0], starts[i][1]);
	// 	grid.SetGoal(goals[i][0], goals[i][1]);
	// 	grid.Plan(false);
	// }

	Grid2D grid(mapfile);
	grid.CreateSearch();
	// grid.CreateWAStarSearch(1.0);
	grid.SetStart(28, 20);
	grid.SetGoal(15, 45);
	grid.Plan(true);
}
