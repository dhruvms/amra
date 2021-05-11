// project includes
#include <amra/environment.hpp>

// system includes

// standard includes
#include <vector>

using namespace AMRA;

int main(int argc, char** argv)
{
	std::vector<std::vector<int> > starts = {
		{164, 72},
		{79, 13},
		{71, 164},
		{27, 84},
		{103, 22},
		{84, 22},
		{72, 48},
		{25, 153},
		{24, 152},
		{90, 161},
	};
	std::vector<std::vector<int> > goals = {
		{36, 106},
		{70, 136},
		{74, 73},
		{81, 32},
		{108, 178},
		{32, 149},
		{34, 148},
		{94, 20},
		{69, 78},
		{177, 56},
	};

	std::string mapfile(argv[1]);

	// for (int trial = 0; trial < starts.size(); ++trial)
	// {
	// 	Environment env(mapfile);
	// 	env.CreateSearch();
	// 	env.SetStart(starts.at(trial)[0], starts.at(trial)[1]);
	// 	env.SetGoal(goals.at(trial)[0], goals.at(trial)[1]);
	// 	env.Plan(false);
	// }

	Environment env(mapfile);
	env.CreateSearch();
	env.Plan();
}
