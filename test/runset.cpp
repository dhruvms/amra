// project includes
#include <amra/grid2d.hpp>

// system includes

// standard includes
#include <random>

using namespace AMRA;

int main(int argc, char** argv)
{
	std::string maptype(argv[1]), mapfile;

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0.0, 1.0);

	// run 2D gridworld planning for random starts and goals
	mapfile = "../dat/" + maptype + "-map/";
	for (int i = 0; i < 1; ++i)
	{
		mapfile += maptype + "512-";
		int id1, id2;
		if (maptype.compare("maze") == 0)
		{
			int exp = (std::ceil(dis(gen) * 6)) - 1;
			id1 = int(std::pow(2, exp));
			id2 = int((std::ceil(dis(gen) * 10)) - 1);
			mapfile += std::to_string(id1) + "-" + std::to_string(id2) + ".map";
		}
		else if (maptype.compare("random") == 0)
		{
			int mult = (std::ceil(dis(gen) * 7)) + 1;
			id1 = 5 * mult;
			id2 = int((std::ceil(dis(gen) * 10)) - 1);
			mapfile += std::to_string(id1) + "-" + std::to_string(id2) + ".map";
		}

		Grid2D grid(mapfile);
		grid.CreateARASearch();
		grid.Plan(true);
	}
}
