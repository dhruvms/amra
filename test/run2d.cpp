// project includes
#include <amra/environment.hpp>

// system includes

// standard includes
#include <vector>

using namespace AMRA;

int main(int argc, char** argv)
{
	std::string mapfile(argv[1]);
	Environment env(mapfile);
	env.CreateSearch();
	// env.CreateWAStarSearch(1.0);
	env.Plan();
}
