// project includes
#include <amra/uav_env.hpp>

// system includes

// standard includes
#include <vector>

using namespace AMRA;

int main(int argc, char** argv)
{
    std::string mapfile(argv[1]);

    // TODO
    UAVEnv env(mapfile);
    // env.CreateSearch();
    // env.SetStart(x, y, theta, v);
    // env.SetGoal(x, y, theta, v);
    // env.Plan();
}
