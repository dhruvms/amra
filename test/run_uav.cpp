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
    UAVEnv uav_env(mapfile);

    std::string mprimfile = "../dat/mprim/mhi_3m_9m.mprim";
    uav_env.ReadMprims(mprimfile);

    uav_env.CreateSearch();

    ContState goal = { 27.3, 51.4, 0.0, 0.0 };
    uav_env.SetGoal(goal);

    // ContState start = { 12.5, 9.5, 0.0, 0.0 };
    ContState start = { 3.5, 9.5, 0.0, 0.0 };
    uav_env.SetStart(start);

    // ANCHOR = 0,
    // HIGH = 1,
    // MID = 2,
    // LOW = 3
    int state_id = 1;
    Resolution::Level level = Resolution::MID;
    std::vector<int> succs;
    std::vector<unsigned int> costs;

    uav_env.GetSuccs(state_id, level, &succs, &costs);

    printf("Huuuuu\n");

    // uav_env.Plan();
}
