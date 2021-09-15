// project includes
#include <amra/uav_env.hpp>

// system includes

// standard includes
#include <vector>

using namespace AMRA;

int main(int argc, char** argv)
{
    std::string mapfile(argv[1]);
    std::string savefile(argv[2]);
    std::string mprimfile = "../dat/mprim/mhi_3m_9m.mprim";

    // std::vector<std::vector<double> > starts, goals;

    // starts = { ... };
    // goals = { ... };

    // for (int i = 0; i < starts.size(); ++i)
    // {

    //     UAVEnv uav_env(mapfile, savefile);
    //     uav_env.ReadMprims(mprimfile);
    //     uav_env.CreateSearch();
    //     uav_env.SetGoal(goals[i]);
    //     uav_env.SetStart(starts[i]);
    //     uav_env.Plan(true);
    // }

    for (int i = 0; i < 1; ++i)
    {
        UAVEnv uav_env(mapfile, savefile);
        uav_env.ReadMprims(mprimfile);
        uav_env.CreateSearch();
        uav_env.Plan(true);
    }
}
