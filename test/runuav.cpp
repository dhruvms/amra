// project includes
#include <amra/griduav.hpp>

// system includes

// standard includes
#include <vector>

using namespace AMRA;

int main(int argc, char** argv)
{
    std::string mapfile(argv[1]);
    std::string mprimfile = "../dat/mprim/mhi_3m_9m.mprim";

    // // run UAV planning for fixed starts and goals
    // std::vector<std::vector<double> > starts, goals;
    // starts = { {981.000000, 639.000000, 3.665191, 0.000000} };
    // goals = { {297.000000, 180.000000, 3.665191, 0.000000} };
    // for (int i = 0; i < starts.size(); ++i)
    // {
    //     GridUAV uav(mapfile);
    //     uav.ReadMprims(mprimfile);
    //     uav.CreateSearch();
    //     uav.SetGoal(goals[i]);
    //     uav.SetStart(starts[i]);
    //     uav.Plan(true);
    // }

    // run UAV planning for random starts and goals
    for (int i = 0; i < 1; ++i)
    {
        GridUAV uav(mapfile);
        uav.ReadMprims(mprimfile);
        uav.CreateSearch();
        uav.Plan(true);
    }
}
