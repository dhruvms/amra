// project includes
#include <amra/uav_env.hpp>

// system includes

// standard includes
#include <vector>

using namespace AMRA;

int main(int argc, char** argv)
{
    std::string mapfile(argv[1]);

    UAVEnv uav_env(mapfile);

    std::string mprimfile = "../dat/mprim/mhi_3m_9m.mprim";
    uav_env.ReadMprims(mprimfile);

    uav_env.CreateSearch();

    // ContState goal = {
    //     /* row */ 50.,
    //     /* col */ 175.,
    //     0., 0.
    // };
    // uav_env.SetGoal(goal);

    // ContState start = {
    //     /* row */ 125.,
    //     /* col */ 20.,
    //     M_PI,
    //     0.
    // };
    // uav_env.SetStart(start);

    ContState goal = {
        /* row */ 162.,
        /* col */ 153.,
        2 * (2*M_PI/12),
        3.
    };
    uav_env.SetGoal(goal);

    ContState start = {
        /* row */ 123.,
        /* col */ 57.,
        -M_PI/2,
        0.
    };
    uav_env.SetStart(start);

    uav_env.Plan(true);
}
