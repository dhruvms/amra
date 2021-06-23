// project includes
#include <amra/uav_env.hpp>
#include <amra/heuristic.hpp>
#include <amra/constants.hpp>
#include <amra/amra.hpp>
#include <amra/wastar.hpp>

// system includes
#include <smpl/console/console.h>

// standard includes

auto std::hash<AMRA::UAVState>::operator()(
    const argument_type& s) const -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
    return seed;
}

namespace AMRA
{

UAVEnv::UAVEnv(const std::string& mapname)
:
m_mapname(mapname),
m_start_set(false),
m_goal_set(false)
{
    m_map = std::make_unique<MovingAI>(mapname);

    // reset everything
    for (auto* s : m_states) {
        if (s != NULL) {
            delete s;
            s = nullptr;
        }
    }
    m_state_to_id.clear();
}

void UAVEnv::CreateSearch()
{

}

bool UAVEnv::Plan(bool save)
{
    return false;
}

void UAVEnv::GetSuccs(
    int state_id,
    Resolution::Level level,
    std::vector<int>* succs,
    std::vector<unsigned int>* costs)
{

}

bool UAVEnv::IsGoal(const int& id)
{
    return false;
}

void UAVEnv::SaveExpansions(
    int iter, double w1, double w2,
    const std::vector<int>& curr_solution)
{

}

void UAVEnv::GetStart(MapState& start)
{

}

void UAVEnv::GetGoal(MapState& goal)
{

}

void UAVEnv::GetStateFromID(const int& id, MapState& state)
{

}

Resolution::Level UAVEnv::GetResLevel(const int& state_id)
{
    UAVState s; return s.level;
}

}  // namespace AMRA
