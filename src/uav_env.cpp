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

void UAVEnv::SetStart(ContState& startState)
{
    assert(!m_start_set);

    DiscState coords =
    {
        (int)startState[0],
        (int)startState[1],
        (int)startState[2],
        (int)startState[3]
    };

    m_start_id = getOrCreateState(coords);
    m_start_set = true;

    auto* state = getHashEntry(m_start_id);
    printf("Set start: %d, %d, %d, %d\n", state->coord[0], state->coord[1], state->coord[2], state->coord[3]);
    printf("Resolution::Level: %d\n", state->level);
}

void UAVEnv::SetGoal(ContState& goalState)
{
    assert(!m_goal_set);

    DiscState coords =
    {
        (int)goalState[0],
        (int)goalState[1],
        (int)goalState[2],
        (int)goalState[3]
    };

    m_goal_id = getOrCreateState(coords);
    m_goal_set = true;

    auto* state = getHashEntry(m_goal_id);
    printf("Set goal: %d, %d, %d, %d\n", state->coord[0], state->coord[1], state->coord[2], state->coord[3]);
    printf("Resolution::Level: %d\n", state->level);
}

void UAVEnv::CreateSearch()
{
    m_heurs.emplace_back(new EuclideanDist(this));
    m_heurs_map.emplace_back(Resolution::ANCHOR, 0); // anchor always goes first
    m_heurs_map.emplace_back(Resolution::HIGH, 0);
    m_res_count = 1; // inadmissible resolution count
    m_heur_count = 1;

    if (NUM_RES > 1)
    {
        m_heurs_map.emplace_back(Resolution::MID, 0);
        m_res_count++;
    }
    if (NUM_RES == 3)
    {
        m_heurs_map.emplace_back(Resolution::LOW, 0);
        m_res_count++;
    }

    for (int i = 0; i < m_heurs_map.size(); ++i) {
        m_closed[i].clear(); // init expansions container
    }

    m_search = std::make_unique<AMRAStar>(
        this, m_heurs, m_heurs_map,
        m_heur_count, m_res_count);
    m_search->reset();
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

// Return a pointer to the data for the input the state id
// if it exists, else return nullptr
UAVState* UAVEnv::getHashEntry(int state_id) const
{
    if (state_id < 0 || state_id >= (int)m_states.size()) {
        return nullptr;
    }
    return m_states[state_id];
}

// Return the state id of the state with the given data or -1 if the
// state has not yet been allocated.
int UAVEnv::getHashEntry(DiscState& inCoords)
{
    UAVState state;
    state.coord = inCoords;

    auto sit = m_state_to_id.find(&state);
    if (sit == m_state_to_id.end()) {
        return -1;
    }
    return sit->second;
}

int UAVEnv::getOrCreateState(DiscState& inCoords)
{
    int state_id = getHashEntry(inCoords);
    if (state_id < 0) {
        state_id = createHashEntry(inCoords);
    }
    return state_id;
}

int UAVEnv::createHashEntry(DiscState& inCoords)
{
    int state_id = reserveHashEntry();
    UAVState* entry = getHashEntry(state_id);

    entry->coord = inCoords;

    int d1 = inCoords.at(0); // x
    int d2 = inCoords.at(1); // y

    if (NUM_RES == 3 &&
        (d1 % LOWRES_MULT == 0 && d2 % LOWRES_MULT == 0))
    {
        entry->level = Resolution::LOW;
    }
    else if (NUM_RES >= 2 &&
        (d1 % MIDRES_MULT == 0 && d2 % MIDRES_MULT == 0)) {
        entry->level = Resolution::MID;
    }
    else {
        entry->level = Resolution::HIGH;
    }

    // map state -> state id
    m_state_to_id[entry] = state_id;

    return state_id;
}

int UAVEnv::reserveHashEntry()
{
    UAVState* entry = new UAVState;
    entry->coord.resize(2, 0);

    int state_id = (int)m_states.size();

    // map state id -> state
    m_states.push_back(entry);

    return state_id;
}


}  // namespace AMRA
