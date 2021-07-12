// project includes
#include <amra/uav_env.hpp>
#include <amra/heuristic.hpp>
#include <amra/constants.hpp>
#include <amra/helpers.hpp>
#include <amra/amra.hpp>
#include <amra/wastar.hpp>

// system includes
#include <smpl/console/console.h>

// standard includes
#include <fstream>
#include <sstream>

auto std::hash<AMRA::UAVState>::operator()(
    const argument_type& s) const -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
    return seed;
}

auto split(std::string& s, char delim) -> std::vector<std::string>
{
    std::string item;
    std::stringstream ss(s);
    std::vector<std::string> elems;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
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

    if (!m_map->IsTraversible(startState[0], startState[1])) {
        printf("ERROR: Manually set start state not traversable\n");
        assert(false);
    }

    ContToDiscState(startState, m_start_coords);

    m_start_id = getOrCreateState(m_start_coords);
    m_start_set = true;

    auto* state = getHashEntry(m_start_id);
    printf("Set start (id = %d): %d, %d, %d, %d, ", m_start_id, state->coord[0], state->coord[1], state->coord[2], state->coord[3]);
    printf("Resolution::Level: %d\n", state->level);
}

void UAVEnv::SetGoal(ContState& goalState)
{
    assert(!m_goal_set);

    if (!m_map->IsTraversible(goalState[0], goalState[1])) {
        printf("ERROR: Manually set goal state not traversable\n");
        assert(false);
    }

    ContToDiscState(goalState, m_goal_coords);

    m_goal_id = getOrCreateState(m_goal_coords);
    assert(m_goal_id == 0);
    m_goal_set = true;

    auto* state = getHashEntry(m_goal_id);
    printf("Set goal (id = %d): %d, %d, %d, %d, ", m_goal_id, state->coord[0], state->coord[1], state->coord[2], state->coord[3]);
    printf("Resolution::Level: %d\n", state->level);
}

void UAVEnv::ReadMprims(std::string& mprimfile)
{
    m_actions.clear();

    std::string line;
    char split_char = ' ';
    std::vector<std::string> tokens;

    int numIntermediatePoses = 0;
    int mprimCount = 0;

    // read line
    std::ifstream mprimFileStream(mprimfile);
    while (std::getline(mprimFileStream, line))
    {
        std::istringstream lineStream(line);
        std::string field;

        Action currentAction;

        // parse current line
        while (lineStream >> field)
        {
            // std::cout << "field: " << field << std::endl;
            if (field == "resolution_m:")
            {
                std::string resolution;
                lineStream >> resolution;
                double res1 = std::stod(resolution);
                lineStream >> resolution;
                double res2 = std::stod(resolution);
            }
            else if (field == "numberofangles:")
            {
                std::string num;
                lineStream >> num;
                m_totalAngles = std::stoi(num);
            }
            else if (field == "totalnumberofprimitives:")
            {
                std::string prims;
                lineStream >> prims;
                m_totalPrims = std::stoi(prims);
                m_primsPerAngle = m_totalPrims / m_totalAngles;
            }
            else if (field == "numberofhighresprimitives:")
            {
                std::string high_res_prims;
                lineStream >> high_res_prims;
                m_numHighResPrims = std::stoi(high_res_prims);
            }
            else if (field == "numberofmidresprimitives:")
            {
                std::string mid_res_prims;
                lineStream >> mid_res_prims;
                m_numMidResPrims = std::stoi(mid_res_prims);
            }
            else if (field == "primID:")
            {
                std::string id;
                lineStream >> id;
                // currentAction.reset();
                currentAction.primID = std::stoi(id);
            }
            else if (field == "startangle_c:")
            {
                std::string angle;
                lineStream >> angle;
            }
            else if (field == "endpose_c:")
            {
                std::string dummy;
                lineStream >> dummy;
                lineStream >> dummy;
                lineStream >> dummy;
                lineStream >> dummy;
            }
            else if (field == "start_velocity:")
            {
                std::string dummy;
                lineStream >> dummy;
            }
            else if (field == "end_velocity:")
            {
                std::string dummy;
                lineStream >> dummy;
            }
            else if (field == "duration:")
            {
                std::string dummy;
                lineStream >> dummy;
            }
            else if (field == "intermediateposes:")
            {
                std::string num;
                lineStream >> num;
                numIntermediatePoses = std::stoi(num);
            }
            else
            {
                // intermediate continuous states
                int count = 0;
                do {
                    auto poses = split(line, split_char);
                    ContState state = {
                        std::stod(poses[0]),
                        std::stod(poses[1]),
                        std::stod(poses[2]),
                        std::stod(poses[3])
                    };
                    currentAction.intermediateStates.push_back(state);
                    if (count++ == numIntermediatePoses - 1)
                        break;
                } while (std::getline(mprimFileStream, line));
                storeAction(currentAction);
                ++mprimCount;
                break;
            }
        }
    }
    assert(mprimCount == m_totalPrims);
}

void UAVEnv::storeAction(Action& action)
{
    auto last_int_state = action.intermediateStates.back();
    auto first_int_state = action.intermediateStates.front();

    action.start = {
        (int)first_int_state[0],
        (int)first_int_state[1],
        ContToDiscTheta(first_int_state[2]),
        (int)first_int_state[3]
    };
    assert(action.start[3] == 0 || action.start[3] == 3 || action.start[3] == 8);

    action.end = {
        (int)last_int_state[0],
        (int)last_int_state[1],
        ContToDiscTheta(last_int_state[2]),
        (int)last_int_state[3]
    };
    assert(action.end[3] == 0 || action.end[3] == 3 || action.end[3] == 8);
    m_actions.push_back(action);
}

/// For every angle, first 9 mprims are of resolution 3m and next 9 are of
/// resolution 9m. For e.g., if disc_angle = 2, primID 0 to 8 (i.e. action idx
/// 2*18+0 = 36 to 2*18+8 = 44) are mprims of 3m resolution.
int UAVEnv::getActionIdx(int& disc_angle, int& primID)
{
    return m_primsPerAngle * disc_angle + primID;
}

void UAVEnv::CreateSearch()
{
    m_heurs.emplace_back(new EuclideanDist(this));
    m_heurs_map.emplace_back(Resolution::ANCHOR, 0); // anchor always goes first
    // m_heurs_map.emplace_back(Resolution::HIGH, 0);
    m_res_count = 1; // inadmissible resolution count
    m_heur_count = 1;

    if (NUM_RES != 2) {
        printf("For now, NUM_RES must be 2 for UAV domain.\n");
        assert(false);
    }

    /// Add MID and LOW resolution queues
    m_heurs_map.emplace_back(Resolution::MID, 0);
    m_res_count++;
    m_heurs_map.emplace_back(Resolution::LOW, 0);
    m_res_count++;

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
    int d1s, d2s, d1g, d2g;

    // set random goal
    if (!m_goal_set)
    {
        m_map->GetRandomState(d1g, d2g);
        while (d1g == d1s && d2g == d2s);
        // set goal theta and velocity to zero
        ContState goal{ (double)d1g, (double)d2g, 0.0, 0.0 };
        SetGoal(goal);
    }

    // set random start != goal
    if (!m_start_set)
    {
        do
        {
            m_map->GetRandomState(d1s, d2s);
        }
        while (d1g == d1s && d2g == d2s);
        // set start theta and velocity to zero
        ContState start{ (double)d1s, (double)d2s, 0.0, 0.0 };
        SetStart(start);
    }

    m_search->set_goal(m_goal_id);
    m_search->set_start(m_start_id);

    std::vector<int> solution;
    std::vector<int> action_ids;
    int solcost;
    bool result = m_search->replan(&solution, &action_ids, &solcost);

    if (result && save)
    {
        std::vector<ContState> solpath;
        convertPath(solution, action_ids, solpath);
        std::ofstream sol_log;
        sol_log.open("../dat/solutions/uavsol.txt");
        for (auto s : solpath)
        {
            sol_log << s[0] << ","
                    << s[1] << ","
                    << s[2] << ","
                    << s[3] << std::endl;
        }
        sol_log.close();

        return true;
    }

    return false;
}

void UAVEnv::GetSuccs(
    int state_id,
    Resolution::Level level,
    std::vector<int>* succs,
    std::vector<unsigned int>* costs,
    std::vector<int>* action_ids)
{
    assert(state_id >= 0);
    succs->clear();
    costs->clear();
    action_ids->clear();

    UAVState* parent = getHashEntry(state_id);
    assert(parent);

    /// Discrete coordinates of parent state
    auto parent_x     = parent->coord[0];
    auto parent_y     = parent->coord[1];
    auto parent_theta = parent->coord[2];
    auto parent_vel   = parent->coord[3];

    assert(m_map->IsTraversible(parent_x, parent_y));
    m_closed[static_cast<int>(level)].push_back(parent);

    // goal state should be absorbing
    if (state_id == GetGoalID()) {
        SMPL_INFO("Expanding goal state (id = %d)!", GetGoalID());
        return;
    }

    /// Apply motion primitives to generate successors of resolution grid_res.
    /// If anchor, also generate successors for other resolutions.

    int primid_start = -1;
    int primid_end   = -1;
    switch (level)
    {
        case Resolution::ANCHOR: {
            primid_start = 0;
            primid_end   = 17;
            break;
        }
        case Resolution::MID: {
            primid_start = 0;
            primid_end   = 8;
            break;
        }
        case Resolution::LOW: {
            primid_start = 9;
            primid_end   = 17;
            break;
        }
    }

    for(
    auto primid = primid_start;
    primid <= primid_end;
    ++primid)
    {
        int actionidx = getActionIdx(parent_theta, primid);
        auto action = m_actions.at(actionidx);

        /// Reject action if not applicable at parent
        auto action_start_theta = action.start[2];
        auto action_start_vel   = action.start[3];
        if (parent_theta != action_start_theta) {
            continue;
        }
        if (parent_vel != action_start_vel) {
            continue;
        }

        /// Check if applying action keeps robot collision-free and in bounds
        if (!validAction(parent, action)) {
            continue;
        }

        /// Compute discrete successor state
        DiscState succCoords =
        {
            parent->coord.at(0) + action.end.at(0), // x
            parent->coord.at(1) + action.end.at(1), // y
            action.end.at(2),                       // theta
            action.end.at(3)                        // velocity
        };
        if (succCoords[2] > m_totalAngles-1) succCoords[2] -= m_totalAngles;
        if (succCoords[2] < 0) succCoords[2] += m_totalAngles;

        int succ_state_id = getOrCreateState(succCoords);
        succs->push_back(succ_state_id);
        costs->push_back(10); // TODO: add action costs
        action_ids->push_back(actionidx);
    }
}

bool UAVEnv::validAction(UAVState* state, Action& action)
{
    for (const auto& s : action.intermediateStates)
    {
        auto intx = state->coord.at(0) + s.at(0);
        auto inty = state->coord.at(1) + s.at(1);

        // convert continuous intermediate point to 1m x 1m discrete cell
        int intx_disc = CONTXY2DISC(intx, 1.0);
        int inty_disc = CONTXY2DISC(inty, 1.0);

        // performs bounds and collision check
        if (!m_map->IsTraversible(intx_disc, inty_disc)) {
            return false;
        }
    }
    return true;
}

void UAVEnv::ContToDiscState(ContState& inContState, DiscState& outDiscState)
{
    assert(inContState.size() == 4);

    // TODO: Make sure these are in correct resolutions?
    int x = (int)inContState[0];
    int y = (int)inContState[1];

    int theta = ContToDiscTheta(inContState[2]);
    assert(validTheta(theta));

    /// Motion primitives resources/mprim/mhi_3m_9m.mprim uses discrete
    /// velocities 0, 3, and 8 m/s.
    int vels[3] = { 0, 3, 8 };
    int v = (int)inContState[3];
    int dist = std::numeric_limits<int>::max();
    int closest = -1;
    for (auto vel : vels) {
        if (abs(v - vel) < dist) {
            dist = abs(v - vel);
            closest = vel;
        }
    }
    v = closest;
    assert(v == 0 || v == 3 || v == 8);

    outDiscState = { x, y, theta, v };
}

bool UAVEnv::validTheta(int& theta)
{
    return theta >= 0 && theta < m_totalAngles;
}

bool UAVEnv::IsGoal(const int& id)
{
    UAVState state, goal;
    GetStateFromID(id, state);
    GetGoal(goal);

    auto sx = state.coord[0];
    auto sy = state.coord[1];
    auto goalx = goal.coord[0];
    auto goaly = goal.coord[1];

    auto distToGoalSqrd = (sx-goalx)*(sx-goalx) + (sy-goaly)*(sy-goaly);
    return (distToGoalSqrd < 5*5);
}

bool UAVEnv::IsGoal(const int& sx, const int& sy)
{
    UAVState goal;
    GetGoal(goal);

    auto goalx = goal.coord[0];
    auto goaly = goal.coord[1];

    auto distToGoalSqrd = (sx-goalx)*(sx-goalx) + (sy-goaly)*(sy-goaly);
    return (distToGoalSqrd < 5*5);
}

void UAVEnv::SaveExpansions(
    int iter, double w1, double w2,
    const std::vector<int>& curr_solution)
{
    printf("UAVEnv::SaveExpansions not implemented\n");
}

void UAVEnv::GetStart(MapState& start)
{
    GetStateFromID(m_start_id, start);
}

void UAVEnv::GetGoal(MapState& goal)
{
    GetStateFromID(m_goal_id, goal);
}

void UAVEnv::GetStateFromID(const int& id, MapState& state)
{
    UAVState* hashentry = getHashEntry(id);
    state = *hashentry;
}

Resolution::Level UAVEnv::GetResLevel(const int& state_id)
{
    auto s = getHashEntry(state_id);
    assert(s);
    return s->level;
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

    assert(NUM_RES == 2);
    if (d1 % LOWRES_MULT == 0 && d2 % LOWRES_MULT == 0)
    {
        entry->level = Resolution::LOW;
    }
    else
    {
        entry->level = Resolution::MID;
    }

    // map state -> state id
    m_state_to_id[entry] = state_id;

    return state_id;
}

int UAVEnv::reserveHashEntry()
{
    UAVState* entry = new UAVState;
    entry->coord.resize(4, 0);

    int state_id = (int)m_states.size();

    // map state id -> state
    m_states.push_back(entry);

    return state_id;
}

bool UAVEnv::convertPath(
    const std::vector<int>& solution_ids,
    const std::vector<int>& action_ids,
    std::vector<ContState>& path)
{
    path.clear();
    ContState start = {
        (double)m_start_coords[0],
        (double)m_start_coords[1],
        (double)m_start_coords[2],
        (double)m_start_coords[3]
    };
    path.push_back(start);

    if (solution_ids.size() == 1) {
        return true;
    }

    assert(solution_ids.size() == action_ids.size());
    for (auto i = 0; i < solution_ids.size(); ++i)
    {
        if (i == solution_ids.size()-2) break;

        MapState state;
        GetStateFromID(solution_ids[i], state);
        auto action = m_actions.at(action_ids[i+1]);

        for(
        auto wp_i = 1;
        wp_i < action.intermediateStates.size();
        ++wp_i)
        {
            const auto& wp = action.intermediateStates[wp_i];
            ContState solstate = {
                state.coord.at(0) + wp.at(0),
                state.coord.at(1) + wp.at(1),
                wp.at(2),
                wp.at(3)
            };
            path.push_back(solstate);
        }
    }
    return true;
}

}  // namespace AMRA
