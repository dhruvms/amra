#ifndef GRID_UAV_HPP
#define GRID_UAV_HPP

// project includes
#include <amra/movingai.hpp>
#include <amra/heuristic.hpp>

// system includes
#include <smpl/types.h>

// standard includes
#include <memory>
#include <fstream>
#include <random>

namespace std {

template <>
struct hash<AMRA::MapState>
{
    typedef AMRA::MapState argument_type;
    typedef std::size_t result_type;
    result_type operator()(const argument_type& s) const;
};

} // namespace std

namespace AMRA
{

class GridUAV : public Environment
{
public:
    GridUAV(const std::string& mapname);

    void SetStart(ContState& startState);
    void SetGoal(ContState& goalState);

    void GetStart(MapState& start);
    void GetGoal(MapState& goal);
    void GetStateFromID(const int& id, MapState& state);

    void ReadMprims(std::string& mprimfile);
    void storeAction(Action& action);
    int getActionIdx(int& disc_angle, int& primID);

    /// Required public functions from AMRA::Environment ///////////////////////
    void CreateSearch() override;
    bool Plan(bool save=false) override;

    void GetSuccs(
        int state_id,
        Resolution::Level level,
        std::vector<int>* succs,
        std::vector<unsigned int>* costs,
        std::vector<int>* action_ids) override;
    bool IsGoal(const int& id) override;

    void SaveExpansions(
        int iter, double w1, double w2,
        const std::vector<int>& curr_solution,
        const std::vector<int>& action_ids) override;

    Resolution::Level GetResLevel(const int& state_id) override;
    ////////////////////////////////////////////////////////////////////////////

    bool IsGoal(const int& sx, const int& sy);

private:
    MapState* getHashEntry(int state_id) const;
    int getHashEntry(DiscState& state);
    int getOrCreateState(DiscState& state);
    int createHashEntry(DiscState& state);
    int reserveHashEntry();

    bool convertPath(
        const std::vector<int>& solution_ids,
        const std::vector<int>& action_ids,
        std::vector<ContState>& path);

    bool validAction(MapState* state, Action& action);
    int getActionCost(std::vector<int>& startCoord, Action* action);

    /// Converts a continuous (x,y,theta,v) state to a discrete one.
    void ContToDiscState(ContState& inContState, DiscState& outDiscState);

    bool validTheta(int& theta);

private:
    std::string m_mapname;
    std::unique_ptr<MovingAI> m_map;

    std::vector<std::shared_ptr<Heuristic> > m_heurs;
    std::vector<std::pair<Resolution::Level, int> > m_heurs_map;
    int m_heur_count, m_res_count;

    DiscState m_goal_coords, m_start_coords;
    bool m_start_set, m_goal_set;
    std::vector<MapState*> m_states;
    EXPANDS_t m_closed;

    std::vector<Action> m_actions;
    int m_totalAngles, m_totalPrims, m_primsPerAngle;
    int m_numHighResPrims, m_numMidResPrims;

    ContState m_start, m_goal;

    std::random_device m_dev;
    std::mt19937 m_rng;
    std::uniform_int_distribution<> m_distI;

    // maps from coords to stateID
    typedef MapState StateKey;
    typedef smpl::PointerValueHash<StateKey> StateHash;
    typedef smpl::PointerValueEqual<StateKey> StateEqual;
    smpl::hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;
};

}  // namespace AMRA

#endif  // GRID_UAV_HPP
