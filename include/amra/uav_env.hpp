#ifndef UAV_ENV_HPP
#define UAV_ENV_HPP

// project includes
#include <amra/movingai.hpp>
#include <amra/heuristic.hpp>

// system includes
#include <smpl/types.h>

// standard includes
#include <memory>

namespace std {

template <>
struct hash<AMRA::UAVState>
{
    typedef AMRA::UAVState argument_type;
    typedef std::size_t result_type;
    result_type operator()(const argument_type& s) const;
};

}
 // namespace std

namespace AMRA
{

class UAVEnv : public Environment
{
public:
    UAVEnv(const std::string& mapname);

    void SetStart(ContState& startState);
    void SetGoal(ContState& goalState);

    void GetStart(MapState& start);
    void GetGoal(MapState& goal);
    void GetStateFromID(const int& id, MapState& state);

    void ReadMprims(std::string& mprimfile);

    /// Required public functions from AMRA::Environment ///////////////////////
    void CreateSearch() override;
    bool Plan(bool save=false) override;

    void GetSuccs(
        int state_id,
        Resolution::Level level,
        std::vector<int>* succs,
        std::vector<unsigned int>* costs) override;
    bool IsGoal(const int& id) override;

    void SaveExpansions(
        int iter, double w1, double w2,
        const std::vector<int>& curr_solution) override;

    Resolution::Level GetResLevel(const int& state_id) override;
    ////////////////////////////////////////////////////////////////////////////

private:
    UAVState* getHashEntry(int state_id) const;
    int getHashEntry(DiscState& state);
    int getOrCreateState(DiscState& state);
    int createHashEntry(DiscState& state);
    int reserveHashEntry();

private:
    std::string m_mapname;
    std::unique_ptr<MovingAI> m_map;

    std::vector<std::shared_ptr<Heuristic> > m_heurs;
    std::vector<std::pair<Resolution::Level, int> > m_heurs_map;
    int m_heur_count, m_res_count;

    bool m_start_set, m_goal_set;
    std::vector<UAVState*> m_states;
    EXPANDS_t m_closed;

    std::vector<Action> m_actions;
    int m_totalAngles, m_totalPrims, m_primsPerAngle;

    // maps from coords to stateID
    typedef UAVState StateKey;
    typedef smpl::PointerValueHash<StateKey> StateHash;
    typedef smpl::PointerValueEqual<StateKey> StateEqual;
    smpl::hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;
};

}  // namespace AMRA

#endif  // UAV_ENV_HPP
