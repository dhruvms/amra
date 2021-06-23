#ifndef UAV_ENV_HPP
#define UAV_ENV_HPP

// project includes
#include <amra/movingai.hpp>

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

    void GetStart(MapState& start);
    void GetGoal(MapState& goal);
    void GetStateFromID(const int& id, MapState& state);

    Resolution::Level GetResLevel(const int& state_id) override;

private:
    std::string m_mapname;
    std::unique_ptr<MovingAI> m_map;

    bool m_start_set;
    bool m_goal_set;
    std::vector<UAVState*> m_states;

    // maps from coords to stateID
    typedef UAVState StateKey;
    typedef smpl::PointerValueHash<StateKey> StateHash;
    typedef smpl::PointerValueEqual<StateKey> StateEqual;
    smpl::hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;
};

}  // namespace AMRA

#endif  // UAV_ENV_HPP
