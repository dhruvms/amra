#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

// project includes
#include <amra/movingai.hpp>
#include <amra/amra.hpp>

// system includes
#include <smpl/types.h>

// standard includes
#include <memory>

namespace std {

template <>
struct hash<AMRA::MapState>
{
	typedef AMRA::MapState argument_type;
	typedef std::size_t result_type;
	result_type operator()(const argument_type& s) const;
};

}
 // namespace std

namespace AMRA
{

class Environment
{
public:
	Environment(const std::string& mapname);

	void CreateSearch();
	void SetStart(const int& d1, const int& d2);
	void SetGoal(const int& d1, const int& d2);
	bool Plan(bool save=true);

	void GetSuccs(
		int state_id,
		Resolution::Level level,
		std::vector<int>* succs,
		std::vector<int>* costs);
	bool IsGoal(const int& id);

	int GetStartID() const { return m_start_id; };
    int GetGoalID() const { return m_goal_id; };

    void GetStart(MapState& goal);
    void GetGoal(MapState& goal);
    void GetStateFromID(const int& id, MapState& state);

    Resolution::Level GetResLevel(const int& state_id);

private:
	std::string m_mapname;
	std::unique_ptr<MovingAI> m_map;
	std::unique_ptr<AMRAStar> m_search;

	std::vector<std::shared_ptr<Heuristic> > m_heurs;
	std::vector<std::pair<Resolution::Level, int> > m_heurs_map;
	int m_heur_count, m_res_count;

	bool m_start_set, m_goal_set;
	int m_start_id, m_goal_id, m_expansions = 0;
	std::vector<MapState*> m_states;

	// maps from coords to stateID
	typedef MapState StateKey;
	typedef smpl::PointerValueHash<StateKey> StateHash;
	typedef smpl::PointerValueEqual<StateKey> StateEqual;
	smpl::hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;

	MapState* getHashEntry(int state_id) const;
	int getHashEntry(
		const int& d1,
		const int& d2);
	int reserveHashEntry();
	int createHashEntry(
		const int& d1,
		const int& d2);
	int getOrCreateState(
		const int& d1,
		const int& d2);

	int generateSuccessor(
		const MapState* parent,
		int a1, int a2, int grid_res,
		std::vector<int>* succs,
		std::vector<int>* costs);
	int cost(
		const MapState* s1,
		const MapState* s2);

	bool convertPath(
		const std::vector<int>& idpath,
		std::vector<MapState>& path);
};

}  // namespace AMRA

#endif  // ENVIRONMENT_HPP
