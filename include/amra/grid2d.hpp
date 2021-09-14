#ifndef GRID2D_HPP
#define GRID2D_HPP

// project includes
#include <amra/movingai.hpp>

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

class Heuristic;

class Grid2D : public Environment
{
public:
	Grid2D(const std::string& mapname);

	void SetStart(const int& d1, const int& d2);
	void SetGoal(const int& d1, const int& d2);

	void CreateSearch() override;
	void CreateARAStarSearch();
	bool Plan(bool save=false) override;

	void GetSuccs(
		int state_id,
		Resolution::Level level,
		std::vector<int>* succs,
		std::vector<unsigned int>* costs,
		int hidx) override;
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

	std::vector<std::shared_ptr<Heuristic> > m_heurs;
	std::vector<std::pair<Resolution::Level, int> > m_heurs_map;
	int m_heur_count, m_res_count;
	Resolution::Level m_default_res;

	int m_s1, m_s2, m_g1, m_g2;
	bool m_start_set, m_goal_set;
	std::vector<MapState*> m_states;
	EXPANDS_t m_closed;

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
		std::vector<unsigned int>* costs);
	unsigned int cost(
		const MapState* s1,
		const MapState* s2);

	bool convertPath(
		const std::vector<int>& idpath,
		std::vector<MapState>& path);
};

}  // namespace AMRA

#endif  // GRID2D_HPP
