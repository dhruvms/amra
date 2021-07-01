// project includes
#include <amra/grid2d.hpp>
#include <amra/heuristic.hpp>
#include <amra/constants.hpp>
#include <amra/amra.hpp>
#include <amra/wastar.hpp>
#include <amra/helpers.hpp>

// system includes
#include <smpl/console/console.h>

// standard includes

auto std::hash<AMRA::MapState>::operator()(
	const argument_type& s) const -> result_type
{
	size_t seed = 0;
	boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
	return seed;
}

namespace AMRA
{

Grid2D::Grid2D(const std::string& mapname)
:
m_mapname(mapname),
m_start_set(false),
m_goal_set(false)
{
	m_map = std::make_unique<MovingAI>(mapname);

	// reset everything
	for (MapState* s : m_states) {
		if (s != NULL) {
			delete s;
			s = nullptr;
		}
	}
	m_state_to_id.clear();
}

void Grid2D::CreateSearch()
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

void Grid2D::CreateWAStarSearch(double w)
{
	m_heurs.emplace_back(new EuclideanDist(this));
	m_search = std::make_unique<WAStar>(this, m_heurs.at(0), w);
	m_search->reset();
}

void Grid2D::SetStart(const int& d1, const int& d2)
{
	assert(!m_start_set);
	m_start_id = getOrCreateState(d1, d2);
	m_start_set = true;
}

void Grid2D::SetGoal(const int& d1, const int& d2)
{
	assert(!m_goal_set);
	m_goal_id = getOrCreateState(d1, d2);
	m_goal_set = true;
}

bool Grid2D::Plan(bool save)
{
	int d1s, d2s, d1g, d2g;
	if (!m_start_set)
	{
		// set random start
		m_map->GetRandomState(d1s, d2s);
		m_start_id = getOrCreateState(d1s, d2s);
		m_start_set = true;
	}

	if (!m_goal_set)
	{
		// set random goal
		m_map->GetRandomState(d1g, d2g);
		while (d1g == d1s && d2g == d2s) {
			m_map->GetRandomState(d1g, d2g);
		}
		m_goal_id = getOrCreateState(d1g, d2g);
		m_goal_set = true;
	}

	m_search->set_start(m_start_id);
	m_search->set_goal(m_goal_id);

	std::vector<int> solution;
	int solcost;
	bool result = m_search->replan(&solution, &solcost);

	if (result && save)
	{
		std::vector<MapState> solpath;
		convertPath(solution, solpath);
		m_map->SavePath(solpath);

		return true;
	}

	return false;
}

void Grid2D::GetSuccs(
	int state_id,
	Resolution::Level level,
	std::vector<int>* succs,
	std::vector<unsigned int>* costs)
{
	assert(state_id >= 0);
	succs->clear();
	costs->clear();

	MapState* parent = getHashEntry(state_id);
	assert(parent);
	assert(m_map->IsTraversible(parent->coord.at(0), parent->coord.at(1)));
	m_closed[static_cast<int>(level)].push_back(parent);

	// goal state should be absorbing
	if (state_id == GetGoalID()) {
		// SMPL_INFO("Expanding goal state (id = %d)!", GetGoalID());
		return;
	}

	int grid_res;
	switch (level)
	{
		case Resolution::ANCHOR:
		case Resolution::HIGH: {
			grid_res = 1;
			break;
		}
		case Resolution::MID: {
			grid_res = MIDRES_MULT;
			break;
		}
		case Resolution::LOW: {
			grid_res = LOWRES_MULT;
			break;
		}
	}

	for (int a1 = -1; a1 <= 1; ++a1)
	{
		for (int a2 = -1; a2 <= 1; ++a2)
		{
			// ignore ordinal directions for 4-connected grid
			if (GRID == 4 && std::abs(a1 * a2) == 1) {
				continue;
			}
			if (a1 == 0 && a2 == 0) {
				continue;
			}

			int succ_id = generateSuccessor(parent, a1, a2, grid_res, succs, costs);

			// generate coarse resolution successors for anchor
			if (level == Resolution::ANCHOR)
			{
				if (NUM_RES >= 2 && parent->level >= Resolution::MID) {
					succ_id = generateSuccessor(parent, a1, a2, MIDRES_MULT, succs, costs);
				}
				if (NUM_RES == 3 && parent->level >= Resolution::LOW) {
					succ_id = generateSuccessor(parent, a1, a2, LOWRES_MULT, succs, costs);
				}
			}
		}
	}
}

bool Grid2D::IsGoal(const int& id)
{
	MapState state, goal;
	GetStateFromID(id, state);
	GetGoal(goal);

	return (id == m_goal_id) && (state == goal);
}

void Grid2D::SaveExpansions(
	int iter, double w1, double w2,
	const std::vector<int>& curr_solution)
{
	m_map->SaveExpansions(iter, w1, w2, m_closed);
	if (!SAVE_ALL)
	{
		for (int i = 0; i < m_heurs_map.size(); ++i) {
			m_closed[i].clear(); // init expansions container
		}
	}

	std::vector<MapState> solpath;
	convertPath(curr_solution, solpath);
	m_map->SavePath(solpath, iter);
}

void Grid2D::GetStart(MapState& start)
{
	GetStateFromID(m_start_id, start);
}

void Grid2D::GetGoal(MapState& goal)
{
	GetStateFromID(m_goal_id, goal);
}

void Grid2D::GetStateFromID(const int& id, MapState& state)
{
	MapState* hashentry = getHashEntry(id);
	state = *hashentry;
}

Resolution::Level Grid2D::GetResLevel(const int& state_id)
{
	auto s = getHashEntry(state_id);
	assert(s);
	return s->level;
}

int Grid2D::generateSuccessor(
	const MapState* parent,
	int a1, int a2, int grid_res,
	std::vector<int>* succs,
	std::vector<unsigned int>* costs)
{
	int succ_d1, succ_d2;
	bool valid = true;
	for (int m = grid_res; m >= 1; --m) {
		succ_d1 = parent->coord.at(0) + a1*m;
		succ_d2 = parent->coord.at(1) + a2*m;
		valid = m_map->IsTraversible(succ_d1, succ_d2);

		if (!valid) {
			return -1;
		}
	}

	succ_d1 = parent->coord.at(0) + a1*grid_res;
	succ_d2 = parent->coord.at(1) + a2*grid_res;

	int succ_state_id = getOrCreateState(succ_d1, succ_d2);
	MapState* successor = getHashEntry(succ_state_id);

	succs->push_back(succ_state_id);
	costs->push_back(cost(parent, successor));

	return succ_state_id;
}

bool Grid2D::convertPath(
	const std::vector<int>& idpath,
	std::vector<MapState>& path)
{
	std::vector<MapState> opath;

	if (idpath.empty()) {
		return true;
	}

	// attempt to handle paths of length 1...do any of the sbpl planners still
	// return a single-point path in some cases?
	if (idpath.size() == 1)
	{
		auto state_id = idpath[0];

		if (state_id == GetGoalID())
		{
			auto* entry = getHashEntry(GetStartID());
			if (!entry)
			{
				SMPL_ERROR("Failed to get state entry for state %d", GetStartID());
				return false;
			}
			opath.push_back(*entry);
		}
		else
		{
			auto* entry = getHashEntry(state_id);
			if (!entry)
			{
				SMPL_ERROR("Failed to get state entry for state %d", state_id);
				return false;
			}
			opath.push_back(*entry);
		}
	}

	if (idpath[0] == GetGoalID())
	{
		SMPL_ERROR("Cannot extract a non-trivial path starting from the goal state");
		return false;
	}

	// grab the first point
	{
		auto* entry = getHashEntry(idpath[0]);
		if (!entry)
		{
			SMPL_ERROR("Failed to get state entry for state %d", idpath[0]);
			return false;
		}
		opath.push_back(*entry);
	}

	// grab the rest of the points
	for (size_t i = 1; i < idpath.size(); ++i)
	{
		auto prev_id = idpath[i - 1];
		auto curr_id = idpath[i];

		if (prev_id == GetGoalID())
		{
			SMPL_ERROR("Cannot determine goal state predecessor state during path extraction");
			return false;
		}

		auto* entry = getHashEntry(curr_id);
		if (!entry)
		{
			SMPL_ERROR("Failed to get state entry state %d", curr_id);
			return false;
		}
		opath.push_back(*entry);
	}
	path = std::move(opath);
	return true;
}

// Return a pointer to the data for the input the state id
// if it exists, else return nullptr
MapState* Grid2D::getHashEntry(int state_id) const
{
	if (state_id < 0 || state_id >= (int)m_states.size()) {
		return nullptr;
	}

	return m_states[state_id];
}

// Return the state id of the state with the given data or -1 if the
// state has not yet been allocated.
int Grid2D::getHashEntry(
		const int& d1,
		const int& d2)
{
	MapState state;
	state.coord.resize(2, 0);

	state.coord.at(0) = d1;
	state.coord.at(1) = d2;

	auto sit = m_state_to_id.find(&state);
	if (sit == m_state_to_id.end()) {
		return -1;
	}
	return sit->second;
}

int Grid2D::reserveHashEntry()
{
	MapState* entry = new MapState;
	entry->coord.resize(2, 0);

	int state_id = (int)m_states.size();

	// map state id -> state
	m_states.push_back(entry);

	// // map planner state -> graph state
	// int* pinds = new int[NUMOFINDICES_STATEID2IND];
	// std::fill(pinds, pinds + NUMOFINDICES_STATEID2IND, -1);
	// StateID2IndexMapping.push_back(pinds);

	return state_id;
}

int Grid2D::createHashEntry(
		const int& d1,
		const int& d2)
{
	int state_id = reserveHashEntry();
	MapState* entry = getHashEntry(state_id);

	entry->coord.at(0) = d1;
	entry->coord.at(1) = d2;
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

int Grid2D::getOrCreateState(
		const int& d1,
		const int& d2)
{
	int state_id = getHashEntry(d1, d2);
	if (state_id < 0) {
		state_id = createHashEntry(d1, d2);
	}
	return state_id;
}

unsigned int Grid2D::cost(
	const MapState* s1,
	const MapState* s2)
{
	if (COSTMAP)
	{
		int dir1 = sgn(s2->coord.at(0) - s1->coord.at(0));
		int dir2 = sgn(s2->coord.at(1) - s1->coord.at(1));

		int h, w;
		auto map = m_map->GetMap();
		h = m_map->GetH();
		w = m_map->GetW();

		unsigned int cost = 0;
		for (int d1 = s1->coord.at(0) + dir1, d2 = s1->coord.at(1) + dir2;
					d1 != s2->coord.at(0) + dir1 || d2 != s2->coord.at(1) + dir2;
						d1 += dir1, d2 += dir2)
		{
			cost += map[GETMAPINDEX(d1, d2, h, w)];
		}
		return (cost * COST_MULT);
	}
	else
	{
		double dist = std::sqrt(std::pow(s1->coord.at(0) - s2->coord.at(0), 2) +
								std::pow(s1->coord.at(1) - s2->coord.at(1), 2));
		return (dist * COST_MULT);
	}
}

}  // namespace AMRA
