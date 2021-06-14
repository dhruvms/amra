// project includes
#include <amra/environment.hpp>
#include <amra/heuristic.hpp>
#include <amra/constants.hpp>
#include <amra/amra.hpp>
#include <amra/wastar.hpp>

// system includes
#include <smpl/console/console.h>

// standard includes

auto std::hash<AMRA::MapState>::operator()(
	const argument_type& s) const -> result_type
{
	size_t seed = 0;
	boost::hash_combine(seed, s.d1);
	boost::hash_combine(seed, s.d2);
	return seed;
}

namespace AMRA
{

Environment::Environment(const std::string& mapname)
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

void Environment::CreateSearch()
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

void Environment::CreateWAStarSearch(double w)
{
	m_heurs.emplace_back(new EuclideanDist(this));
	m_search = std::make_unique<WAStar>(this, m_heurs.at(0), w);
	m_search->reset();
}

void Environment::SetStart(const int& d1, const int& d2)
{
	assert(!m_start_set);
	m_start_id = getOrCreateState(d1, d2);
	m_start_set = true;
}

void Environment::SetGoal(const int& d1, const int& d2)
{
	assert(!m_goal_set);
	m_goal_id = getOrCreateState(d1, d2);
	m_goal_set = true;
}

bool Environment::Plan(bool save)
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

void Environment::GetSuccs(
	int state_id,
	Resolution::Level level,
	std::vector<int>* succs,
	std::vector<unsigned int>* costs)
{
	assert(state_id >= 0);
	succs->clear();
	costs->clear();

	// goal state should be absorbing
	if (state_id == GetGoalID()) {
		SMPL_INFO("Expanding goal state (id = %d)!", GetGoalID());
		return;
	}

	MapState* parent = getHashEntry(state_id);
	assert(parent);
	assert(m_map->IsTraversible(parent->d1, parent->d2));
	m_closed[static_cast<int>(level)].push_back(parent);

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

bool Environment::IsGoal(const int& id)
{
	MapState state, goal;
	GetStateFromID(id, state);
	GetGoal(goal);

	return (id == m_goal_id) && (state == goal);
}

void Environment::SaveExpansions(
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

void Environment::GetStart(MapState& start)
{
	GetStateFromID(m_start_id, start);
}

void Environment::GetGoal(MapState& goal)
{
	GetStateFromID(m_goal_id, goal);
}

void Environment::GetStateFromID(const int& id, MapState& state)
{
	MapState* hashentry = getHashEntry(id);
	state = *hashentry;
}

Resolution::Level Environment::GetResLevel(const int& state_id)
{
	auto s = getHashEntry(state_id);
	assert(s);
	return s->level;
}

int Environment::generateSuccessor(
	const MapState* parent,
	int a1, int a2, int grid_res,
	std::vector<int>* succs,
	std::vector<unsigned int>* costs)
{
	int succ_d1, succ_d2;
	bool valid = true;
	for (int m = grid_res; m >= 1; --m) {
		succ_d1 = parent->d1 + a1*m;
		succ_d2 = parent->d2 + a2*m;
		valid = m_map->IsTraversible(succ_d1, succ_d2);

		if (!valid) {
			return -1;
		}
	}

	succ_d1 = parent->d1 + a1*grid_res;
	succ_d2 = parent->d2 + a2*grid_res;

	int succ_state_id = getOrCreateState(succ_d1, succ_d2);
	MapState* successor = getHashEntry(succ_state_id);

	succs->push_back(succ_state_id);
	costs->push_back(cost(parent, successor));

	return succ_state_id;
}

bool Environment::convertPath(
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
MapState* Environment::getHashEntry(int state_id) const
{
	if (state_id < 0 || state_id >= (int)m_states.size()) {
		return nullptr;
	}

	return m_states[state_id];
}

// Return the state id of the state with the given data or -1 if the
// state has not yet been allocated.
int Environment::getHashEntry(
		const int& d1,
		const int& d2)
{
	MapState state;
	state.d1 = d1;
	state.d2 = d2;

	auto sit = m_state_to_id.find(&state);
	if (sit == m_state_to_id.end()) {
		return -1;
	}
	return sit->second;
}

int Environment::reserveHashEntry()
{
	MapState* entry = new MapState;
	int state_id = (int)m_states.size();

	// map state id -> state
	m_states.push_back(entry);

	// // map planner state -> graph state
	// int* pinds = new int[NUMOFINDICES_STATEID2IND];
	// std::fill(pinds, pinds + NUMOFINDICES_STATEID2IND, -1);
	// StateID2IndexMapping.push_back(pinds);

	return state_id;
}

int Environment::createHashEntry(
		const int& d1,
		const int& d2)
{
	int state_id = reserveHashEntry();
	MapState* entry = getHashEntry(state_id);

	entry->d1 = d1;
	entry->d2 = d2;
	if (NUM_RES == 3 &&
			(entry->d1 % LOWRES_MULT == 0 && entry->d2 % LOWRES_MULT == 0))
	{
		entry->level = Resolution::LOW;
	}
	else if (NUM_RES >= 2 &&
			(entry->d1 % MIDRES_MULT == 0 && entry->d2 % MIDRES_MULT == 0)) {
		entry->level = Resolution::MID;
	}
	else {
		entry->level = Resolution::HIGH;
	}

	// map state -> state id
	m_state_to_id[entry] = state_id;

	return state_id;
}

int Environment::getOrCreateState(
		const int& d1,
		const int& d2)
{
	int state_id = getHashEntry(d1, d2);
	if (state_id < 0) {
		state_id = createHashEntry(d1, d2);
	}
	return state_id;
}

unsigned int Environment::cost(
	const MapState* s1,
	const MapState* s2)
{
	double dist = std::sqrt(std::pow(s1->d1 - s2->d1, 2) + std::pow(s1->d2 - s2->d2, 2));
	return (dist * COST_MULT);
}

}  // namespace CMUPlanner
