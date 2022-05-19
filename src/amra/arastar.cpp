// project includes
#include <amra/arastar.hpp>
#include <amra/constants.hpp>
#include <amra/types.hpp>
#include <amra/heuristic.hpp>
#include <amra/helpers.hpp>

// system includes
#include <smpl/console/console.h>

// standard includes
#include <algorithm>

namespace AMRA
{

ARAStar::ARAStar(
	Environment* space,
	std::shared_ptr<Heuristic> heur)
:
m_space(space),
m_call_number(0),
m_heur(heur),
m_w_delta(0.5), m_w_i(10.0), m_w_f(1.0),
m_start_id(-1),
m_goal_id(-1)
{
	// Set default max planing time
	m_time_limit = double(MAX_PLANNING_TIME_MS / 1000.0); // seconds
	m_w = m_w_i;

	m_open = new OpenList[1];
	m_expands = new int[1];

	m_initial_t = 0.0;
	m_final_t = 0.0;
	m_initial_c = -1;
	m_final_c = -1;
	m_total_e = -1;
}

ARAStar::~ARAStar()
{
	reset();
}

int ARAStar::set_start(int start_id)
{
	m_start_id = start_id;
	m_start = get_state(m_start_id);
	return m_start_id;
}

int ARAStar::set_goal(int goal_id)
{
	m_goal_id = goal_id;
	m_goal = get_state(m_goal_id);
	return m_goal_id;
}

int ARAStar::get_n_expands() const
{
	return m_expands[0];
}

void ARAStar::reset()
{
	// Clear OPEN list
	for (int i = 0; i < num_heuristics(); ++i) {
		if (!m_open[i].empty()) {
			m_open[i].clear();
		}
	}

	// free states
	for (size_t i = 0; i < m_states.size(); ++i)
	{
		if (m_states[i] != nullptr)
		{
			free(m_states[i]);
			// m_states[i] = nullptr;
		}
	}

	// Clear state table
	m_states.clear();
	// m_states.shrink_to_fit();

	m_start_id = -1;
	m_goal_id = -1;
	m_start = nullptr;
	m_goal = nullptr;
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
ARAStarState* ARAStar::get_state(int state_id)
{
	assert(state_id >= 0);

	if (m_states.size() <= state_id)
	{
		size_t state_size = sizeof(ARAStarState);
		ARAStarState* s = (ARAStarState*)malloc(state_size);

		// Use placement new(s) to construct allocated memory
		new (s) ARAStarState;

		// assert(state_id == m_states.size());

		init_state(s, state_id);
		m_states.push_back(s);

		return s;
	}

	return m_states[state_id];
}

void ARAStar::init_state(ARAStarState *state, int state_id)
{
	state->call_number = 0; // not initialized for any iteration
	state->state_id = state_id;
	for (int i = 0; i < num_heuristics(); ++i) {
		state->od[i].me = state;
	}
}

// Lazily (re)initialize a search state.
void ARAStar::reinit_state(ARAStarState *state)
{
	if (state->call_number != m_call_number) {
		state->call_number = m_call_number;
		state->g = std::numeric_limits<unsigned int>::max();
		state->bp = nullptr;

		for (int i = 0; i < num_heuristics(); ++i) {
			state->od[i].h = compute_heuristic(state->state_id, i);
			state->od[i].f = std::numeric_limits<unsigned int>::max();
		}

		state->closed = false;
	}
}

int ARAStar::replan(
	std::vector<int>* solution_path, int* solution_cost)
{
	if (is_goal(m_start_id))
	{
		// m_logger->LogMsg("Start is goal!", LogLevel::WARN);
		solution_path->push_back(m_start_id);
		solution_cost = 0;
		return 1;
	}

	m_w_solve = -1;

	for (int i = 0; i < num_heuristics(); ++i) {
		m_expands[i] = 0;
	}

	m_call_number++;
	reinit_state(m_goal);
	reinit_state(m_start);
	m_start->g = 0;

	// clear all OPEN lists
	for (int i = 0; i < num_heuristics(); ++i) {
		m_open[i].clear();
	}

	m_incons.clear();
	m_incons.push_back(m_start);

	m_search_time = 0.0;
	m_iter = 0;

	while (m_search_time < m_time_limit && (m_w >= m_w_f))
	{
		for (auto* s : m_incons)
		{
			s->od[0].f = compute_key(s, 0);
			s->closed = false;
			insert_or_update(s, 0);
		}
		m_incons.clear();

		reorder_open();
		for (auto* s : m_states) {
			s->closed = false;
		}

		double search_start_time = GetTime();
		double search_time = 0.0;
		int curr_exps = get_n_expands();
		bool result = improve_path(search_start_time, search_time);

		m_search_time += search_time;

		if(!result || m_search_time >= m_time_limit) {
			break;
		}
		if (m_w_solve < 0) {
			m_initial_t = m_search_time;
		}

		extract_path(*solution_path, *solution_cost);
		SMPL_INFO("Solved with (%f) | expansions = %d | time = %f | cost = %d", m_w, get_n_expands(), search_time, *solution_cost);
		if (curr_exps < get_n_expands()) {
			m_space->SaveExpansions(m_iter, m_w, 1.0, *solution_path);
		}

		if (m_w == m_w_f) {
			break;
		}
		m_w = std::max(m_w_f, m_w * m_w_delta);

		m_iter++;

		if (SUCCESSIVE)
		{
			// clear all OPEN lists
			for (int i = 0; i < num_heuristics(); ++i) {
				m_open[i].clear();
			}

			m_incons.clear();
			m_incons.push_back(m_start);
		}
	}

	if (m_w_solve < 0)
	{
		solution_path->clear();
		*solution_cost = -1;
		return 0;
	}

	// SMPL_INFO("Total expansions = %d, Total time = %f", get_n_expands(), m_search_time);
	m_final_t = m_search_time;
	m_final_c = *solution_cost;
	m_total_e = get_n_expands();

	return 1;
}

bool ARAStar::improve_path(
	const double& start_time,
	double& elapsed_time)
{
	elapsed_time = 0.0;
	while (!m_open[0].empty() &&
				m_open[0].min()->f < std::numeric_limits<unsigned int>::max())
	{
		elapsed_time = GetTime() - start_time;
		if (elapsed_time >= m_time_limit) {
			return false;
		}

		if (m_open[0].empty()) {
			return false;
		}

		unsigned int f_check = m_open[0].min()->f;
		if (m_goal->g <= f_check) {
			return true;
		}

		// expand from anchor
		ARAStarState *s = m_open[0].min()->me;
		expand(s, 0);
		if (s->state_id == m_goal_id) {
			return true;
		}
		++m_expands[0];
	}
}

void ARAStar::expand(ARAStarState *s, int hidx)
{
	assert(!s->closed);
	s->closed = true;

	if (m_open[0].contains(&s->od[0])) {
		m_open[0].erase(&s->od[0]);
	}

	std::vector<int> succ_ids;
	std::vector<unsigned int> costs;
	m_space->GetSuccs(s->state_id, static_cast<Resolution::Level>(-1), &succ_ids, &costs, hidx);

	for (size_t sidx = 0; sidx < succ_ids.size(); ++sidx)
	{
		unsigned int cost = costs[sidx];

		ARAStarState *succ_state = get_state(succ_ids[sidx]);
		reinit_state(succ_state);

		unsigned int new_g = s->g + costs[sidx];
		if (new_g < succ_state->g)
		{
			succ_state->g = new_g;
			succ_state->bp = s;
			if (succ_state->closed) {
				m_incons.push_back(succ_state);
			}
			else
			{
				succ_state->od[0].f = compute_key(succ_state, 0);
				insert_or_update(succ_state, 0);
			}
		}
	}
}

bool ARAStar::is_goal(int state_id)
{
	return m_space->IsGoal(state_id);
}

unsigned int ARAStar::compute_heuristic(int state_id, int hidx)
{
	assert(num_heuristics() >= hidx);
	return m_heur->GetGoalHeuristic(state_id);

}

unsigned int ARAStar::compute_key(ARAStarState *state, int hidx)
{
	return state->g + m_w * state->od[hidx].h;
}

void ARAStar::insert_or_update(ARAStarState *state, int hidx)
{
	if (m_open[hidx].contains(&state->od[hidx])) {
		m_open[hidx].update(&state->od[hidx]);
	}
	else {
		m_open[hidx].push(&state->od[hidx]);
	}
}

void ARAStar::reorder_open()
{
	for (auto hidx = 0; hidx < num_heuristics(); hidx++)
	{
		for (auto it = m_open[hidx].begin(); it != m_open[hidx].end(); ++it) {
			(*it)->f = compute_key((*it)->me, hidx);
		}
		m_open[hidx].make();
	}
}

void ARAStar::extract_path(
	std::vector<int>& solution, int& cost)
{
	if (m_w_solve < 0) {
		m_initial_c = m_goal->g;
	}

	m_w_solve = m_w;
	cost = m_goal->g;
	m_solution_cost = m_goal->g;

	solution.clear();

	// m_goal->state_id == m_goal_id == 0 should be true
	for (ARAStarState *state = m_goal; state; state = state->bp) {
		solution.push_back(state->state_id);
	}
	std::reverse(solution.begin(), solution.end());
}

}  // namespace CMUPlanner
