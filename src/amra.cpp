// project includes
#include <amra/amra.hpp>
#include <amra/constants.hpp>
#include <amra/types.hpp>
#include <amra/heuristic.hpp>

// system includes
#include <smpl/console/console.h>
#include <smpl/time.h>

// standard includes
#include <algorithm>

static double GetTime()
{
	using namespace smpl;
	return to_seconds(clock::now().time_since_epoch());
}

namespace AMRA
{

AMRAStar::AMRAStar(
	Environment* space,
	const std::vector<std::shared_ptr<Heuristic>>& heurs,
	const std::vector<std::pair<Resolution::Level, int> >& heurs_map,
	int heur_count, int res_count)
:
m_space(space),
m_call_number(0),
m_heur_count(heur_count),
m_res_count(res_count),
m_w1_i(2.0), m_w2_i(2.0),
m_w1_f(1.0), m_w2_f(1.0),
m_w1_delta(0.5), m_w2_delta(0.5),
m_start_id(-1),
m_goal_id(-1)
{
	// Set default max planing time
	m_time_limit = double(MAX_PLANNING_TIME_MS / 1000.0); // seconds

	m_w1 = m_w1_i;
	m_w2 = m_w2_i;

	m_heurs = heurs;
	m_heurs_map = heurs_map;
	m_open = new OpenList[m_heurs_map.size()];  // inadmissible(s) + anchor
	m_expands = new int[m_heurs_map.size()];
}

AMRAStar::~AMRAStar()
{
	reset();
}

int AMRAStar::set_start(int start_id)
{
	m_start_id = start_id;
	m_start = get_state(m_start_id);
	printf("AMRA: set start %d\n", m_start_id);
	return m_start_id;
}

int AMRAStar::set_goal(int goal_id)
{
	m_goal_id = goal_id;
	m_goal = get_state(m_goal_id);
	printf("AMRA: set goal %d\n", m_goal_id);
	return m_goal_id;
}

int AMRAStar::get_n_expands() const
{
	int expansions = 0;
	for (int i = 0; i < num_heuristics(); ++i) {
		expansions += m_expands[i];
	}
	return expansions;
}

std::string AMRAStar::get_expands_str() const
{
	std::string expansions;
	for (int i = 0; i < num_heuristics(); ++i) {
		expansions += std::to_string(m_expands[i]);
		if (i < num_heuristics() - 1) {
			expansions += ", ";
		}
	}
	return expansions;
}

void AMRAStar::reset()
{
	// Clear OPEN lists
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
AMRAState* AMRAStar::get_state(int state_id)
{
	assert(state_id >= 0);

	if (m_states.size() <= state_id)
	{
		size_t state_size =
			sizeof(AMRAState) +
			sizeof(AMRAState::HeapData) * (num_heuristics() - 1) +
			sizeof(bool) * (m_res_count - 1);
		AMRAState* s = (AMRAState*)malloc(state_size);

		// Use placement new(s) to construct allocated memory
		new (s) AMRAState;
		for (int i = 0; i < m_res_count-1; ++i) {
			new (&s->closed_in_res[1 + i]) bool;
		}
		for (int i = 0; i < num_heuristics() - 1; ++i) {
			new (&s->od[1 + i]) AMRAState::HeapData;
		}

		// assert(state_id == m_states.size());

		init_state(s, state_id);
		m_states.push_back(s);

		return s;
	}

	return m_states[state_id];
}

void AMRAStar::init_state(AMRAState *state, int state_id)
{
	state->call_number = 0; // not initialized for any iteration
	state->state_id = state_id;
	for (int i = 0; i < num_heuristics(); ++i) {
		state->od[i].me = state;
	}
}

// Lazily (re)initialize a search state.
void AMRAStar::reinit_state(AMRAState *state)
{
	if (state->call_number != m_call_number) {
		state->call_number = m_call_number;
		state->g = std::numeric_limits<unsigned int>::max();
		state->res = m_space->GetResLevel(state->state_id);
		state->bp = nullptr;

		for (int i = 0; i < num_heuristics(); ++i) {
			state->od[i].h = compute_heuristic(state->state_id, i);
			state->od[i].f = std::numeric_limits<unsigned int>::max();
		}

		state->closed_in_anc = false;
		for (int i = 0; i < m_res_count; ++i) {
			state->closed_in_res[i] = false;
		}
	}
}

int AMRAStar::replan(
	std::vector<int>* solution_path,
	std::vector<int>* action_ids,
	int* solution_cost)
{
	if (is_goal(m_start_id))
	{
		// m_logger->LogMsg("Start is goal!", LogLevel::WARN);
		printf("    Start is goal!");
		solution_path->push_back(m_start_id);
		solution_cost = 0;
		return 1;
	}

	m_w1 = m_w1_i;
	m_w2 = m_w2_i;
	m_w1_solve = -1.0;
	m_w2_solve = -1.0;

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

	while (m_search_time < m_time_limit && (m_w1 >= m_w1_f && m_w2 >= m_w2_f))
	{
		for (auto* s : m_incons)
		{
			s->od[0].f = compute_key(s, 0);
			s->closed_in_anc = false;
			insert_or_update(s, 0);
		}
		m_incons.clear();

		for (auto it = m_open[0].begin(); it != m_open[0].end(); ++it)
		{
			for (auto hidx = 1; hidx < num_heuristics(); hidx++)
			{
				// numerically greater resolutions are coarser
				if ((*it)->me->res >= m_heurs_map.at(hidx).first)
				{
					(*it)->me->od[hidx].f = compute_key((*it)->me, hidx);
					(*it)->me->closed_in_res[hidx - 1] = false;
					insert_or_update((*it)->me, hidx);
				}
			}
		}
		reorder_open();

		double search_start_time = GetTime();
		double search_time = 0.0;
		bool result = improve_path(search_start_time, search_time);

		m_search_time += search_time;

		if(!result) {
			break;
		}

		// SMPL_INFO("Solved with (%f, %f) | expansions = %s | time = %f", m_w1, m_w2, get_expands_str().c_str(), search_time);
		printf("Solved with (%f, %f) | expansions = %s | time = %f\n", m_w1, m_w2, get_expands_str().c_str(), search_time);
		extract_path(*solution_path, *action_ids, *solution_cost);
		m_space->SaveExpansions(m_iter, m_w1, m_w2, *solution_path, *action_ids);

		if (m_w1 == m_w1_f && m_w2 == m_w2_f) {
			break;
		}
		m_w1 = std::max(m_w1_f, m_w1 * m_w1_delta);
		m_w2 = std::max(m_w2_f, m_w2 * m_w2_delta);

		m_iter++;
	}

	if (m_w1_solve < 0 || m_w2_solve < 0)
	{
		solution_path->clear();
		*solution_cost = -1;
		return 0;
	}

	// m_ss << "time (s) = " << m_search_time << " | expansions = " << get_n_expands() << " | solution_cost = " << m_solution_cost;
	// log_it(LogLevel::WARN);

	SMPL_INFO("%d (%s), %f", get_n_expands(), get_expands_str().c_str(), m_search_time);

	return 1;
}

bool AMRAStar::improve_path(
	const double& start_time,
	double& elapsed_time)
{
	printf("improve_path ...\n");

	elapsed_time = 0.0;
	while (!m_open[0].empty() &&
				m_open[0].min()->f < std::numeric_limits<unsigned int>::max())
	{
		elapsed_time = GetTime() - start_time;
		if (elapsed_time >= m_time_limit) {
			return false;
		}

		for (int i = 1; i < num_heuristics(); ++i)
		{
			if (m_open[0].empty()) {
				printf("[1] AMRA*: OPEN[0] empty\n");
				return false;
			}

			unsigned int f_check = m_w2 * m_open[0].min()->f;
			printf("\tmin_f =     [%u], state ID = [%d]\n", m_open[0].min()->f, m_open[0].min()->me->state_id);
			printf("\tf_check =   [%u]\n", f_check);
			printf("\tm_goal->g = [%u]\n", m_goal->g);
			printf("\n");
			if (m_goal->g <= f_check) {
				printf("  m_goal->g = [%u]\n", m_goal->g);
				return true;
			}

			if (!m_open[i].empty() &&
					m_open[i].min()->f <= f_check)
			{
				AMRAState *s = m_open[i].min()->me;
				expand(s, i);
				if (s->state_id == m_goal_id) {
					return true;
				}
				++m_expands[i];
			}
			else
			{
				// expand from anchor
				AMRAState *s = m_open[0].min()->me;
				expand(s, 0);
				if (s->state_id == m_goal_id) {
					return true;
				}
				++m_expands[0];
			}
		}
	}
	printf("[2] AMRA*: OPEN[0] empty\n");
}

void AMRAStar::expand(AMRAState *s, int hidx)
{
	// close s in correct resolution
	// and remove from appropriate OPENs

	int hres_i = static_cast<int>(m_heurs_map.at(hidx).first);
	if (hidx == 0)
	{
		assert(!s->closed_in_anc);
		s->closed_in_anc = true;

		if (m_open[0].contains(&s->od[0])) {
			m_open[0].erase(&s->od[0]);
		}
	}
	else
	{
		assert(!s->closed_in_res[hres_i - 1]);
		s->closed_in_res[hres_i - 1] = true;
		if (m_open[hidx].contains(&s->od[hidx])) {
			m_open[hidx].erase(&s->od[hidx]);
		}

		for (int j = 1; j < num_heuristics(); ++j)
		{
			if (j != hidx && hres_i == static_cast<int>(m_heurs_map.at(j).first))
			{
				if (m_open[j].contains(&s->od[j])) {
					m_open[j].erase(&s->od[j]);
				}
			}
		}
	}

	bool exp_pot_goal = false;

	std::vector<int> succ_ids;
	std::vector<unsigned int> costs;
	std::vector<int> action_ids;
	if (is_goal(s->state_id))
	{
		MapState state; m_space->GetStateFromID(s->state_id, state);
		printf("  expanding potential goal [%d, %d, %d, %d]\n", state.coord[0], state.coord[1], state.coord[2], state.coord[3]);
		exp_pot_goal = true;

		succ_ids.push_back(m_goal_id);
		costs.push_back(1);
		action_ids.push_back(-1);
	}
	else
	{
		m_space->GetSuccs(s->state_id,
						  static_cast<Resolution::Level>(hres_i),
						  &succ_ids,
						  &costs,
						  &action_ids);
	}

	for (size_t sidx = 0; sidx < succ_ids.size(); ++sidx)
	{
		unsigned int cost = costs[sidx];

		AMRAState *succ_state = get_state(succ_ids[sidx]);
		reinit_state(succ_state);

		unsigned int new_g = s->g + costs[sidx];

		if (new_g < succ_state->g)
		{
			if (exp_pot_goal) {
				printf("\tedge cost:                 [%u]\n", cost);
				printf("\told g-value of m_goal:     [%u]\n", m_goal->g);
				printf("\told g-value of m_goal:     [%u]\n", succ_state->g);
				printf("\tupdated g-value of m_goal: [%u]\n", new_g);
				printf("\n");
				printf("\tsolution can be improved\n");
			}

			succ_state->g = new_g;
			succ_state->bp = s;
			if (!action_ids.empty())
				succ_state->actionidx = action_ids[sidx];
			if (succ_state->closed_in_anc) {
				m_incons.push_back(succ_state);
			}
			else
			{
				unsigned int f_0 = compute_key(succ_state, 0);
				succ_state->od[0].f = f_0;
				insert_or_update(succ_state, 0);

				for (int j = 1; j < num_heuristics(); ++j)
				{
					int hres_j = static_cast<int>(m_heurs_map.at(j).first);
					// if state resolution is coarser than queue resolution,
					// insert or update in queue
					// this assumes a high resolution grid coincides with
					// all grids coarser than it
					if (static_cast<int>(succ_state->res) >= hres_j)
					{
						if (!succ_state->closed_in_res[hres_j - 1])
						{
							unsigned int f_j = compute_key(succ_state, j);
							if (f_j <= m_w2 * f_0)
							{
								succ_state->od[j].f = f_j;
								insert_or_update(succ_state, j);
							}
						}
					}
				}
			}
		}
	}
}

bool AMRAStar::is_goal(int state_id)
{
	return m_space->IsGoal(state_id);
}

unsigned int AMRAStar::compute_heuristic(int state_id, int hidx)
{
	assert(num_heuristics() >= hidx);
	return m_heurs.at(m_heurs_map.at(hidx).second)->GetGoalHeuristic(state_id);
}

unsigned int AMRAStar::compute_key(AMRAState *state, int hidx)
{
	return state->g + m_w1 * state->od[hidx].h;
}

void AMRAStar::insert_or_update(AMRAState *state, int hidx)
{
	if (m_open[hidx].contains(&state->od[hidx])) {
		m_open[hidx].update(&state->od[hidx]);
	}
	else {
		m_open[hidx].push(&state->od[hidx]);
	}
}

void AMRAStar::reorder_open()
{
	for (auto hidx = 0; hidx < num_heuristics(); hidx++)
	{
		for (auto it = m_open[hidx].begin(); it != m_open[hidx].end(); ++it) {
			(*it)->f = compute_key((*it)->me, hidx);
		}
		m_open[hidx].make();
	}
}

void AMRAStar::extract_path(
	std::vector<int>& solution,
	std::vector<int>& action_ids,
	int& cost)
{
	m_w1_solve = m_w1;
	m_w2_solve = m_w2;
	cost = m_goal->g;
	m_solution_cost = m_goal->g;
	printf("  Solution cost: [%u]\n", m_solution_cost);

	solution.clear();
	action_ids.clear();

	// m_goal->state_id == m_goal_id == 0 should be true
	for (AMRAState *state = m_goal; state; state = state->bp) {
		solution.push_back(state->state_id);
		action_ids.push_back(state->actionidx);
	}
	std::reverse(solution.begin(), solution.end());
	std::reverse(action_ids.begin(), action_ids.end());
}

}  // namespace AMRA
