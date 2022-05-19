// project includes
#include <amra/dijkstra.hpp>
#include <amra/constants.hpp>
#include <amra/movingai.hpp>

// system includes

// standard includes
#include <cmath>
#include <cassert>

auto std::hash<AMRA::AbstractState>::operator()(
	const argument_type& s) const -> result_type
{
	size_t seed = 0;
	boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
	return seed;
}

namespace AMRA
{

Dijkstra::Dijkstra(Environment* space, MovingAI* map) :
	Heuristic(space), m_map(map), m_start_id(-1), m_goal_id(-1)
{
	m_open = new OpenList[1];
}

void Dijkstra::Init(const DiscState& robot, const DiscState& goal)
{
	m_start = goal;
	m_goal = robot;

	m_start_id = getOrCreateState(m_start);
	m_goal_id = getOrCreateState(m_goal);

	AbstractState* s = getHashEntry(m_start_id);
	s->g = 0;
	s->od[0].f = s->g + s->od[0].h;
	insert_or_update(s);

	resume(m_goal_id);
}

unsigned int Dijkstra::GetGoalHeuristic(int state_id)
{
	MapState state;
	m_space->GetStateFromID(state_id, state);

	int dijkstra_id = getOrCreateState(state.coord);
	AbstractState* s = getHashEntry(dijkstra_id);

	if (s->closed) {
		return s->g;
	}

	if (resume(dijkstra_id)) {
		return s->g;
	}

	return std::numeric_limits<unsigned int>::max();
}

bool Dijkstra::resume(int state_id)
{
	while (!m_open[0].empty())
	{
		AbstractState *parent = m_open[0].min()->me;
		parent->closed = true;
		m_open[0].pop();

		if (parent->state_id == state_id) {
			return true;
		}

		for (int d1 = 1; d1 >= -1; --d1)
		{
			for (int d2 = 1; d2 >= -1; --d2)
			{
				// ignore ordinal directions for 4-connected grid
				if (GRID == 4 && std::abs(d1 * d2) == 1) {
					continue;
				}
				if (d1 == 0 && d2 == 0) {
					continue;
				}

				DiscState child_s = parent->coord;
				child_s.at(0) += d1;
				child_s.at(1) += d2;
				if (!m_map->IsTraversible(child_s.at(0), child_s.at(1))) {
					continue;
				}

				int child_id = getOrCreateState(child_s);
				AbstractState* child = getHashEntry(child_id);

				unsigned int new_g = parent->g + cost(child_s, parent->coord);
				if (new_g < child->g)
				{
					child->g = new_g;
					child->bp = parent;
					if (!child->closed)
					{
						child->od[0].f = child->g + child->od[0].h;
						insert_or_update(child);
					}
				}
			}
		}
	}

	return false;
}

int Dijkstra::getOrCreateState(DiscState& s)
{
	int state_id = getHashEntry(s);
	if (state_id < 0) {
		state_id = createHashEntry(s);
	}
	return state_id;
}

int Dijkstra::getHashEntry(DiscState& s)
{
	AbstractState t;
	t.coord.resize(2, 0);

	t.coord.at(0) = s.at(0);
	t.coord.at(1) = s.at(1);
	auto sit = m_state_to_id.find(&t);
	if (sit == m_state_to_id.end()) {
		return -1;
	}
	return sit->second;
}

int Dijkstra::createHashEntry(const DiscState& s)
{
	int state_id = reserveHashEntry();
	AbstractState* entry = getHashEntry(state_id);

	entry->state_id = state_id;
	entry->g = std::numeric_limits<unsigned int>::max();
	entry->bp = nullptr;
	entry->closed = false;

	entry->coord.at(0) = s.at(0);
	entry->coord.at(1) = s.at(1);

	entry->od[0].me = entry;
	entry->od[0].h = euclidean_dist(entry->coord);
	entry->od[0].f = std::numeric_limits<unsigned int>::max();

	// map state -> state id
	m_state_to_id[entry] = state_id;

	return state_id;
}

AbstractState* Dijkstra::getHashEntry(int state_id) const
{
	if (state_id < 0 || state_id >= (int)m_states.size()) {
		return nullptr;
	}
	return m_states[state_id];
}

int Dijkstra::reserveHashEntry()
{
	int state_id = (int)m_states.size();

	AbstractState* entry = new AbstractState;
	entry->coord.resize(2, 0.0);

	// map state id -> state
	m_states.push_back(entry);

	return state_id;
}

void Dijkstra::insert_or_update(AbstractState *state)
{
	if (m_open[0].contains(&state->od[0])) {
		m_open[0].update(&state->od[0]);
	}
	else {
		m_open[0].push(&state->od[0]);
	}
}

unsigned int Dijkstra::euclidean_dist(const DiscState& s)
{
	return cost(s, m_goal);
}

unsigned int Dijkstra::cost(const DiscState& a, const DiscState& b)
{
	assert(a.size() == b.size());

	double dist = 0.0;
	for (size_t i = 0; i < a.size(); ++i) {
		dist += std::pow(a.at(i) - b.at(i), 2);
	}
	dist = std::sqrt(dist);
	return COST_MULT * dist;
}

}  // namespace AMRA
