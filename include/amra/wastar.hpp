#ifndef WASTAR_HPP
#define WASTAR_HPP

// project includes
#include <amra/types.hpp>

// system includes
#include <smpl/heap/intrusive_heap.h>

// standard includes
#include <vector>
#include <memory>

namespace AMRA
{

struct WAStarState
{
	int call_number;
	int state_id;
	unsigned int g;
	WAStarState* bp;
	// std::pair<int, int> actionids;

	struct HeapData : public smpl::heap_element
	{
		// TODO: rather than map back to the state, the heap could know its
		// index into od for updates, though that might make it hard to
		// overallocate an array for the heap index, h, and f

		// TODO: in any case, this offset can be much smaller
		WAStarState* me;
		unsigned int h;
		unsigned int f;
	};

	bool closed;
	HeapData od[1]; // overallocated for additional n heuristics
};

class Environment;
class Heuristic;

class WAStar : public Search
{
public:
	WAStar(
		Environment* space,
		std::shared_ptr<Heuristic> heur,
		double w=1.0);
	~WAStar();

	int set_start(int start_id) override;
	int set_goal(int goal_id) override;
	void set_max_planning_time(double max_planning_time_ms) override {
		m_time_limit = max_planning_time_ms * 1e-3;
	};
	int get_n_expands() const override;
	void reset() override;

	int replan(
		std::vector<int>* solution_path, int* solution_cost) override;

private:
	Environment* m_space = nullptr;

	std::shared_ptr<Heuristic> m_heur;

	struct HeapCompare {
		bool operator()(
				const WAStarState::HeapData& s,
				const WAStarState::HeapData& t) const
		{
			return s.f < t.f;
		}
	};

	using OpenList = smpl::intrusive_heap<WAStarState::HeapData, HeapCompare>;
	OpenList* m_open = nullptr;  // sequence of (m_heur_count + 1) open lists

	// Search params
	int m_call_number;
	double m_time_limit, m_w;
	WAStarState *m_goal, *m_start;

	int m_start_id, m_goal_id;

	std::vector<WAStarState*> m_states;

	// Search statistics
	double m_search_time;
	int *m_expands; // expansions per queue
	int m_solution_cost;

	int num_heuristics() const { return 1; }

	WAStarState* get_state(int state_id);
	// WAStarState* create_state(int state_id);
	void init_state(WAStarState *state, int state_id);
	void reinit_state(WAStarState *state);

	void expand(WAStarState *state, int hidx);
	bool is_goal(int state_id);

	unsigned int compute_heuristic(int state_id, int hidx);
	unsigned int compute_key(WAStarState *state, int hidx);
	void insert_or_update(WAStarState *state, int hidx);
	void reorder_open();

	void extract_path(
		std::vector<int>& solution, int& cost);
};

}  // namespace AMRA

#endif  // WASTAR_HPP
