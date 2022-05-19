#ifndef ARASTAR_HPP
#define ARASTAR_HPP

// project includes
#include <amra/types.hpp>

// system includes
#include <smpl/heap/intrusive_heap.h>

// standard includes
#include <vector>
#include <memory>

namespace AMRA
{

struct ARAStarState
{
	int call_number;
	int state_id;
	unsigned int g;
	ARAStarState* bp;
	int actionidx;

	struct HeapData : public smpl::heap_element
	{
		// TODO: rather than map back to the state, the heap could know its
		// index into od for updates, though that might make it hard to
		// overallocate an array for the heap index, h, and f

		// TODO: in any case, this offset can be much smaller
		ARAStarState* me;
		unsigned int h;
		unsigned int f;
	};

	bool closed;
	HeapData od[1]; // overallocated for additional n heuristics
};

class Environment;
class Heuristic;

class ARAStar : public Search
{
public:
	ARAStar(
		Environment* space,
		std::shared_ptr<Heuristic> heur);
	~ARAStar();

	int set_start(int start_id) override;
	int set_goal(int goal_id) override;
	void set_max_planning_time(double max_planning_time_ms) override {
		m_time_limit = max_planning_time_ms * 1e-3;
	};
	int get_n_expands() const override;
	void reset() override;

	int replan(
		std::vector<int>* solution_path,
		std::vector<int>* action_ids,
		int* solution_cost) override;

private:
	Environment* m_space = nullptr;

	std::shared_ptr<Heuristic> m_heur;

	struct HeapCompare {
		bool operator()(
				const ARAStarState::HeapData& s,
				const ARAStarState::HeapData& t) const
		{
			return s.f < t.f;
		}
	};

	using OpenList = smpl::intrusive_heap<ARAStarState::HeapData, HeapCompare>;
	OpenList* m_open = nullptr;  // sequence of (m_heur_count + 1) open lists

	// Search params
	int m_call_number, m_iter;
	double m_time_limit, m_w, m_w_delta, m_w_i, m_w_f;
	ARAStarState *m_goal, *m_start;

	int m_start_id, m_goal_id;

	std::vector<ARAStarState*> m_states, m_incons;

	// Search statistics
	double m_search_time;
	int *m_expands; // expansions per queue
	int m_w_solve, m_solution_cost;

	int num_heuristics() const { return 1; }

	ARAStarState* get_state(int state_id);
	// ARAStarState* create_state(int state_id);
	void init_state(ARAStarState *state, int state_id);
	void reinit_state(ARAStarState *state);

	bool improve_path(
		const double& start_time,
		double& elapsed_time);
	void expand(ARAStarState *state, int hidx);
	bool is_goal(int state_id);

	unsigned int compute_heuristic(int state_id, int hidx);
	unsigned int compute_key(ARAStarState *state, int hidx);
	void insert_or_update(ARAStarState *state, int hidx);
	void reorder_open();

	void extract_path(
		std::vector<int>& solution,
		std::vector<int>& action_ids,
		int& cost);
};

}  // namespace AMRA

#endif  // ARASTAR_HPP
