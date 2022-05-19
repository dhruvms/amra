#ifndef AMRA_HPP
#define AMRA_HPP

// project includes
#include <amra/types.hpp>

// system includes
#include <smpl/heap/intrusive_heap.h>

// standard includes
#include <vector>
#include <memory>

namespace AMRA
{

struct AMRAState
{
	int call_number;
	int state_id;
	unsigned int g;
	Resolution::Level res;
	AMRAState* bp;
	int actionidx;

	bool closed_in_anc;
	bool closed_in_res[3]; // overallocated for each resolution

	struct HeapData : public smpl::heap_element
	{
		// TODO: rather than map back to the state, the heap could know its
		// index into od for updates, though that might make it hard to
		// overallocate an array for the heap index, h, and f

		// TODO: in any case, this offset can be much smaller
		AMRAState* me;
		unsigned int h;
		unsigned int f;
	};
	HeapData od[1]; // overallocated for additional n heuristics
};

class Environment;
class Heuristic;

class AMRAStar : public Search
{
public:
	AMRAStar(
		Environment* space,
		const std::vector<std::shared_ptr<Heuristic>>& heurs,
		const std::vector<std::pair<Resolution::Level, int> >& heurs_map,
		int heur_count, int res_count);
	~AMRAStar();

	int set_start(int start_id) override;
	int set_goal(int goal_id) override;
	void set_max_planning_time(double max_planning_time_ms) override {
		m_time_limit = max_planning_time_ms * 1e-3;
	};
	int get_n_expands() const override;
	std::string get_expands_str() const;
	void reset() override;

	int replan(
		std::vector<int>* solution_path,
		std::vector<int>* action_ids,
		int* solution_cost) override;

private:
	Environment* m_space = nullptr;

	std::vector<std::shared_ptr<Heuristic>> m_heurs;
	std::vector<std::pair<Resolution::Level, int> > m_heurs_map;
	int m_heur_count, m_res_count;

	struct HeapCompare {
		bool operator()(
				const AMRAState::HeapData& s,
				const AMRAState::HeapData& t) const
		{
			return s.f < t.f;
		}
	};

	using OpenList = smpl::intrusive_heap<AMRAState::HeapData, HeapCompare>;
	OpenList* m_open = nullptr;  // sequence of (m_heur_count + 1) open lists

	// Search params
	int m_call_number, m_iter, m_offset;
	double m_time_limit;
	double m_w1_i, m_w1_f, m_w2_i, m_w2_f, m_w1, m_w2;
	double m_w1_delta, m_w2_delta;
	AMRAState *m_goal, *m_start;

	int m_start_id, m_goal_id;

	std::vector<AMRAState*> m_states, m_incons;

	// Search statistics
	double m_search_time;
	int *m_expands; // expansions per queue
	int m_w1_solve, m_w2_solve;
	int m_solution_cost;

	int num_heuristics() const { return m_heurs_map.size(); }

	AMRAState* get_state(int state_id);
	// AMRAState* create_state(int state_id);
	void init_state(AMRAState *state, int state_id);
	void reinit_state(AMRAState *state);

	bool improve_path(
		const double& start_time,
		double& elapsed_time);
	void expand(AMRAState *state, int hidx);
	bool is_goal(int state_id);

	unsigned int compute_heuristic(int state_id, int hidx);
	unsigned int compute_key(AMRAState *state, int hidx);
	void insert_or_update(AMRAState *state, int hidx);
	void reorder_open();

	void extract_path(
		std::vector<int>& solution,
		std::vector<int>& action_ids,
		int& cost);
};

}  // namespace AMRA

#endif  // AMRA_HPP
