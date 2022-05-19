#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

// project includes
#include <amra/types.hpp>
#include <amra/heuristic.hpp>

// system includes
#include <smpl/console/console.h>
#include <smpl/heap/intrusive_heap.h>
#include <smpl/types.h>

// standard includes

namespace AMRA
{

struct AbstractState
{
	int state_id;
	unsigned int g;
	DiscState coord;
	AbstractState* bp;

	struct HeapData : public smpl::heap_element
	{
		// TODO: rather than map back to the state, the heap could know its
		// index into od for updates, though that might make it hard to
		// overallocate an array for the heap index, h, and f

		// TODO: in any case, this offset can be much smaller
		AbstractState* me;
		unsigned int h;
		unsigned int f;
	};

	bool closed;
	HeapData od[1]; // overallocated for additional n heuristics
};

inline
bool operator==(const AbstractState& a, const AbstractState& b)
{
    return (a.coord == b.coord);
}

} // namespace AMRA

namespace std
{

template <>
struct hash<AMRA::AbstractState>
{
    typedef AMRA::AbstractState argument_type;
    typedef std::size_t result_type;
    result_type operator()(const argument_type& s) const;
};

} // namespace std


namespace AMRA
{

class MovingAI;

class Dijkstra : public Heuristic
{
public:
	Dijkstra(Environment* space, MovingAI* map);
	void Init(const DiscState& robot, const DiscState& goal) override;

	unsigned int GetGoalHeuristic(int state_id) override;

	unsigned int GetStartHeuristic(int state_id) override {
		SMPL_ERROR("Dijkstra does not implement GetStartHeuristic");
	};
	unsigned int GetFromToHeuristic(int from_id, int to_id) override {
		SMPL_ERROR("Dijkstra does not implement GetFromToHeuristic");
	};

private:
	MovingAI* m_map;
	DiscState m_start, m_goal;
	int m_start_id, m_goal_id;

	struct HeapCompare {
		bool operator()(
				const AbstractState::HeapData& s,
				const AbstractState::HeapData& t) const
		{
			return s.f < t.f;
		}
	};

	using OpenList = smpl::intrusive_heap<AbstractState::HeapData, HeapCompare>;
	OpenList* m_open = nullptr;  // sequence of (m_heur_count + 1) open lists

	// maps from coords to stateID
    typedef AbstractState StateKey;
    typedef smpl::PointerValueHash<StateKey> StateHash;
    typedef smpl::PointerValueEqual<StateKey> StateEqual;
    smpl::hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;

    std::vector<AbstractState*> m_states;

    bool resume(int state_id);

    int getOrCreateState(DiscState& s);
	int getHashEntry(DiscState& s);
	int createHashEntry(const DiscState& s);
	AbstractState* getHashEntry(int state_id) const;
	int reserveHashEntry();

	void insert_or_update(AbstractState *state);

	unsigned int euclidean_dist(const DiscState& s);
	unsigned int cost(const DiscState& a, const DiscState& b);
};

}  // namespace AMRA

#endif  // DIJKSTRA_HPP
