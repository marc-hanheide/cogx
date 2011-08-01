#ifndef FF_HEURISTIC_H
#define FF_HEURISTIC_H

#include "relaxation_heuristic.h"

#include <ext/hash_set>
#include <queue>

class less_costs {
    public:
    bool operator() (Proposition* p1, Proposition* p2) { return p1->h_add_cost > p2->h_add_cost; }
};

class FFHeuristic : public RelaxationHeuristic {
    typedef __gnu_cxx::hash_set<const Operator *, hash_operator_ptr> RelaxedPlan;
    typedef std::priority_queue<Proposition*, std::vector<Proposition*>, less_costs> PropositionQueue;
    
    //Proposition **reachable_queue_start;
    //Proposition **reachable_queue_read_pos;
    //Proposition **reachable_queue_write_pos;
    PropositionQueue queue;

    void setup_exploration_queue();
    void setup_exploration_queue_state(const State &state);
    void relaxed_exploration();

    void collect_relaxed_plan(Proposition *goal, RelaxedPlan &relaxed_plan);

    int compute_hsp_add_heuristic();
    int compute_ff_heuristic();
protected:
    virtual void initialize();
    virtual int compute_heuristic(const State &state);
public:
    FFHeuristic(bool use_cache=false);
    ~FFHeuristic();
};

#endif
