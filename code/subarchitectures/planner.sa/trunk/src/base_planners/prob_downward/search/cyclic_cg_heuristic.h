#ifndef CYCLIC_CG_HEURISTIC_H
#define CYCLIC_CG_HEURISTIC_H

#include <vector>
#include <set>

#include "globals.h"
#include "heuristic.h"
#include "domain_transition_graph.h"

// #define USE_CONTEXT

class ValueTransitionLabel;
class State;

class LocalProblem;
class LocalProblemNode;
class CyclicCGHeuristic;

// TODO: Fix friend statements and access qualifiers.

enum {
    HEURISTIC_MODE_COST,
    HEURISTIC_MODE_TIME
};

class LocalTransition {
    friend class CyclicCGHeuristic;
    friend class LocalProblem;
    friend class LocalProblemNode;

    LocalProblemNode *source;
    LocalProblemNode *target;
    const ValueTransitionLabel *label;
    int action_duration;
    int action_cost;
    double action_prob;

    int target_cost;
    int target_action_costs;
    double target_prob;
    double min_prob;
    int max_cost;
    int unreached_conditions;

    LocalTransition(LocalProblemNode *source_, LocalProblemNode *target_,
                    const ValueTransitionLabel *label_, int action_cost_,
                    int duration_, double action_prob_);

    #ifdef USE_CONTEXT
    inline void get_context(std::vector<std::pair<int, int> >& result) const;
    #endif

    void on_source_expanded(const State &state);
    void on_condition_reached(int cost, double prob);
    void try_to_fire();
};


class LocalProblemNode {
    friend class CyclicCGHeuristic;
    friend class LocalProblem;
    friend class LocalTransition;

    // Static attributes (fixed upon initialization).
    LocalProblem *owner;
    std::vector<LocalTransition> outgoing_transitions;

    // Dynamic attributes (modified during heuristic computation).
    int cost;
    int action_cost;
    double prob;
    inline int priority() const;
    // Nodes have both a "cost" and a "priority", which are related.
    // The cost is an estimate of how expensive it is to reach this
    // node. The "priority" is the lowest bucket value in the overall
    // cost computation for which this node will be important. It is
    // essentially the sum of the cost and a local-problem-specific
    // "base priority", which depends on where this local problem is
    // needed for the overall computation.

    bool expanded;
    std::vector<short> children_state;
    int var;
    int value;

    LocalTransition *reached_by;
    // Before a node is expanded, reached_by is the "current best"
    // transition leading to this node. After a node is expanded, the
    // reached_by value of the parent is copied (unless the parent is
    // the initial node), so that reached_by is the *first* transition
    // on the optimal path to this node. This is useful for preferred
    // operators. (The two attributes used to be separate, but this
    // was a bit wasteful.)

    std::vector<LocalTransition *> waiting_list;

    LocalProblemNode(LocalProblem *owner, int children_state_size);
    void add_to_waiting_list(LocalTransition *transition);
    void on_expand();
    void mark_helpful_transitions(const State &state, int level=0);
    std::pair<int, double> compute_probability(const State &state, std::set<const Operator *>& ops);
};

#ifdef USE_CONTEXT
class LocalProblemTable {
    friend class CyclicCGHeuristic;
    int var;
    int gvar;
    vector<LocalProblemTable> children;
    LocalProblem* problem;
public:
    LocalProblemTable() : var(-1), problem(0) {};
    void add(LocalProblem* _problem, int var_no = 0);
    LocalProblem* get(const std::vector<int>& context, int var_no = 0);
};
#endif

class LocalProblem {
    friend class CyclicCGHeuristic;
    friend class LocalProblemTable;
    friend class LocalProblemNode;
    friend class LocalTransition;
    enum {QUITE_A_LOT = 1000000};

    int base_priority;

    std::vector<LocalProblemNode> nodes;

    std::vector<int> *causal_graph_parents;
    #ifdef USE_CONTEXT
    std::vector<int> *poss_add_context;
    std::vector<LocalAssignment> add_context;
    std::vector<int> context_values;

    void build_nodes_for_variable(int var_no, const std::vector<std::pair<int, int> >& context);
    #else
    void build_nodes_for_variable(int var_no);
    #endif

    void build_nodes_for_goal();
    inline bool is_initialized() const;
public:
    LocalProblem(int var_no = -1);
    LocalProblem(int var_no, const std::vector<std::pair<int, int> >& context );
    void initialize(int base_priority, int start_value, const State &state);

    #ifdef USE_CONTEXT
    static void context_key(int var_no, const std::vector<std::pair<int, int> >& context, std::vector<int>& result);
    #endif
};


class CyclicCGHeuristic : public Heuristic {
    friend class LocalProblem;
    friend class LocalProblemNode;
    friend class LocalTransition;

    std::vector<LocalProblem *> local_problems;
    #ifdef USE_CONTEXT
    std::vector<std::vector<LocalProblemTable *> > local_problem_index;
    #else
    std::vector<std::vector<LocalProblem *> > local_problem_index;
    #endif
    LocalProblem *goal_problem;
    LocalProblemNode *goal_node;

    std::vector<std::vector<LocalProblemNode *> > buckets;
    int heap_size;
    int mode;
    double p_heuristic;

    int compute_costs(const State &state);
    void initialize_heap();
    void add_to_heap(LocalProblemNode *node);

    #ifdef USE_CONTEXT
    inline LocalProblem *get_local_problem(int var_no, int value, const std::vector<std::pair<int, int> >& context);
    #else
    inline LocalProblem *get_local_problem(int var_no, int value);
    #endif
protected:
    virtual void initialize();
    virtual int compute_heuristic(const State &state);
public:
    CyclicCGHeuristic();
    ~CyclicCGHeuristic();
    virtual bool dead_ends_are_reliable() {return false;}
    virtual double get_p_heuristic() const {return p_heuristic;}

};


#ifdef USE_CONTEXT
inline void LocalTransition::get_context(std::vector<std::pair<int, int> >& result) const {
    int *parent_vars = &*source->owner->causal_graph_parents->begin();

    for (int i = 0; i < label->new_context.size(); i++) {
        int global_var_no = parent_vars[label->new_context[i].local_var];
        result.push_back(pair<int, int>(global_var_no, label->new_context[i].value));
        cout << "new context: " << label->new_context[i].local_var << "/"<< g_variable_name[global_var_no] << " = " << label->new_context[i].value << endl;
    }
    vector<LocalAssignment>::const_iterator c_it = source->owner->add_context.begin();
    vector<LocalAssignment>::const_iterator c_end = source->owner->add_context.end();
    for (; c_it != c_end; c_it++) {
        int global_var_no = parent_vars[c_it->local_var];
        result.push_back(pair<int, int>(global_var_no, c_it->value));
        cout << "old context: " << c_it->local_var << "/" << g_variable_name[global_var_no] << " = " << c_it->value << endl;
    }

}
#endif

inline int LocalProblemNode::priority() const {
    return cost + owner->base_priority;
}

inline bool LocalProblem::is_initialized() const {
    return base_priority != -1;
}

#ifdef USE_CONTEXT
inline LocalProblem *CyclicCGHeuristic::get_local_problem(int var_no, int value, const std::vector<std::pair<int, int> >& context) {
    LocalProblemTable *table = local_problem_index[var_no][value];
    if (!table) {
        table = new LocalProblemTable();
        local_problem_index[var_no][value] = table;
    }
    
    std::vector<int> key;
    LocalProblem::context_key(var_no, context, key);
    LocalProblem *result = table->get(key);
    if(!result) {
        result = new LocalProblem(var_no, context);
        table->add(result);
        local_problems.push_back(result);
    }
    return result;
}
#else
inline LocalProblem *CyclicCGHeuristic::get_local_problem(int var_no, int value) {
    LocalProblem *result = local_problem_index[var_no][value];
    if(!result) {
        result = new LocalProblem(var_no);
        local_problem_index[var_no][value] = result;
        local_problems.push_back(result);
    }
    return result;
}
#endif



#endif
