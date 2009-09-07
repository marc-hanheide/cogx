#ifndef CYCLIC_CG_HEURISTIC_H
#define CYCLIC_CG_HEURISTIC_H

#include <vector>
#include <queue>
#include <tr1/unordered_map>
#include "heuristic.h"
#include "state.h"
#include "domain_transition_graph.h"
#include <ext/slist>
#include <cmath>

class FuncTransitionLabel;
class TimeStampedState;

class LocalProblemDiscrete;
class LocalProblemComp;
class LocalProblemNodeDiscrete;
class LocalProblemNodeComp;
class LocalProblemNode;
class CyclicCGHeuristic;
class LocalProblem;
class LocalTransitionComp;
class LocalTransitionDiscrete;
class LocalTransition;
class Node_compare;

typedef priority_queue<LocalProblemNode*, std::vector<LocalProblemNode*>,
        Node_compare> node_queue;

typedef __gnu_cxx ::slist<std::pair<LocalTransition*, int> > waiting_list_t;
typedef waiting_list_t::const_iterator const_it_waiting_list;
typedef waiting_list_t::iterator it_waiting_list;

typedef std::tr1::unordered_map<int, int> hashmap;
typedef hashmap::value_type ValuePair;

//*****************************************************
//general classes
//*****************************************************
class LocalTransition {
public:
    double target_cost;
    int duration_var_local;
    const ValueTransitionLabel *label;
    virtual LocalProblemNode* get_source() = 0;
    virtual void on_condition_reached(int, double) = 0;
    virtual void print_description() = 0;
    LocalTransition(ValueTransitionLabel *the_label) :
        label(the_label) {
    }
    virtual ~LocalTransition() {
    }
    double get_direct_cost();
};

class LocalProblemNode {
public:
    // Static attributes
    LocalProblem *owner;

    std::vector<double> children_state;

    LocalTransition *reached_by;

    LocalTransition *pred;

    // Dynamic attributes (modified during heuristic computation).
    double cost;
    inline double priority() const;
    bool expanded;
    double reached_by_wait_for;
    virtual void dump() {
    }

    virtual void on_expand(const TimeStampedState &state) = 0;

    waiting_list_t waiting_list;

    void updatePrimitiveNumericVariable(assignment_op a_op,
            int primitive_var_local, int influencing_var_local,
            vector<double> &temp_children_state);
    void
            updateNumericVariablesRec(int var,
                    vector<double> &temp_children_state);
    void updateSubtermNumericVariables(int var, binary_op op, int left_var,
            int right_var, vector<double> &temp_children_state);
    void updateComparisonVariables(int var, binary_op op, int left_var,
            int right_var, vector<double> &temp_children_state);

    bool all_conds_satiesfied(const ValueTransitionLabel *label, const TimeStampedState &state);
    void mark_helpful_transitions(const TimeStampedState &state);
    virtual ~LocalProblemNode() {
    }
    LocalProblemNode(LocalProblem* owner, int children_state_size) :
        owner(owner) {
        children_state.resize(children_state_size, 0.0);
        cost = -1.0;
    }
    void add_to_waiting_list(LocalTransition *transition, int start_val);
    virtual void print_name() = 0;
};

class LocalProblem {
public:
    enum {
        QUITE_A_LOT = 10000000
    };
    double base_priority;
    const int var_no;
    std::vector<int> *causal_graph_parents;
    const int start_value;
    virtual LocalProblemNode* get_node(int var_no) = 0;
    hashmap *global_to_local_parents;
    vector<vector<int> > depending_vars;
    vector<vector<int> > children_in_cg;
    int getLocalIndexOfGlobalVariable(int global_var);
    void buildDependingVars(int parents_num);
    void extract_subplan(LocalProblemNode* goalNode, string indent, vector<pair<Operator*, double> >& needed_ops, const TimeStampedState& state);
    virtual ~LocalProblem() {
    }
    virtual void initialize(double base_priority, int start_value,
            const TimeStampedState &state) = 0;
    inline bool is_initialized() const;
    LocalProblem(int the_var_no, int the_start_value);
};

//*****************************************************
//discrete variables
//*****************************************************
class LocalProblemNodeDiscrete: public LocalProblemNode {
public:

    // Static attributes (fixed upon initialization).
    std::vector<LocalTransitionDiscrete> outgoing_transitions;
    std::vector<LocalTransitionDiscrete> additional_outgoing_transitions;

    int value;

    LocalProblemNodeDiscrete(LocalProblemDiscrete *owner,
            int children_state_size, int value);
    virtual void on_expand(const TimeStampedState &state);
    void dump();
    void print_name();
};

class LocalTransitionDiscrete: public LocalTransition {
public:

    LocalProblemNodeDiscrete *source;
    LocalProblemNodeDiscrete *target;

    LocalProblemNodeDiscrete* get_source() {
        return source;
    }

    int unreached_conditions;

    LocalTransitionDiscrete(ValueTransitionLabel *the_label,
            LocalProblemNodeDiscrete *the_source,
            LocalProblemNodeDiscrete *the_target) :
        LocalTransition(the_label), source(the_source), target(the_target) {
    }

    void on_source_expanded(const TimeStampedState &state);
    virtual void on_condition_reached(int cond_no, double cost);
    void try_to_fire();
    virtual void print_description();
};

class LocalProblemDiscrete: public LocalProblem {
public:

    std::vector<LocalProblemNodeDiscrete> nodes;
    virtual LocalProblemNodeDiscrete* get_node(int var_no) {
        return &(nodes[var_no]);
    }

    void build_nodes_for_variable(int var_no);
    void build_nodes_for_goal();
    void compile_DTG_arcs_to_LTD_objects(DomainTransitionGraphSymb *dtgs);
    LocalProblemDiscrete(int var_no, int start_val);
    virtual void initialize(double base_priority, int start_value,
            const TimeStampedState &state);
};

//*****************************************************
//comparison case
//*****************************************************
class LocalProblemNodeComp: public LocalProblemNode {
public:

    std::vector<LocalTransitionComp> outgoing_transitions;

    vector<vector<pair<LocalProblemNode*, int> > >
            nodes_where_this_subscribe;

    int value;

    binary_op op;

    bool opened;
    LocalTransitionComp* bestTransition;

    LocalProblemNodeComp(LocalProblemComp *owner_, int children_state_size,
            int the_value, binary_op the_binary_op);
    void fire(LocalTransitionComp* trans);
    virtual void on_expand(const TimeStampedState &state);
    void expand(LocalTransitionComp* trans);
    bool is_satiesfied(int trans_index, LocalTransitionComp* trans,
            const TimeStampedState &state);
    bool is_directly_satiesfied(const LocalAssignment &pre_cond);
    void subscribe_to_waiting_lists();
    void updateNumericVariables(LocalTransitionComp &trans,
            vector<double> &temp_children_state);
    bool check_progress_of_transition(vector<double> &temp_children_state, LocalTransitionComp *trans);
    void dump();
    void print_name();
};

class LocalTransitionComp: public LocalTransition {
public:

    LocalProblemNodeComp* source;

    LocalProblemNodeComp* target;

    LocalProblemNodeComp* get_source() {
        return source;
    }

    vector<bool> conds_satiesfied;

    LocalTransitionComp(FuncTransitionLabel *the_label,
            LocalProblemNodeComp *the_source, LocalProblemNodeComp *the_target) :
        LocalTransition(the_label), source(the_source), target(the_target) {
        target_cost = 0.0;
        //TODO: is this necessary?
        conds_satiesfied.resize(label->precond.size());
    }

    virtual void on_condition_reached(int cond_no, double cost);
    virtual void print_description();
};

class LocalProblemComp: public LocalProblem {
public:

    //nodes[0] = false, nodes[1] = true
    std::vector<LocalProblemNodeComp> nodes;
    LocalProblemNodeComp* get_node(int var_no) {
        return &(nodes[var_no]);
    }

    //inherited member variable start_value always has start_value 0 (true) or 1 (false)

//    binary_op comp_op;

    void build_nodes_for_variable(int var_no, int the_start_value);

    LocalProblemComp(int var_no, int start_val);
    virtual void initialize(double base_priority, int start_value,
            const TimeStampedState &state);
};

//*****************************************************
//CyclicCGHeuristic
//*****************************************************

class Node_compare {
public:
    bool operator()(const LocalProblemNode* ln, const LocalProblemNode* rn) const {
        return (rn->priority() < ln->priority());
    }
};

class CyclicCGHeuristic: public Heuristic {
public:

    std::vector<LocalProblem *> local_problems;
    std::vector<std::vector<LocalProblem *> > local_problem_index;
    LocalProblemDiscrete *goal_problem;
    LocalProblemNodeDiscrete *goal_node;

    node_queue open_nodes;

    int number_of_nodes_in_queue;

    vector<LocalProblemNodeDiscrete*> nodes_with_an_additional_transition;
    vector<ValueNode*> dtg_nodes_with_an_additional_transition;
    vector<Operator*> generated_waiting_ops;
    vector<ValueTransitionLabel*> generated_labels;

    bool is_running(LocalTransition* trans, const TimeStampedState& state);
    double compute_costs(const TimeStampedState &state);
    void initialize_queue();
    void add_to_queue(LocalProblemNode *node);
    LocalProblemNode* remove_from_queue();
    void build_transitions_for_running_ops(const TimeStampedState &state);
    void remove_transitions_for_running_ops(const TimeStampedState &state);

    inline LocalProblem *get_local_problem(int var_no, int value);

    virtual void initialize();
    virtual double compute_heuristic(const TimeStampedState &state);

    CyclicCGHeuristic();
    ~CyclicCGHeuristic();
    virtual bool dead_ends_are_reliable() {
        return false;
    }
};

//*****************************************************
//inline functions
//*****************************************************
inline double LocalProblemNode::priority() const {
    return cost + owner->base_priority;
}

inline bool LocalProblem::is_initialized() const {
    return base_priority != -1;
}

inline LocalProblem *CyclicCGHeuristic::get_local_problem(int var_no,
        int value) {
    LocalProblem *result = local_problem_index[var_no][value];
    if(!result) {
        if(g_variable_types[var_no] == comparison) {
            result = new LocalProblemComp(var_no, value);
        } else {
            assert(g_variable_types[var_no] == logical);
            result = new LocalProblemDiscrete(var_no, value);
        }
        local_problem_index[var_no][value] = result;
        local_problems.push_back(result);
    }
    return result;
}

#endif
