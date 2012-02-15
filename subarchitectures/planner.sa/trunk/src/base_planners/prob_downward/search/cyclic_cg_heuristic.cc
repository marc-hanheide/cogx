#include "cyclic_cg_heuristic.h"

#include "domain_transition_graph.h"
#include "globals.h"
#include "operator.h"
#include "state.h"
#include "search_space.h"

#include <algorithm>
#include <cassert>
#include <vector>
using namespace std;

/*
TODO: The responsibilities between the different classes need to be
      divided more clearly. For example, the LocalProblem
      initialization code contains some stuff (related to initial node
      initialization) that may better fit into the LocalProblemNode
      class. It's probably best to have all the code in the
      CyclicCGHeuristic class and have everything else be PODs.
      This would also get rid of g_HACK.
 */

CyclicCGHeuristic *g_HACK = 0;

inline void CyclicCGHeuristic::add_to_heap(LocalProblemNode *node) {
    int bucket_no = node->priority();
    if(bucket_no >= buckets.size()) {
        buckets.resize(max<size_t>(bucket_no + 1, 2 * buckets.size()));
        std::cout << "resize: " << buckets.size() << std::endl;
    }
    // std::cout << "push: " << bucket_no << " - " << heap_size+1  << std::endl;
    buckets[bucket_no].push_back(node);
    ++heap_size;
}

LocalTransition::LocalTransition(
    LocalProblemNode *source_, LocalProblemNode *target_,
    const ValueTransitionLabel *label_, int action_cost_, int duration_, double action_prob_) {
    source = source_;
    target = target_;
    label = label_;
    action_cost = action_cost_;
    action_prob = action_prob_;
    action_duration = duration_;

    // if (label->op)
    //     std::cout << "local transition: "<< label->op->get_name() << " " << action_cost << std::endl;
    // else
    //     std::cout << "local transition: (no op) " << action_cost << std::endl;

    // Set the following to explicitly invalid values.
    // They are initialized in on_source_expanded.
    target_cost = -1;
    target_action_costs = -1;
    target_prob = -1;
    min_prob = 1.0;
    max_cost = 0;
    unreached_conditions = -1;
}

inline void LocalTransition::try_to_fire() {
    if(!unreached_conditions && target_cost < target->cost) {
        target->action_cost = target_action_costs + max_cost;
        target->prob = target_prob * min_prob;
        target->cost = target->action_cost + (1-target->prob) * g_reward + 0.5;
        target->reached_by = this;
        if (g_debug && label && label->op)
            cout << "    firing: " << label->op->get_name() << " cost: " << target->cost << endl;
        g_HACK->add_to_heap(target);
    }
}

void LocalTransition::on_source_expanded(const State &state) {
    /* Called when the source of this transition is reached by
       Dijkstra exploration. Tries to compute cost for the target of
       the transition from the source cost, action cost, and set-up
       costs for the conditions on the label. The latter may yet be
       unknown, in which case we "subscribe" to the waiting list of
       the node that will tell us the correct value. */

    assert(source->cost >= 0);
    assert(source->cost < LocalProblem::QUITE_A_LOT);

    if (g_HACK->mode == HEURISTIC_MODE_COST) {
        target_action_costs = source->action_cost + action_cost;
        target_prob = source->prob * action_prob;
    } else {
        target_action_costs = source->action_cost + action_duration;
        target_prob = source->prob * 1;
    }

    target_cost = target_action_costs + (1-target_prob) * g_reward + 0.5;
    min_prob = 1.0;
    max_cost = 0;
    // target_cost = source->cost + action_cost + (1-action_prob) * g_reward * g_multiplier;

    if(target->cost <= target_cost) {
        // Transition cannot find a shorter path to target.
        return;
    }


    short *children_state = &source->children_state.front();
    int *parent_vars = &*source->owner->causal_graph_parents->begin();

#ifdef USE_CONTEXT
    vector<vector<LocalProblemTable *> > &problem_index = g_HACK->local_problem_index;
    vector<pair<int, int> > context;
    get_context(context);
#else
    vector<vector<LocalProblem *> > &problem_index = g_HACK->local_problem_index;
#endif

    unreached_conditions = 0;
    const vector<LocalAssignment> &precond = label->precond;
    if (g_debug && label->op) {
        std::cout << "open:" << label->op->get_name()  << " - " << target_cost << " / " << target_prob << " / " << min_prob << std::endl;
        if (source->reached_by) {
            const ValueTransitionLabel* l = source->reached_by->label;
            std::cout << "    reached by:" << l->op->get_name()  << " - " << source->cost << " / " << source->prob << std::endl;
        }
    }

    vector<LocalAssignment>::const_iterator
        curr_precond = precond.begin(),
        last_precond = precond.end();

    for(; curr_precond != last_precond; ++curr_precond) {
        int local_var = curr_precond->local_var;
        int current_val = children_state[local_var];
        // double current_prob = ((double) children_state[local_var].prob) / (1 << 15);
        int precond_value = curr_precond->value;
        int precond_var_no = parent_vars[local_var];
        
        if(current_val == precond_value) {
            continue;
        }

#ifdef USE_CONTEXT
        LocalProblemTable *&table = problem_index[precond_var_no][current_val];
        LocalProblem* child_problem = 0;
        // cout << "table is: " << g_variable_name[precond_var_no] << "  " << precond_var_no << " = " << current_val << endl ;
        if (!table) {
            table = new LocalProblemTable();
        }
        else {
            std::vector<int> key;
            LocalProblem::context_key(precond_var_no, context, key);
            child_problem = table->get(key);
        }
        
        if(!child_problem) {
            child_problem = new LocalProblem(precond_var_no, context);
            table->add(child_problem);
            g_HACK->local_problems.push_back(child_problem);
            if (g_debug)
                cout << "    new subproblem: " << g_variable_name[precond_var_no] << " = " << precond_value << endl;
        }
        else {
            if (g_debug)
                cout << "    old subproblem: " << g_variable_name[precond_var_no] << " = " << precond_value << endl;
        }
#else
        LocalProblem *&child_problem = problem_index[precond_var_no][current_val];
        if(!child_problem) {
            child_problem = new LocalProblem(precond_var_no);
            g_HACK->local_problems.push_back(child_problem);
            if (g_debug)
                cout << "    new subproblem: " << g_variable_name[precond_var_no] << " = " << precond_value << "(now: " << current_val << ")" << endl;
        }
        else if (g_debug)
            cout << "    old subproblem: " << g_variable_name[precond_var_no] << " = " << precond_value << "(now: " << current_val << ")" << endl;

#endif

        if(!child_problem->is_initialized())
            child_problem->initialize(source->priority(), current_val, state);
        LocalProblemNode *cond_node = &child_problem->nodes[precond_value];
        if(cond_node->expanded) {
            // target_cost = target_cost + target_prob * (cond_node->cost + (1-cond_node->prob) * g_multiplier * g_reward);
            // target_action_costs += cond_node->action_cost;
            if (g_debug && label->op) {
                double new_target_prob = target_prob * cond_node->prob;
                int new_target_cost = target_action_costs + (1-target_prob) * g_reward +0.5;
                const ValueTransitionLabel* l = cond_node->reached_by->label;
                std::cout << "    already expanded:" << l->op->get_name()  << " - " << cond_node->cost << " / " << cond_node->prob << std::endl;
                std::cout << "    now:" << label->op->get_name()  << " - " << target_cost << " / " << target_prob << " ---> " << new_target_prob << " / " << new_target_cost  << std::endl;
            }
            min_prob = min(min_prob, cond_node->prob);
            max_cost = max(max_cost, cond_node->action_cost);
            // max_cost += cond_node->action_cost;
            // target_prob *= cond_node->prob;
            target_cost = target_action_costs + max_cost + (1-target_prob*min_prob) * g_reward + 0.5;

            if(target->cost <= target_cost) {
                // Transition cannot find a shorter path to target.
                return;
            }
        } else {
            cond_node->add_to_waiting_list(this);
            ++unreached_conditions;
        }
    }
    try_to_fire();
}

void LocalTransition::on_condition_reached(int cost, double prob) {
    assert(unreached_conditions);
    --unreached_conditions;
    // target_action_costs += cost;
    // target_cost = target_cost + target_prob * (cost + (1-prob) * g_multiplier * g_reward);
    if (g_debug && label->op) {
        double new_min_prob = min(min_prob, prob);
        double new_max_cost = max(max_cost, cost);
        // double new_max_cost = max_cost + cost;
        int new_target_cost = target_action_costs + new_max_cost + (1-target_prob*new_min_prob) * g_reward + 0.5;
        std::cout << "    condition reached: " << label->op->get_name() << " - " << cost << " / " << prob << "  remaining: " << unreached_conditions << std::endl;
        //std::cout << "    " << target_action_costs << " - " << new_target_prob << std::endl;
        std::cout << "    update: " << target_cost << " / " << (target_action_costs - cost) << " / " << target_prob*min_prob << " ---> " << new_target_cost << " / " << target_action_costs << " / " << target_prob * new_min_prob  << std::endl;
        std::cout << "    " << target_prob << " / " << min_prob << " / " << new_min_prob << endl;
    }
    // target_prob *= prob;
    min_prob = min(min_prob, prob);
    max_cost = max(max_cost, cost);
    // max_cost += cost;
    target_cost = target_action_costs + max_cost + (1-target_prob*min_prob) * g_reward + 0.5;
    // std::cout << "target " << std::endl;
    try_to_fire();
}

LocalProblemNode::LocalProblemNode(LocalProblem *owner_,
                                   int children_state_size) {
    owner = owner_;
    action_cost = -1;
    var = -1;
    value = -1;
    cost = -1;
    prob = 1.0;
    expanded = false;
    reached_by = 0;
    children_state.resize(children_state_size, -1);
}

void LocalProblemNode::add_to_waiting_list(LocalTransition *transition) {
    waiting_list.push_back(transition);
}

void LocalProblemNode::on_expand() {
    expanded = true;
    // Set children state unless this was an initial node.
    if(reached_by) {
        LocalProblemNode *parent = reached_by->source;
        children_state = parent->children_state;
#ifdef USE_CONTEXT       
        const vector<LocalAssignment> &context = parent->owner->add_context;
        for(int i = 0; i < context.size(); i++)
            children_state[context[i].local_var] = context[i].value;
#endif
        const vector<LocalAssignment> &precond = reached_by->label->precond;
        for(int i = 0; i < precond.size(); i++)
            children_state[precond[i].local_var] = precond[i].value;
        const vector<LocalAssignment> &effect = reached_by->label->effect;
        for(int i = 0; i < effect.size(); i++)
            children_state[effect[i].local_var] = effect[i].value;
        if(parent->reached_by)
            reached_by = parent->reached_by;
    }
    for(int i = 0; i < waiting_list.size(); i++)
        waiting_list[i]->on_condition_reached(action_cost, prob);
    waiting_list.clear();
}


#ifdef USE_CONTEXT       
void LocalProblemTable::add(LocalProblem* _problem, int var_no) {
    assert ((var == var_no) || (var == -1));
    if (var_no < _problem->context_values.size()) {
        // gvar = (*(_problem->poss_add_context))[var_no];
        // cout << "add: " << var << "/" << gvar<< " ("<< g_variable_name[gvar]<<")"<< endl; 
        if (var == -1) {
            var = var_no;
            children.resize(g_variable_domain[(*(_problem->poss_add_context))[var_no]] + 1);
        }
        int pos = _problem->context_values[var_no];
        if (pos == -1) 
            pos = children.size()-1;
        children[pos].add(_problem, var_no + 1);
    }
    else {
        problem = _problem;
    }
}

LocalProblem* LocalProblemTable::get(const std::vector<int>& context, int var_no) {
    assert(var == var_no || var == -1);
    if (var == -1) {
        return problem;
    }
    assert(var_no < context.size());
    int pos = context[var_no];
    if (pos == -1) 
        pos = children.size()-1;
    assert(pos < children.size());
    return children[pos].get(context, var_no + 1);
}

void LocalProblem::context_key(int var_no, const std::vector<std::pair<int, int> >& context, std::vector<int>& result) {
    DomainTransitionGraph *dtg = g_transition_graphs[var_no];

    result.resize(dtg->oneshot_parents.size());
    fill(result.begin(), result.end(), -1);
    for(int i = 0; i < context.size(); i++) {
        // cout << context[i].first << "/"<< dtg->oneshot_to_local[context[i].first] << "  ";
        if (dtg->ccg_to_oneshot[context[i].first] != -1) {
            result[dtg->ccg_to_oneshot[context[i].first]] = context[i].second;
            // cout << "context key:" << g_variable_name[context[i].first] << "/ "<< context[i].first << " = " << context[i].second << endl;
        }
    }

}
#endif

#ifdef USE_CONTEXT       
void LocalProblem::build_nodes_for_variable(int var_no, const std::vector<std::pair<int, int> >& context) {
#else
void LocalProblem::build_nodes_for_variable(int var_no) {
#endif
    DomainTransitionGraph *dtg = g_transition_graphs[var_no];

    causal_graph_parents = &dtg->ccg_parents;

    int num_parents = causal_graph_parents->size();
    for(int value = 0; value < g_variable_domain[var_no]; value++)
        nodes.push_back(LocalProblemNode(this, num_parents));

#ifdef USE_CONTEXT       
    poss_add_context = &dtg->oneshot_parents;
    context_values.resize(poss_add_context->size());
    fill(context_values.begin(), context_values.end(), -1);
    for(int i = 0; i < context.size(); i++) {
        // cout << context[i].first << "/"<< dtg->oneshot_to_local[context[i].first] << "  ";
        if (dtg->ccg_to_oneshot[context[i].first] != -1) {
            add_context.push_back(LocalAssignment(dtg->ccg_to_local[context[i].first], context[i].second));
            context_values[dtg->ccg_to_oneshot[context[i].first]] = context[i].second;
            // cout << "add context:" << dtg->ccg_to_local[context[i].first] << "/"<< g_variable_name[context[i].first] << " = " << context[i].second << endl;
            // cout << context[i].first << " / " << dtg->ccg_to_local[context[i].first] << " / " << dtg-> ccg_parents[dtg->ccg_to_local[context[i].first]] << endl;
        }
    }
    // cout << endl;
#endif

    // Compile the DTG arcs into LocalTransition objects.
    for(int value = 0; value < nodes.size(); value++) {
        LocalProblemNode &node = nodes[value];
        node.var = var_no;
        node.value = value;
        const ValueNode &dtg_node = dtg->nodes[value];
        for(int i = 0; i < dtg_node.transitions.size(); i++) {
            const ValueTransition &dtg_trans = dtg_node.transitions[i];
            int target_value = dtg_trans.target->value;
            LocalProblemNode &target = nodes[target_value];
            for(int j = 0; j < dtg_trans.ccg_labels.size(); j++) {
                const ValueTransitionLabel &label = dtg_trans.ccg_labels[j];
#ifdef USE_CONTEXT       
                if (!label.is_applicable(add_context)) {
                    cout << "pruned transition "  << label.op->get_name() << endl;
                    continue;
                }
#endif
                // int action_cost = dtg->is_axiom ? 0 : label.op->get_cost() + (g_reward * 0.5 * label.op->get_p_cost()) / g_multiplier;
                int action_cost = dtg->is_axiom ? 0 : label.op->get_cost();
                int action_time = dtg->is_axiom ? 0 : label.op->get_duration();
                // std::cout << "build node: " << label.op->get_name() << " cost: " << label.op->get_cost() << " + " <<  g_reward << " * 0.5 * " << label.op->get_p_cost() << " = " << action_cost << std::endl;
                double action_prob = dtg->is_axiom ? 1.0 : label.op->get_prob();
                LocalTransition trans(&node, &target, &label, action_cost, action_time, action_prob);
                node.outgoing_transitions.push_back(trans);
            }
        }
    }
}

void LocalProblem::build_nodes_for_goal() {
    // TODO: We have a small memory leak here. Could be fixed by
    // making two LocalProblem classes with a virtual destructor.
    causal_graph_parents = new vector<int>;
    for(int i = 0; i < g_goal.size(); i++)
        causal_graph_parents->push_back(g_goal[i].first);

    for(int value = 0; value < 2; value++)
        nodes.push_back(LocalProblemNode(this, g_goal.size()));

    vector<LocalAssignment> goals;
    for(int goal_no = 0; goal_no < g_goal.size(); goal_no++) {
        int goal_value = g_goal[goal_no].second;
        goals.push_back(LocalAssignment(goal_no, goal_value));
    }
    vector<LocalAssignment> no_effects;
    ValueTransitionLabel *label = new ValueTransitionLabel(0, goals, no_effects);
    LocalTransition trans(&nodes[0], &nodes[1], label, 0, 0, 1.0);
    nodes[0].outgoing_transitions.push_back(trans);
}

LocalProblem::LocalProblem(int var_no) {
    base_priority = -1;
    if(var_no == -1)
        build_nodes_for_goal();
    else
#ifdef USE_CONTEXT
        build_nodes_for_variable(var_no, vector<pair<int, int> >());
#else
        build_nodes_for_variable(var_no);
#endif
}

#ifdef USE_CONTEXT       
LocalProblem::LocalProblem(int var_no, const std::vector<std::pair<int, int> >& context ) {
    base_priority = -1;
    build_nodes_for_variable(var_no, context);
}
#endif

void LocalProblem::initialize(int base_priority_, int start_value,
                              const State &state) {
    assert(!is_initialized());
    base_priority = base_priority_;

    for(int to_value = 0; to_value < nodes.size(); to_value++) {
        nodes[to_value].expanded = false;
        nodes[to_value].cost = QUITE_A_LOT;
        nodes[to_value].action_cost = QUITE_A_LOT;
        nodes[to_value].prob = 1.0;
        nodes[to_value].waiting_list.clear();
    }

    LocalProblemNode *start = &nodes[start_value];
    // start->prob = start_prob;
    start->cost = 0;
    start->action_cost = 0;
    start->prob = 1.0;
    for(int i = 0; i < causal_graph_parents->size(); i++)
        start->children_state[i] = state[(*causal_graph_parents)[i]];

    g_HACK->add_to_heap(start);
}

void LocalProblemNode::mark_helpful_transitions(const State &state, int level) {
    string indent;
    indent.resize(level*2, ' ');
    assert(cost >= 0 && cost < LocalProblem::QUITE_A_LOT);
    if(reached_by) {
        if(reached_by->target_action_costs + reached_by->max_cost == reached_by->action_cost) {
            // Transition applicable, all preconditions achieved.
            const Operator *op = reached_by->label->op;
            assert(!op->is_axiom());
            assert(op->is_applicable(state));
            if (g_debug)
                std::cout << indent << "helpful: " << op->get_name() << " / " << op->get_prob() << std::endl;
            g_HACK->set_preferred(op);
        } else {
            if (g_debug && reached_by->label->op)
                std::cout << indent << "indirectly helpful: " << reached_by->label->op->get_name() << " / " << reached_by->label->op->get_prob() << std::endl;

#ifdef USE_CONTEXT       
            vector<pair<int, int> > context;
            reached_by->get_context(context);
#endif
            // Recursively compute helpful transitions for precondition variables.
            const vector<LocalAssignment> &precond = reached_by->label->precond;
            int *parent_vars = &*owner->causal_graph_parents->begin();
            for(int i = 0; i < precond.size(); i++) {
                int precond_value = precond[i].value;
                int local_var = precond[i].local_var;
                int precond_var_no = parent_vars[local_var];
                if(state[precond_var_no] == precond_value)
                    continue;
#ifdef USE_CONTEXT       
                LocalProblemNode *child_node = &g_HACK->get_local_problem(
                    precond_var_no, state[precond_var_no], context)->nodes[precond_value];
#else
                LocalProblemNode *child_node = &g_HACK->get_local_problem(
                    precond_var_no, state[precond_var_no])->nodes[precond_value];
#endif
                child_node->mark_helpful_transitions(state, level+1);
            }
        }
    }
}

std::pair<int, double> LocalProblemNode::compute_probability(const State &state, std::set<const Operator *>& ops) {
    assert(cost >= 0 && cost < LocalProblem::QUITE_A_LOT);
    double p_min = 1.0;
    int c_max = 0;
    if(reached_by) {
        const Operator *op = reached_by->label->op;
        if (op && ops.find(op) == ops.end())
            ops.insert(op);

        // if(reached_by->target_action_costs + reached_by->max_cost == reached_by->action_cost) {
        // // if (op && ops.find(op) == ops.end())
        // //     ops.insert(op);
        //     // Transition applicable, all preconditions achieved.
        // } else {
            // Recursively compute helpful transitions for precondition variables.
#ifdef USE_CONTEXT       
            vector<pair<int, int> > context;
            reached_by->get_context(context);
#endif
            const vector<LocalAssignment> &precond = reached_by->label->precond;
            int *parent_vars = &*owner->causal_graph_parents->begin();
            for(int i = 0; i < precond.size(); i++) {
                int precond_value = precond[i].value;
                int local_var = precond[i].local_var;
                int precond_var_no = parent_vars[local_var];
                if(state[precond_var_no] == precond_value)
                    continue;
#ifdef USE_CONTEXT       
                LocalProblemNode *child_node = &g_HACK->get_local_problem(
                    precond_var_no, state[precond_var_no], context)->nodes[precond_value];
#else
                LocalProblemNode *child_node = &g_HACK->get_local_problem(
                    precond_var_no, state[precond_var_no])->nodes[precond_value];
#endif
                pair<int, double> r = child_node->compute_probability(state, ops);
                //cout << "    " << p_child << endl;
                c_max = max(c_max, r.first);
                p_min = min(p_min, r.second);
            // }
        }
        if (op) {
            //cout << op->get_prob() << " / " << op->get_prob() * p_min << endl;
            return pair<int, double>(op->get_cost() + c_max, op->get_prob() * p_min);
        }
    }
    return pair<int, double>(c_max, p_min);
}

CyclicCGHeuristic::CyclicCGHeuristic() {
    assert(!g_HACK);
    g_HACK = this;
    goal_problem = 0;
    goal_node = 0;
    heap_size = -1;
    mode = HEURISTIC_MODE_COST;
}

CyclicCGHeuristic::~CyclicCGHeuristic() {
    delete goal_problem;
    for(int i = 0; i < local_problems.size(); i++)
        delete local_problems[i];
}

void CyclicCGHeuristic::initialize() {
    assert(goal_problem == 0);
    cout << "Initializing cyclic causal graph heuristic..." << endl;

    int num_variables = g_variable_domain.size();

    goal_problem = new LocalProblem;
    goal_node = &goal_problem->nodes[1];

    local_problem_index.resize(num_variables);
    for(int var_no = 0; var_no < num_variables; var_no++) {
        int num_values = g_variable_domain[var_no];
        local_problem_index[var_no].resize(num_values, 0);
    }
}

int CyclicCGHeuristic::compute_heuristic(const State &state) {
    if (g_use_deadline) {
        int current_time = 0;
        if (einfo != NULL) {
            current_time = einfo->get_t();
            // if (einfo->op != 0) {
            //     cout << "created by: " << einfo->op->get_name() << endl; 
            // }
        }

        // int heuristic;
        mode = HEURISTIC_MODE_TIME;
        initialize_heap();
        goal_problem->base_priority = -1;
        for(int i = 0; i < local_problems.size(); i++)
            local_problems[i]->base_priority = -1;
        goal_problem->initialize(0, 0, state);

        int time_h = compute_costs(state);
        if (time_h == DEAD_END || time_h + current_time > g_deadline) {
            // cout << "deadline pruned: " << time_h << " + " << current_time  << endl;
            return DEAD_END;
        }
    }
    // cout << "not pruned: " << heuristic << " + " << current_time << endl;
    

    mode = HEURISTIC_MODE_COST;
    initialize_heap();
    goal_problem->base_priority = -1;
    for(int i = 0; i < local_problems.size(); i++)
        local_problems[i]->base_priority = -1;
    goal_problem->initialize(0, 0, state);
    int heuristic = compute_costs(state);
    // cout << "h: " << heuristic << endl;
    // cout << heuristic << endl;

    // if(heuristic != DEAD_END && heuristic != 0) {
    //     double p_init = 1.0;
    //     if (einfo != NULL) {
    //         p_init = einfo->get_p();
    //     }
    //     goal_node->mark_helpful_transitions(state);
    //     std::set<const Operator *> ops;
    //     pair<int, double> r = goal_node->compute_probability(state, ops);
    //     int cost = r.first;
    //     double p = r.second;

    //     if (g_debug)
    //         cout << "p: " << p << ", c: " << cost << endl;
    //     // cout << cost << " / "  << p << " - " << p_init * p <<  " // " << cost + (1-p) * g_reward << endl;
    //     heuristic = cost + p_init * (1-p) * g_reward;
    // }

    if (g_debug) {
        cout << "h = " << heuristic << endl;
        exit(0);
    }

    return heuristic;
}

void CyclicCGHeuristic::initialize_heap() {
    /* This was just "buckets.clear()", but it may be advantageous to
       keep the empty buckets around so that there are fewer
       reallocations. At least changing this from buckets.clear() gave
       a significant speed boost (about 7%) for depots #10 on alfons.
    */
    for(int i = 0; i < buckets.size(); i++)
        buckets[i].clear();
    heap_size = 0;
}

int CyclicCGHeuristic::compute_costs(const State &state) {
    double p_init = 1.0;
    if (einfo != NULL) {
        p_init = einfo->get_p();
    }
    for(int curr_priority = 0; heap_size != 0; curr_priority++) {
        assert(curr_priority < buckets.size());
        for(int pos = 0; pos < buckets[curr_priority].size(); pos++) {
            // std::cout << curr_priority  << " - " << pos << " - " << heap_size << std::endl;
            LocalProblemNode *node = buckets[curr_priority][pos];
            if (g_debug) {
                if (node->var != -1)
                    std::cout << curr_priority << ": pop: " << g_variable_name[node->var] << " = " << node->value <<  " - " << node->cost << " / " << node->prob << " / " << node->action_cost << std::endl;
                else
                    std::cout << curr_priority << ": pop: " <<  " - " << node->cost << " / " << node->prob << " / " << node->action_cost << std::endl;
                if (node->reached_by && node->reached_by->label->op )
                    std::cout << "         reached by: " << node->reached_by->label->op->get_name()<< std::endl;

            }
            assert(node->owner->is_initialized());
            if(node->priority() < curr_priority)
                continue;
            if(node == goal_node) {
                if (g_debug) {
                    cout << "cea:" << node->prob << "    " << node->cost << endl;
                }
                if (node->cost == 0)
                    return 0;
                
                // cout << node->action_cost << " / "  << node->prob << " - " << p_init * node->prob <<  " // " << node->action_cost + (1-node->prob) * g_reward << endl;
                return node->action_cost + p_init * (1-node->prob) * g_reward;
                // return 1 + (1-node->prob) * g_reward;
            }

            assert(node->priority() == curr_priority);
            node->on_expand();
            for(int i = 0; i < node->outgoing_transitions.size(); i++)
                node->outgoing_transitions[i].on_source_expanded(state);
        }
        heap_size -= buckets[curr_priority].size();
        buckets[curr_priority].clear();
    }
    return DEAD_END;
}
