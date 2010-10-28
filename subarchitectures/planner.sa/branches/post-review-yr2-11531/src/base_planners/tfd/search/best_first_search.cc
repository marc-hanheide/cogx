#include "best_first_search.h"

#include "globals.h"
#include "heuristic.h"
#include "successor_generator.h"

#include <cassert>
using namespace std;

OpenListInfo::OpenListInfo(Heuristic *heur, bool only_pref) {
    heuristic = heur;
    only_preferred_operators = only_pref;
    priority = 0;
}

BestFirstSearchEngine::BestFirstSearchEngine() :
    current_state(*g_initial_state) {
    generated_states = 0;
    current_predecessor = 0;
    current_operator = 0;
    gettimeofday(&start_time, NULL);
    last_time = start_time;
    last_improvement_time = start_time;
}

BestFirstSearchEngine::~BestFirstSearchEngine() {
}

void BestFirstSearchEngine::add_heuristic(Heuristic *heuristic,
	bool use_estimates, bool use_preferred_operators) {

    // cout << "Adding heuristic" << endl;

    assert(use_estimates || use_preferred_operators);
    heuristics.push_back(heuristic);
    best_heuristic_values.push_back(-1);
    best_states.push_back(NULL);
    if(use_estimates) {
	open_lists.push_back(OpenListInfo(heuristic, false));
	open_lists.push_back(OpenListInfo(heuristic, true));
    }
    if(use_preferred_operators)
	preferred_operator_heuristics.push_back(heuristic);
}

void BestFirstSearchEngine::initialize() {
    assert(!open_lists.empty());
}

void BestFirstSearchEngine::statistics(timeval & current_time) const {
    cout << endl;
    cout << "Search Time: " << (current_time.tv_sec - start_time.tv_sec) << " sec." << endl;
    cout << "Expanded Nodes: " << closed_list.size() << " state(s)." << endl;
    cout << "Generated Nodes: " << generated_states << " state(s)." << endl;
    cout << "Best heuristic value: " << best_heuristic_values[0] << endl;
    cout << "Best state:" << endl;
    const TimeStampedState &state = *best_states[0];
    if(&state) {
        dump_plan_prefix_for__state(state);
        best_states[0]->dump();
    }
    cout << endl;
}

void BestFirstSearchEngine::dump_transition() const {
    cout << endl;
    if(current_predecessor != 0) {
	cout << "DEBUG: In step(), current predecessor is: " << endl;
	current_predecessor->dump();
    }
    cout << "DEBUG: In step(), current operator is: ";
    if(current_operator != 0) {
	current_operator->dump();
    } else {
	cout << "No operator before initial state." << endl;
    }
    cout << "DEBUG: In step(), current state is: " << endl;
    current_state.dump();
    cout << endl;
}

int BestFirstSearchEngine::step() {

    // Invariants:
    // - current_state is the next state for which we want to compute the heuristic.
    // - current_predecessor is a permanent pointer to the predecessor of that state.
    // - current_operator is the operator which leads to current_state from predecessor.

    bool discard = true;
	if (closed_list.contains(current_state)) {
		double min_so_far = closed_list.get_min_ts_of_key(current_state);
		double diff = current_state.timestamp - min_so_far;
		if (diff + EPSILON < 0)
			discard = false;
	} else {
		discard = false;
	}

    if(!discard) {
	const TimeStampedState *parent_ptr = closed_list.insert(current_state,
		current_predecessor, current_operator);
//    	cout << "Prefix:" << endl;
//    	if(current_predecessor)
//    		dump_plan_prefix_for__state(*current_predecessor);
//    	cout << "-" << endl;
//    	current_state.dump();
	for(int i = 0; i < heuristics.size(); i++)
	    heuristics[i]->evaluate(current_state);
	if(!is_dead_end()) {
	    if(check_progress(parent_ptr)) {
		// current_state.dump();
		report_progress();
		reward_progress();
	    }
	    if(check_goal())
		return SOLVED;
	    generate_successors(parent_ptr);
	}
    }

//    statistics();
//    cout << "State was: " << endl;
//    cout << "-" << endl;
//    dump_plan_prefix_for__state(*current_predecessor);
//    cout << "-" << endl;
//    current_state.dump();
//    cout << "---------------------------------------" << endl << endl << endl;

    timeval current_time;
    gettimeofday(&current_time, NULL);
    int elapsed = (current_time.tv_sec - start_time.tv_sec) * 1000 + (current_time.tv_usec - start_time.tv_usec) / 1000; 
    if(g_hard_timeout > 0 && elapsed > g_hard_timeout) {
	exit(0);
    }
    if(found_at_least_one_solution() && g_timeout > 0) {
        int impr_elapsed = (current_time.tv_sec - last_improvement_time.tv_sec) * 1000 + (current_time.tv_usec - last_improvement_time.tv_usec) / 1000; 
        if (impr_elapsed > g_timeout) {
            exit(0);
        }
    }
    if(current_time.tv_sec - last_time.tv_sec >= 10) {
        statistics(current_time);
        last_time = current_time;
    }
    return fetch_next_state();
}

bool BestFirstSearchEngine::is_dead_end() {
    // If a reliable heuristic reports a dead end, we trust it.
    // Otherwise, all heuristics must agree on dead-end-ness.
    int dead_end_counter = 0;
    for(int i = 0; i < heuristics.size(); i++) {
	if(heuristics[i]->is_dead_end()) {
	    if(heuristics[i]->dead_ends_are_reliable())
		return true;
	    else
		dead_end_counter++;
	}
    }
    return dead_end_counter == heuristics.size();
}

bool BestFirstSearchEngine::check_goal() {
    // Any heuristic reports 0 iff this is a goal state, so we can
    // pick an arbitrary one.
    Heuristic *heur = open_lists[0].heuristic;
    if(!heur->is_dead_end() && heur->get_heuristic() == 0) {

	assert(current_state.operators.empty() && current_state.satisfies(g_goal));

	// We actually need this silly !heur->is_dead_end() check because
	// this state *might* be considered a non-dead end by the
	// overall search even though heur considers it a dead end
	// (e.g. if heur is the CG heuristic, but the FF heuristic is
	// also computed and doesn't consider this state a dead end.
	// If heur considers the state a dead end, it cannot be a goal
	// state (heur will not be *that* stupid). We may not call
	// get_heuristic() in such cases because it will barf.
	Plan plan;
        PlanTrace path;
	closed_list.trace_path(current_state, plan, path);
	set_plan(plan);
	set_path(path);
    gettimeofday(&last_improvement_time, NULL);
	return true;
    } else {
	return false;
    }
}

void BestFirstSearchEngine::dump_plan_prefix_for_current_state() const {
    dump_plan_prefix_for__state(current_state);
}

void BestFirstSearchEngine::dump_plan_prefix_for__state(const TimeStampedState &state) const {
    Plan plan;
    PlanTrace path;
    closed_list.trace_path(state, plan, path);
    for(int i = 0; i < plan.size(); i++) {
	const PlanStep& step = plan[i];
	cout << step.start_time << ": "<< "(" << step.op->get_name() << ")" << " ["  << step.duration << "]"<< endl;
    }
}

bool BestFirstSearchEngine::check_progress(const TimeStampedState* state) {
    bool progress = false;
    for(int i = 0; i < heuristics.size(); i++) {
	if(heuristics[i]->is_dead_end())
	    continue;
	double h = heuristics[i]->get_heuristic();
	double &best_h = best_heuristic_values[i];
	if(best_h == -1 || h < best_h) {
	    best_h = h;
	    best_states[i] = state;
	    progress = true;
	}
    }
    return progress;
}

void BestFirstSearchEngine::report_progress() {
    cout << "Best heuristic value: ";
    for(int i = 0; i < heuristics.size(); i++) {
	cout << best_heuristic_values[i];
	if(i != heuristics.size() - 1)
	    cout << "/";
    }
    cout << " [expanded " << closed_list.size() << " state(s)]" << endl;
}

void BestFirstSearchEngine::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    // progress is made. This used to be used in multi-heuristic mode
    // only, but it is also useful in single-heuristic mode, at least
    // in Schedule.
    //
    // TODO: Test the impact of this, and find a better way of rewarding
    // successful exploration. For example, reward only the open queue
    // from which the good state was extracted and/or the open queues
    // for the heuristic for which a new best value was found.

    for(int i = 0; i < open_lists.size(); i++)
	if(open_lists[i].only_preferred_operators)
	    open_lists[i].priority -= 1000;
}

void BestFirstSearchEngine::generate_successors(
	const TimeStampedState *parent_ptr) {
    vector<const Operator *> all_operators;
    g_successor_generator->generate_applicable_ops(current_state, all_operators);

    vector<const Operator *> preferred_operators;
    for(int i = 0; i < preferred_operator_heuristics.size(); i++) {
	Heuristic *heur = preferred_operator_heuristics[i];
	if(!heur->is_dead_end()) {
	    heur->get_preferred_operators(preferred_operators);
	}
    }

//    cout << "Preferred ops: " << endl;
//    for(int i = 0; i < preferred_operators.size(); ++i) {
//    	cout << preferred_operators[i]->get_name() << endl;
//    }
//
//    cout << "............................." << endl;

    for(int i = 0; i < open_lists.size(); i++) {
	Heuristic *heur = open_lists[i].heuristic;
	if(!heur->is_dead_end()) {
	    double h = heur->get_heuristic();
	    double val = 0;
	    if(g_greedy) {
	        val = h;
	    } else {
	        val = h + parent_ptr->timestamp;
	    }
	    OpenList &open = open_lists[i].open;
	    vector<const Operator *>
		    &ops =
			    open_lists[i].only_preferred_operators ? preferred_operators
				    : all_operators;

	    for(int j = 0; j < ops.size(); j++) {
		assert(ops[j]->get_name().compare("wait") != 0);
		if(ops[j]->is_applicable(*parent_ptr)) {
			open.push(std::tr1::make_tuple(parent_ptr, ops[j], val));
		    generated_states++;
		}
	    }

	    // there exists another happening in the future, so
	    // we can let time pass until then, which is represented
	    // here as a NULL pointer

	    //		cout << "Debug: Inserting dummy operator in open list." << endl;

	    // Only add a no-op if letting time pass does not
	    // lead to an inconsistent state. This test is necessary for
	    // the following reason: Assume that the current state does
	    // not yet satisfy the over-all condition of an operator
	    // scheduled to start in the current state. Then we have
	    // to schedule another operator for application right now
	    // which has an at-start effect asserting this over-all
	    // condition. Therefore, although the current state CAN
	    // be a prefix of a valid plan, the NO-OP operator is
	    // not applicable in the current state. This issue can be
	    // checked by calling parent_ptr->is_consistent_when_progressed().
	    //		cout << "DEBUG: Calling is_consistent_when_progressed() from generate_successors()." << endl;
	    assert(parent_ptr->is_consistent_when_progressed());
	    open.push(std::tr1::make_tuple(parent_ptr, g_let_time_pass, val));
	    generated_states++;
	}
    }
}

int BestFirstSearchEngine::fetch_next_state() {
    OpenListInfo *open_info = select_open_queue();
    if(!open_info) {
	cout << "Completely explored state space -- no solution!" << endl;
	exit(0);
	return FAILED;
    }

    std::tr1::tuple<const TimeStampedState *, const Operator *, double> next =
	open_info->open.top();
    open_info->open.pop();
    open_info->priority++;

    current_predecessor = std::tr1::get<0>(next);
    current_operator = std::tr1::get<1>(next);

//    cout << "---------------------------" << endl;
//    cout << "current_predecessor: ";
//
//    cout << "performed actions: " << endl;
//    dump_plan_prefix_for_current_state();
//    current_predecessor->dump();
//
//
//    cout << "current_operator: ";
//    current_operator->dump();

    if(current_operator == g_let_time_pass) {
	// do not apply an operator but rather let some time pass until
	// next scheduled happening
	current_state = current_predecessor->let_time_pass();
    } else {
        assert(current_operator->get_name().compare("wait") != 0);
	current_state = TimeStampedState(*current_predecessor,
		*current_operator);
    }
    return IN_PROGRESS;
}

OpenListInfo *BestFirstSearchEngine::select_open_queue() {
    OpenListInfo *best = 0;
    for(int i = 0; i < open_lists.size(); i++) {
	if(!open_lists[i].open.empty() && (best == 0 || open_lists[i].priority < best->priority))
	    best = &open_lists[i];
    }
    if (best == 0) {
    	assert(open_lists[0].open.empty() && open_lists[1].open.empty());
    }
    return best;
}
