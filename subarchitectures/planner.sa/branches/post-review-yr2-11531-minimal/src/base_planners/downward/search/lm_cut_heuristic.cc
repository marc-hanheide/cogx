#include "lm_cut_heuristic.h"

#include "globals.h"
#include "operator.h"
#include "state.h"

#include <cassert>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>
using namespace std;

// construction and destruction
LandmarkCutHeuristic::LandmarkCutHeuristic(int _iteration_limit, bool use_cache)
    : Heuristic(use_cache), iteration_limit(_iteration_limit) {
}

LandmarkCutHeuristic::~LandmarkCutHeuristic() {
}

// initialization
void LandmarkCutHeuristic::initialize() {
    cout << "Initializing landmark cut heuristic..." << endl;

    // Build propositions.
    propositions.resize(g_variable_domain.size());
    for(int var = 0; var < g_variable_domain.size(); var++) {
        for(int value = 0; value < g_variable_domain[var]; value++)
            propositions[var].push_back(RelaxedProposition());
    }

    // Build relaxed operators for operators and axioms.
    for(int i = 0; i < g_operators.size(); i++)
        build_relaxed_operator(g_operators[i]);
    if(!g_axioms.empty()) {
        cerr << "error: LM-cut heuristic implementation does not "
             << "support axioms" << endl;
        ::exit(1);
    }

    // Simplify relaxed operators.
    // simplify();
    /* TODO: Put this back in and test if it makes sense,
       but only after trying out whether and how much the change to
       unary operators hurts. */

    // Build artificial goal proposition and operator.
    vector<RelaxedProposition *> goal_op_pre, goal_op_eff;
    for(int i = 0; i < g_goal.size(); i++) {
        int var = g_goal[i].first, val = g_goal[i].second;
        RelaxedProposition *goal_prop = &propositions[var][val];
        goal_op_pre.push_back(goal_prop);
    }
    goal_op_eff.push_back(&artificial_goal);
    add_relaxed_operator(goal_op_pre, goal_op_eff, 0, 0);

    // Cross-reference relaxed operators.
    for(int i = 0; i < relaxed_operators.size(); i++) {
        RelaxedOperator *op = &relaxed_operators[i];
        for(int j = 0; j < op->precondition.size(); j++)
            op->precondition[j]->precondition_of.push_back(op);
        for(int j = 0; j < op->effects.size(); j++)
            op->effects[j]->effect_of.push_back(op);
    }
}

void LandmarkCutHeuristic::build_relaxed_operator(const Operator &op) {
    int base_cost = op.get_cost();
    if(base_cost > 1000) {
        // HACK -- but doing it this way and failing noisily is better
        // than using this implementation for high action cost settings
        // accidentally.
        // TODO: Think about how to do this properly.
        cerr << "error: LM-cut heuristic implementation not suitable "
             << "for high action costs" << endl;
        ::exit(1);
    }
    const vector<Prevail> &prevail = op.get_prevail();
    const vector<PrePost> &pre_post = op.get_pre_post();
    vector<RelaxedProposition *> precondition;
    vector<RelaxedProposition *> effects;
    for(int i = 0; i < prevail.size(); i++)
        precondition.push_back(&propositions[prevail[i].var][prevail[i].prev]);
    for(int i = 0; i < pre_post.size(); i++) {
        const vector<Prevail> &cond = pre_post[i].cond;
        if(!cond.empty()) {
            // Accept conditions that are redundant, but nothing else.
            // In a better world, these would never be included in the
            // input in the first place.
            int var = pre_post[i].var;
            int post = pre_post[i].post;
            if(cond.size() != 1 || cond[0].var != var ||
               g_variable_domain[var] != 2 || cond[0].prev == post) {
                cerr << "Heuristic does not support conditional effects "
                     << "(operator " << g_operators[i].get_name() << ")"
                     << endl << "Terminating." << endl;
                exit(1);
            }
        }

        if(pre_post[i].pre != -1)
            precondition.push_back(&propositions[pre_post[i].var][
                                       pre_post[i].pre]);
        effects.push_back(&propositions[pre_post[i].var][pre_post[i].post]);

    }
    add_relaxed_operator(precondition, effects, &op, base_cost);
}

void LandmarkCutHeuristic::add_relaxed_operator(
    const vector<RelaxedProposition *> &precondition,
    const vector<RelaxedProposition *> &effects,
    const Operator *op, int base_cost) {
    RelaxedOperator relaxed_op(precondition, effects, op, base_cost);
    if(relaxed_op.precondition.empty())
        relaxed_op.precondition.push_back(&artificial_precondition);
    relaxed_operators.push_back(relaxed_op);
}

// heuristic computation
void LandmarkCutHeuristic::setup_exploration_queue() {
    reachable_queue.clear();

    for(int var = 0; var < propositions.size(); var++) {
        for(int value = 0; value < propositions[var].size(); value++) {
            RelaxedProposition &prop = propositions[var][value];
            prop.status = UNREACHED;
        }
    }

    artificial_goal.status = UNREACHED;
    artificial_precondition.status = UNREACHED;

    for(int i = 0; i < relaxed_operators.size(); i++) {
        RelaxedOperator &op = relaxed_operators[i];
        op.unsatisfied_preconditions = op.precondition.size();
        op.h_max_supporter = 0;
    }
}

void LandmarkCutHeuristic::setup_exploration_queue_state(const State &state) {
    for(int var = 0; var < propositions.size(); var++) {
        RelaxedProposition *init_prop = &propositions[var][state[var]];
        enqueue_if_necessary(init_prop, 0);
    }
    enqueue_if_necessary(&artificial_precondition, 0);
}

void LandmarkCutHeuristic::first_exploration(const State &state) {
    setup_exploration_queue();
    setup_exploration_queue_state(state);
    for(int bucket_no = 0; bucket_no < reachable_queue.size(); bucket_no++) {
        for(;;) {
            Bucket &bucket = reachable_queue[bucket_no];
            // NOTE: Cannot set "bucket" outside the loop because the
            //       reference can change if reachable_queue is
            //       resized.
            if(bucket.empty())
                break;
            RelaxedProposition *prop = bucket.back();
            bucket.pop_back();
            int prop_cost = prop->h_max_cost;
            assert(prop_cost <= bucket_no);
            if(prop_cost < bucket_no)
                continue;
            const vector<RelaxedOperator *> &triggered_operators =
                prop->precondition_of;
            for(int i = 0; i < triggered_operators.size(); i++) {
                RelaxedOperator *relaxed_op = triggered_operators[i];
                relaxed_op->unsatisfied_preconditions--;
                assert(relaxed_op->unsatisfied_preconditions >= 0);
                if(relaxed_op->unsatisfied_preconditions == 0) {
		    relaxed_op->h_max_supporter = prop;
		    int target_cost = prop_cost + relaxed_op->cost;
		    for(int j = 0; j < relaxed_op->effects.size(); j++) {
			RelaxedProposition *effect = relaxed_op->effects[j];
			enqueue_if_necessary(effect, target_cost);
		    }
                }
            }
        }
    }
}

void LandmarkCutHeuristic::first_exploration_incremental(
    vector<RelaxedOperator *> &cut) {
    // TODO: This implementation may not be very suitable for problems
    //       with action costs because of potential reexpansions; might
    //       use a priority queue instead. Needs testing.
    // TODO: We could probably integrate the h_max test with the h_max supporter
    //       update to avoid iterating through the operator preconditions too
    //       often.
    // TODO: Maybe it's worth storing h_max values in the actual operators.
    //       Or maybe not.
    vector<pair<int, RelaxedOperator *> > queue;
    for(int i = 0; i < cut.size(); i++) {
	RelaxedOperator *op = cut[i];
	queue.push_back(make_pair(op->h_max_cost() + op->cost, op));
    }
    while(!queue.empty()) {
	int cost = queue.back().first;
	RelaxedOperator *op = queue.back().second;
	queue.pop_back();
	for(int i = 0; i < op->effects.size(); i++) {
	    RelaxedProposition *prop = op->effects[i];
	    int old_prop_cost = prop->h_max_cost;
	    if(old_prop_cost > cost) {
		prop->h_max_cost = cost;
		/* Note: Instead of iterations over all operators of which
		   prop is a precondition, we only really want to iterate
		   over all operators of which prop is the h_max supporter.
		   Iterating over all instead may give us asymptotically
		   worse performence, but maintaining the extra data
		   structures to keep track of the best supporter relationship
		   is probably a waste of time in the common case of few
		   preconditions per operator.
		   TODO: Try this out. */
		for(int j = 0; j < prop->precondition_of.size(); j++) {
		    RelaxedOperator *next_op = prop->precondition_of[j];
		    if(next_op->h_max_supporter == prop) {
			int next_op_cost = next_op->h_max_cost();
			next_op->update_h_max_supporter();
			if(next_op_cost < old_prop_cost) {
			    queue.push_back(
				make_pair(next_op_cost + next_op->cost,
					  next_op));
			}
		    }
		}
	    }
        }
    }
}

void LandmarkCutHeuristic::second_exploration(
    const State &state, vector<RelaxedProposition *> &queue, vector<RelaxedOperator *> &cut) {
    assert(queue.empty());

    artificial_precondition.status = BEFORE_GOAL_ZONE;
    queue.push_back(&artificial_precondition);

    for(int var = 0; var < propositions.size(); var++) {
        RelaxedProposition *init_prop = &propositions[var][state[var]];
	init_prop->status = BEFORE_GOAL_ZONE;
	queue.push_back(init_prop);
    }

    while(!queue.empty()) {
	RelaxedProposition *prop = queue.back();
	queue.pop_back();
	const vector<RelaxedOperator *> &triggered_operators =
	    prop->precondition_of;
	for(int i = 0; i < triggered_operators.size(); i++) {
	    RelaxedOperator *relaxed_op = triggered_operators[i];
	    if(relaxed_op->h_max_supporter == prop) {
		bool reached_goal_zone = false;
		for(int j = 0; j < relaxed_op->effects.size(); j++) {
		    RelaxedProposition *effect = relaxed_op->effects[j];
		    if(effect->status == GOAL_ZONE) {
			assert(relaxed_op->cost > 0);
			reached_goal_zone = true;
			cut.push_back(relaxed_op);
			break;
		    }
		}
		if(!reached_goal_zone) {
		    for(int j = 0; j < relaxed_op->effects.size(); j++) {
			RelaxedProposition *effect = relaxed_op->effects[j];
			if(effect->status != BEFORE_GOAL_ZONE) {
			    assert(effect->status == REACHED);
			    effect->status = BEFORE_GOAL_ZONE;
			    queue.push_back(effect);
			}
		    }
		}
            }
        }
    }
}

void LandmarkCutHeuristic::mark_goal_plateau(RelaxedProposition *subgoal) {
    // NOTE: subgoal can be null if we got here via recursion through
    // a zero-cost action that is relaxed unreachable. (This can only
    // happen in domains which have zero-cost actions to start with.)
    // For example, this happens in pegsol-strips #01.
    if(subgoal && subgoal->status != GOAL_ZONE) {
        subgoal->status = GOAL_ZONE;
        const vector<RelaxedOperator *> &effect_of = subgoal->effect_of;
        for(int i = 0; i < effect_of.size(); i++)
            if(effect_of[i]->cost == 0)
                mark_goal_plateau(effect_of[i]->h_max_supporter);
    }
}

void LandmarkCutHeuristic::validate_h_max() const {
#ifndef NDEBUG
    for(int i = 0; i < relaxed_operators.size(); i++) {
	const RelaxedOperator *op = &relaxed_operators[i];
	const vector<RelaxedProposition *> prec = op->precondition;
	if(op->unsatisfied_preconditions) {
	    bool reachable = true;
	    for(int j = 0; j < prec.size(); j++)
		if(prec[j]->status == UNREACHED) {
		    reachable = false;
		    break;
		}
	    assert(!reachable);
	    assert(!op->h_max_supporter);
	} else {
	    assert(op->h_max_supporter);
	    int h_max_cost = op->h_max_supporter->h_max_cost;
	    for(int j = 0; j < prec.size(); j++) {
		assert(prec[j]->status != UNREACHED);
		assert(prec[j]->h_max_cost <= h_max_cost);
	    }
	}
    }
#endif
}

int LandmarkCutHeuristic::compute_heuristic(const State &state) {
    // TODO: Possibly put back in some kind of preferred operator mechanism.
    for(int i = 0; i < relaxed_operators.size(); i++) {
        RelaxedOperator &op = relaxed_operators[i];
        op.cost = op.base_cost * COST_MULTIPLIER;
    }

    //cout << "*" << flush;
    int total_cost = 0;

    // The following two variables could be declared inside the loop
    // ("queue" even inside second_exploration), but having them here
    // saves reallocations and hence provides a measurable speed
    // boost.
    vector<RelaxedOperator *> cut;
    vector<RelaxedProposition *> queue;
    first_exploration(state);
    // validate_h_max();
    if(artificial_goal.status == UNREACHED)
	return DEAD_END;

    int num_iterations = 0;
    while ((artificial_goal.h_max_cost != 0) && ((iteration_limit == -1) || (num_iterations < iteration_limit))) {
        num_iterations++;
        //cout << "h_max = " << artificial_goal.h_max_cost << "..." << endl;
        //cout << "total_cost = " << total_cost << "..." << endl;
        mark_goal_plateau(&artificial_goal);
        assert(cut.empty());
        second_exploration(state, queue, cut);
        assert(!cut.empty());
        int cut_cost = numeric_limits<int>::max();
        for(int i = 0; i < cut.size(); i++) {
            cut_cost = min(cut_cost, cut[i]->cost);
            // NOTE: The following line is only needed if COST_MULTIPLIER > 1
            cut_cost = min(cut_cost, cut[i]->base_cost);
        }
        for(int i = 0; i < cut.size(); i++)
            cut[i]->cost -= cut_cost;
	//cout << "{" << cut_cost << "}" << flush;
        total_cost += cut_cost;



	first_exploration_incremental(cut);
	// validate_h_max();
	// TODO: Need better name for all explorations; e.g. this could
	//       be "recompute_h_max"; second_exploration could be
	//       "mark_zones" or whatever.
        cut.clear();

	// TODO: Make this more efficient. For example, we can use
	//       a round-dependent counter for GOAL_ZONE and BEFORE_GOAL_ZONE,
	//       or something based on total_cost, in which case we don't
	//       need a per-round reinitialization.
	for(int var = 0; var < propositions.size(); var++) {
	    for(int value = 0; value < propositions[var].size(); value++) {
		RelaxedProposition &prop = propositions[var][value];
		if(prop.status == GOAL_ZONE || prop.status == BEFORE_GOAL_ZONE)
		    prop.status = REACHED;
	    }
	}
	artificial_goal.status = REACHED;
	artificial_precondition.status = REACHED;
    }
    //cout << "[" << total_cost << "]" << flush;
    //cout << "**************************" << endl;
    return (total_cost + COST_MULTIPLIER - 1) / COST_MULTIPLIER;
}

/* TODO:
   It looks like the change in r3638 reduced the quality of the heuristic
   a bit (at least a preliminary glance at Elevators-03 suggests that).
   The only arbitrary aspect is the tie-breaking policy in choosing h_max
   supporters, so maybe that is adversely affected by the incremental
   procedure? In that case, maybe we should play around with this a bit,
   e.g. use a different tie-breaking rule in every round to spread out the
   values a bit.
 */
