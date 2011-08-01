#include "globals.h"
#include "operator.h"

#include <iostream>
using namespace std;

Prevail::Prevail(istream &in) {
    in >> var >> prev;
}

bool Prevail::is_applicable(const TimeStampedState &state) const {
    assert(var >= 0 && var < g_variable_name.size());
    assert(prev >= 0 && prev < g_variable_domain[var]);
    return double_equals(state[var],prev);
}

PrePost::PrePost(istream &in) {
    int cond_count;
    in >> cond_count;
    for(int i = 0; i < cond_count; i++)
	cond_start.push_back(Prevail(in));
    in >> cond_count;
    for(int i = 0; i < cond_count; i++)
	cond_overall.push_back(Prevail(in));
    in >> cond_count;
    for(int i = 0; i < cond_count; i++)
	cond_end.push_back(Prevail(in));
    in >> var;
    if(is_functional(var)) {
	in >> fop >> var_post;
	// HACK: just use some arbitrary values for pre and post
	// s.t. they do not remain uninitialized
	pre = post = -1;
    } else {
	in >> pre >> post;
	// HACK: just use some arbitrary values for var_post and fop
	// s.t. they do not remain uninitialized
	var_post = -1;
	fop = assign;
    }
}

bool PrePost::is_applicable(const TimeStampedState &state) const {
    assert(var >= 0 && var < g_variable_name.size());
    assert(pre == -1 || (pre >= 0 && pre < g_variable_domain[var]));
    return pre == -1 || (double_equals(state[var],pre));
}

Operator::Operator(istream &in) {
    check_magic(in, "begin_operator");
    in >> ws;
    getline(in, name);
    int count;
    binary_op bop;
    in >> bop >> duration_var;
    if(bop != eq) {
	cout << "Error: The duration constraint must be of the form\n";
	cout << "       (= ?duration (arithmetic_term))" << endl;
	exit(1);
    }

    in >> count; //number of prevail at-start conditions
    for(int i = 0; i < count; i++)
	prevail_start.push_back(Prevail(in));
    in >> count; //number of prevail overall conditions
    for(int i = 0; i < count; i++)
	prevail_overall.push_back(Prevail(in));
    in >> count; //number of prevail at-end conditions
    for(int i = 0; i < count; i++)
	prevail_end.push_back(Prevail(in));
    in >> count; //number of pre_post_start conditions (symbolical)
    for(int i = 0; i < count; i++)
	pre_post_start.push_back(PrePost(in));
    in >> count; //number of pre_post_end conditions (symbolical)
    for(int i = 0; i < count; i++)
	pre_post_end.push_back(PrePost(in));
    in >> count; //number of pre_post_start conditions (functional)
    for(int i = 0; i < count; i++)
	pre_post_start.push_back(PrePost(in));
    in >> count; //number of pre_post_end conditions (functional)
    for(int i = 0; i < count; i++)
	pre_post_end.push_back(PrePost(in));
    check_magic(in, "end_operator");
}

Operator::Operator(bool uses_concrete_time_information) {
    prevail_start   = vector<Prevail>();
    prevail_overall = vector<Prevail>();
    prevail_end     = vector<Prevail>();
    pre_post_start  = vector<PrePost>();
    pre_post_end    = vector<PrePost>();
    if(!uses_concrete_time_information) {
        name = "let_time_pass";
        duration_var = -1;
    } else {
	name = "wait";
	duration_var = -2;
    }
}

void Prevail::dump() const {
    cout << var << ": " << prev;
}

void PrePost::dump() const {
    cout << "var: " << g_variable_name[var] << ", pre: " << pre << " , var_post: " << var_post << ", post: " << post << endl;
}

void Operator::dump() const {
    cout << name;
    cout << endl;
}

bool Operator::is_applicable(const TimeStampedState &state) const {


    if(state[duration_var] <= 0) {
	return false;
    }

    if (g_force_sequential && !state.operators.empty()) {
        return false;
    }

    for(int i = 0; i < prevail_start.size(); i++)
	if(!prevail_start[i].is_applicable(state))
	    return false;
    for(int i = 0; i < pre_post_start.size(); i++)
	if(!pre_post_start[i].is_applicable(state))
	    return false;

    // Make sure that there is no other operator currently running, that
    // has the same end timepoint as this operator would have.
    //for(int i = 0; i < state.scheduled_effects.size(); i++) {
    //if(double_equals(state.scheduled_effects[i].time_increment,
    //    state[duration_var])) {
    //    return false;
    //}
    //}

    // There may be no simultaneous applications of two instances of the
    // same ground operator (for technical reasons, to simplify the task
    // of keeping track of durations committed to at the start of the
    // operator application)
    for(int i = 0; i < state.operators.size(); i++)
	if(state.operators[i].get_name() == get_name())
	    return false;

    return TimeStampedState(state,*this).is_consistent_when_progressed();
}

bool Operator::operator<(const Operator &other) const {
    return name < other.name;
}
