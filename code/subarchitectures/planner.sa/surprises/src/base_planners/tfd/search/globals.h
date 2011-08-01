#ifndef GLOBALS_H
#define GLOBALS_H

#include <iostream>
#include <string>
#include <vector>
#include <set>
using namespace std;

#define EPSILON 0.0001
#define EPS_TIME 0.001

#include "causal_graph.h"

class AxiomEvaluator;
class Cache;
//class CausalGraph;
class DomainTransitionGraph;
class Operator;
class Axiom;
class LogicAxiom;
class NumericAxiom;
class TimeStampedState;
class SuccessorGenerator;
// class ConsistencyCache;

typedef std::vector<TimeStampedState*> PlanTrace;

bool double_equals(double a, double b);

const int REALLYBIG = 9999999;
const int REALLYSMALL = -9999999;

void read_everything(istream &in);
void dump_everything();
void dump_DTGs();

void check_magic(istream &in, string magic);

enum variable_type {logical, primitive_functional, subterm_functional, comparison};
bool is_functional(int var);

extern int g_last_arithmetic_axiom_layer;
extern int g_comparison_axiom_layer;
extern int g_first_logic_axiom_layer;
extern int g_last_logic_axiom_layer;
extern vector<string> g_variable_name;
extern vector<int> g_variable_domain;
extern vector<int> g_axiom_layers;
extern vector<double> g_default_axiom_values;
extern vector<variable_type> g_variable_types;
extern TimeStampedState *g_initial_state;
extern vector<pair<int, double> > g_goal;
extern vector<Operator> g_operators;
extern vector<Axiom*> g_axioms;
extern AxiomEvaluator *g_axiom_evaluator;
extern SuccessorGenerator *g_successor_generator;
extern vector<DomainTransitionGraph *> g_transition_graphs;
extern CausalGraph *g_causal_graph;
extern Cache *g_cache;
extern int g_cache_hits, g_cache_misses;
extern bool g_greedy;
extern int g_timeout;
extern int g_hard_timeout;
extern string g_planMonitorFileName;

extern bool g_force_sequential;

extern Operator *g_let_time_pass;
extern Operator *g_wait_operator;

enum assignment_op {assign=0, scale_up=1, scale_down=2, increase=3, decrease=4};
enum binary_op {add=0, subtract=1, mult=2, divis=3, lt=4, le=5, eq=6, ge=7, gt=8, ue=9};
enum trans_type {start=0, end=1, compressed=2, ax=3};
enum condition_type {start_cond = 0, overall_cond = 1, end_cond = 2, ax_cond};

istream& operator>>(istream &is, assignment_op &aop);
ostream& operator<<(ostream &os, const assignment_op &aop);

istream& operator>>(istream &is, binary_op &bop);
ostream& operator<<(ostream &os, const binary_op &bop);

istream& operator>>(istream &is, trans_type &tt);
ostream& operator<<(ostream &os, const trans_type &tt);

istream& operator>>(istream &is, condition_type &fop);
ostream& operator<<(ostream &os, const condition_type &fop);

void printSet(const set<int> s);

struct PlanStep {
    double start_time;
    double duration;
    const Operator* op;

    PlanStep(double st, double dur, const Operator* o) :
	start_time(st), duration(dur), op(o) {}
};

typedef std::vector<PlanStep> Plan;

class SearchEngine {
private:
    bool solved;
    bool solved_at_least_once;
    Plan plan;
    PlanTrace path;
protected:
    enum {FAILED, SOLVED, IN_PROGRESS};
    virtual int step() = 0;

    void set_plan(const Plan &plan);
    void set_path(const PlanTrace &states);
public:
    SearchEngine();
    virtual ~SearchEngine();
    virtual void statistics(time_t & current_time) const;
    virtual void initialize() {}
    bool found_solution() const;
    bool found_at_least_one_solution() const;
    const Plan &get_plan() const;
    const PlanTrace& get_path() const;
    void search();
};

#endif
