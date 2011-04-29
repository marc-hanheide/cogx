#ifndef BEST_FIRST_SEARCH_H
#define BEST_FIRST_SEARCH_H

#include <sys/time.h>
#include <vector>
#include <queue>
#include "closed_list.h"
#include "search_engine.h"
#include "state.h"
#include "operator.h"
#include <tr1/tuple>

class Heuristic;

typedef std::tr1::tuple<const TimeStampedState *, const Operator *, double> OpenListEntry;

class OpenListEntryCompare {
public:
    bool operator()(const OpenListEntry left_entry, const OpenListEntry right_entry) const {
	return std::tr1::get<2>(right_entry) < std::tr1::get<2>(left_entry);
    }
};

typedef priority_queue<OpenListEntry,std::vector<OpenListEntry>,OpenListEntryCompare> OpenList;

struct OpenListInfo {
    OpenListInfo(Heuristic *heur, bool only_pref);
    Heuristic *heuristic;
    bool only_preferred_operators;
//    OpenList<OpenListEntry> open;
    OpenList open;
    int priority; // low value indicates high priority
};



class BestFirstSearchEngine : public SearchEngine {
    std::vector<Heuristic *> heuristics;
    std::vector<Heuristic *> preferred_operator_heuristics;
    std::vector<OpenListInfo> open_lists;
    ClosedList closed_list;

    std::vector<double> best_heuristic_values;
    std::vector<const TimeStampedState*> best_states;
    int generated_states;

    TimeStampedState current_state;
    const TimeStampedState *current_predecessor;
    const Operator *current_operator;

    bool is_dead_end();
    bool check_goal();
    bool check_progress(const TimeStampedState* state);
    void report_progress();
    void reward_progress();
    void generate_successors(const TimeStampedState *parent_ptr);
    void dump_transition() const;
    OpenListInfo *select_open_queue();

    void dump_plan_prefix_for_current_state() const;
    void dump_plan_prefix_for__state(const TimeStampedState &state) const;

    timeval start_time;
    timeval last_time;
    timeval last_improvement_time;
protected:
    virtual int step();
public:
    BestFirstSearchEngine();
    ~BestFirstSearchEngine();
    void add_heuristic(Heuristic *heuristic, bool use_estimates,
	    bool use_preferred_operators);
    virtual void statistics(timeval & current_time) const;
    virtual void initialize();
    int fetch_next_state();
};

#endif
