#ifndef LANDMARKS_LANDMARKS_COUNT_HEURISTIC_H
#define LANDMARKS_LANDMARKS_COUNT_HEURISTIC_H

#include "../state.h"
#include "../heuristic.h"
#include "landmarks_graph.h"
#include "../best_first_search.h"
#include "exploration.h"
#include "landmark_status_manager.h"
#include "landmark_cost_assignment.h"

extern LandmarksGraph *g_lgraph; // Make global so graph does not need to be built more than once
// even when iterating search (TODO: clean up use of g_lgraph vs.
// lgraph in this class).



class LandmarksCountHeuristic: public Heuristic {

    friend class LamaFFSynergy;
    Exploration* exploration;
    LandmarksGraph &lgraph;
    bool use_preferred_operators;
    int lookahead;
    bool ff_search_disjunctive_lms;

    lm_set goal;
    LandmarkSet initial_state_landmarks;
    LandmarkStatusManager lm_status_manager;
    LandmarkCostAssignment *lm_cost_assignment;

    bool use_dynamic_cost_sharing;
    bool use_shared_cost;
    bool use_action_landmark_count;

    int get_heuristic_value(const State& state);

    void collect_lm_leaves(bool disjunctive_lms, LandmarkSet& result, vector<
            pair<int, int> >& leaves);
    int ff_search_lm_leaves(bool disjunctive_lms, const State& state,
            LandmarkSet& result);

    bool check_node_orders_disobeyed(LandmarkNode& node,
            const LandmarkSet& reached) const;

    void add_node_children(LandmarkNode& node, const LandmarkSet& reached) const;

    bool landmark_is_interesting(const State& s, const LandmarkSet& reached,
            LandmarkNode& lm) const;
    bool generate_helpful_actions(const State& state,
            const LandmarkSet& reached);
    void set_exploration_goals(const State& state);

    //int get_needed_landmarks(const State& state, LandmarkSet& needed) const;
    Exploration* get_exploration() {return exploration;}

protected:
    virtual int compute_heuristic(const State &state);
public:
    LandmarksCountHeuristic(bool use_preferred_operators, bool admissible, bool optimal, int landmarks_type=rpg_sasp);
    ~LandmarksCountHeuristic() {
    }
    virtual bool reach_state(const State& parent_state, const Operator &op,
            const State& state);
    virtual bool dead_ends_are_reliable() {
        return true;
    }

    enum {rpg_sasp = 0, zhu_givan = 1, exhaust = 2, search = 3};
};

#endif
