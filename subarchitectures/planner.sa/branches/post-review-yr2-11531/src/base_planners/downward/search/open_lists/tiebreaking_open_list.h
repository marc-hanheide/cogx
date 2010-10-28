#ifndef OPEN_LISTS_TIEBREAKING_OPEN_LIST_H
#define OPEN_LISTS_TIEBREAKING_OPEN_LIST_H

#include "open_list.h"
#include "../evaluator.h"

#include <deque>
#include <map>
#include <vector>
#include <utility>

class ScalarEvaluator;

template<class Entry>
class TieBreakingOpenList : public OpenList<Entry> {
    
    typedef std::deque<Entry> Bucket;

    std::map<const std::vector<int>, Bucket> buckets;
    int size;

    std::vector<ScalarEvaluator *> evaluators;
    std::vector<int> last_evaluated_value;
    bool last_preferred;
    bool dead_end;
    bool first_is_dead_end;
    bool dead_end_reliable;
    bool allow_unsafe_pruning; // don't insert if main evaluator
    // says dead end, even if not reliably
    
    const std::vector<int>& get_value(); // currently not used
    int dimension() const;
protected:
    Evaluator* get_evaluator() { return this; }

public:
    TieBreakingOpenList(const std::vector<ScalarEvaluator *> &evals,
                        bool preferred_only=false, bool unsafe_pruning=true);
    ~TieBreakingOpenList();

    // open list interface
    int insert(const Entry &entry);
    Entry remove_min();
    bool empty() const;
    void clear();

    // tuple evaluator interface
    void evaluate(int g, bool preferred);
    bool is_dead_end() const;
    bool dead_end_is_reliable() const;
};

#include "tiebreaking_open_list.cc"

// HACK! Need a better strategy of dealing with templates, also in the Makefile.

#endif
