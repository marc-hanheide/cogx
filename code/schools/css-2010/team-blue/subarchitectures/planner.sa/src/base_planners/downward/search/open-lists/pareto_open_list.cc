// HACK! Ignore this if used as a top-level compile target.
#ifdef TIEBREAKING_OPENLIST_H

#include "../tuple_evaluator.h"

#include <iostream>
#include <cassert>
#include <limits>
using namespace std;

template<class Entry>
bool ParetoOpenList<Entry>::dominates(const KeyType &v1, const KeyType &v2) {
    assert (v1.size() == v2.size());
    bool unequal = false;
    for (unsigned int i = 0; i < v1.size(); i++) {
        if (v1[i] > v2[i])
            return false;
        else if (v1[i] < v2[i])
            unequal = true;
    }
    return unequal;
}

template<class Entry>
bool ParetoOpenList<Entry>::is_nondominated(const KeyType &vec,
        KeySet &domination_candidates) {
    for (typename KeySet::iterator it = domination_candidates.begin();
            it != domination_candidates.end(); it++) {
        if (dominates(*it, vec))
            return false;
    }
    return true;
}

template<class Entry>
void ParetoOpenList<Entry>::remove_key(const KeyType key) {
    nondominated.erase(key);
    buckets.erase(key);
    KeySet candidates;
    for (typename BucketMap::iterator it = buckets.begin(); it != buckets.end(); it ++)
        if (nondominated.find(it->first) == nondominated.end() &&
                dominates(key, it->first) &&
                is_nondominated(it->first, nondominated))
            candidates.insert(it->first);
    for (typename KeySet::iterator it = candidates.begin();
            it != candidates.end(); it ++)
        if (is_nondominated(*it, candidates))
            nondominated.insert(*it);
}

template<class Entry>
ParetoOpenList<Entry>::ParetoOpenList(const std::vector<ScalarEvaluator *> &evals,
    bool preferred_only) 
	: OpenList<Entry>(preferred_only), evaluators(evals) {
    last_evaluated_value.resize(evaluators.size());
}

template<class Entry>
ParetoOpenList<Entry>::~ParetoOpenList() {
}

template<class Entry>
int ParetoOpenList<Entry>::insert(const Entry &entry) {
	if (OpenList<Entry>::only_preferred && !last_preferred)
		return 0;
    const std::vector<int> &key = last_evaluated_value;
    Bucket &bucket = buckets[key];
    bool newkey = bucket.empty();
    bucket.push_back(entry);
    if (newkey && is_nondominated(key, nondominated)) {
        typename KeySet::iterator it = nondominated.begin();
        while (it != nondominated.end()) {
            if (dominates(key, *it)) {
                typename KeySet::iterator tmp_it = it;
                it ++;
                nondominated.erase(tmp_it);
            } else {
                it ++;
            }
        }
        nondominated.insert(key);
    }
	return 1;
}

template<class Entry>
Entry ParetoOpenList<Entry>::remove_min() {
    typename KeySet::iterator selected = nondominated.begin();
    int seen = 0;
    for (typename KeySet::iterator it = nondominated.begin();
            it != nondominated.end(); it ++) {
        int numerator;
        if (state_uniform_selection) 
            numerator =  it->size();
        else 
            numerator =  1;
        seen += numerator;
        if ((rand() % seen ) < numerator)
            selected = it;
    }
    Bucket &bucket = buckets[*selected];
    Entry result = bucket.front();
    bucket.pop_front();
    if (bucket.empty())
        remove_key(*selected);
    return result;
}

template<class Entry>
void ParetoOpenList<Entry>::evaluate(int g, bool preferred) {
    dead_end = false;
    dead_end_reliable = false;
    for (unsigned int i = 0; i < evaluators.size(); i++) {
        evaluators[i]->evaluate(g, preferred);

        // check for dead end
        if (evaluators[i]->is_dead_end()) {
            last_evaluated_value[i] = std::numeric_limits<int>::max();
            dead_end = true;
            if (evaluators[i]->dead_end_is_reliable()) {
                dead_end_reliable = true;
            }
        } else { // add value if no dead end
        	last_evaluated_value[i] = evaluators[i]->get_value();
		}
    }
	last_preferred = preferred;
}

template<class Entry>
bool ParetoOpenList<Entry>::is_dead_end() const {
    return dead_end;
}

template<class Entry>
bool ParetoOpenList<Entry>::dead_end_is_reliable() const {
    return dead_end_reliable;
}

#endif
