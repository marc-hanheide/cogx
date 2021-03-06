// HACK! Ignore this if used as a top-level compile target.
#ifdef ALTERNATION_OPENLIST_H

#include <iostream>
#include <cassert>
using namespace std;

template<class Entry>
AlternationOpenList<Entry>::AlternationOpenList(const vector<OpenList<Entry> *> &sublists,
    int boost_influence) 
    : open_lists(sublists), priorities(sublists.size(), 0), size(0), 
      boosting(boost_influence) {
}

template<class Entry>
AlternationOpenList<Entry>::~AlternationOpenList() {
}

template<class Entry>
int AlternationOpenList<Entry>::insert(const Entry &entry) {
	int new_entries = 0;
    for (unsigned int i = 0; i < open_lists.size(); i++) {
		new_entries += open_lists[i]->insert(entry);
    }
    size += new_entries;
	return new_entries;
}

template<class Entry>
Entry AlternationOpenList<Entry>::remove_min() {
    assert(size > 0);
    int best = 0;
    for (unsigned int i = 0; i < open_lists.size(); i++) {
        if (!open_lists[i]->empty() &&
            priorities[i] < priorities[best]) {
            best = i;
        }
    }
    last_used_list = best;
    OpenList<Entry>* best_list = open_lists[best];
    assert (!best_list->empty());
    size--;
    priorities[best]++;
    return best_list->remove_min();
}

template<class Entry>
bool AlternationOpenList<Entry>::empty() const {
    return size == 0;
}

template<class Entry>
void AlternationOpenList<Entry>::evaluate(int g, bool preferred) {
    dead_end = false;
    dead_end_reliable = false;
    for (unsigned int i = 0; i < open_lists.size(); i++) {
        open_lists[i]->evaluate(g, preferred);
        if (open_lists[i]->is_dead_end()) {
            dead_end = true;
            if (open_lists[i]->dead_end_is_reliable()) {
                dead_end_reliable = true;
                break;
            }
        }
	}
}

template<class Entry>
bool AlternationOpenList<Entry>::is_dead_end() const {
    return dead_end;
}

template<class Entry>
bool AlternationOpenList<Entry>::dead_end_is_reliable() const {
    return dead_end_reliable;
}

template<class Entry>
int AlternationOpenList<Entry>::boost_preferred() {
    int total_boost = 0;
    for (unsigned int i = 0; i < open_lists.size(); i++) {
        // if the open list is not an alternation open list
        // (these have always only_preferred==false) and
        // it takes only preferred states, we boost it
        if (open_lists[i]->only_preferred_states()) {
            priorities[i] -= boosting;
            total_boost += boosting;
        }
        // otherwise, we tell it to boost its lists (which
        // has no effect on non-alterntion lists)
        else {
            int boosted = open_lists[i]->boost_preferred();
            // now we have to boost this alternation open list
            // as well to give its boosting some effect
            priorities[i] -= boosted;
            total_boost += boosted;
        }
    }
    return total_boost; // can be used by "parent" alternation list
}

template<class Entry>
void AlternationOpenList<Entry>::boost_last_used_list() {
    priorities[last_used_list] -= boosting;

    // for the case that the last used list is an alternation
    // list 
    open_lists[last_used_list]->boost_last_used_list();
}
#endif
