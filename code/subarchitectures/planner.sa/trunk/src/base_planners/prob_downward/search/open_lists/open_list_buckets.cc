// HACK! Ignore this if used as a top-level compile target.
#ifdef OPEN_LISTS_OPEN_LIST_BUCKETS_H

#include <cassert>
using namespace std;

/*
  Bucket-based implementation of an open list.
  Nodes with identical heuristic value are expanded in FIFO order.

  It would be easy to templatize the "State *" and "Operator *"
  datatypes, because these are only used as anonymous data. However,
  there is little point in applying such generalizations before there
  is any need for them.
*/

template<class Entry>
BucketOpenList<Entry>::BucketOpenList(ScalarEvaluator *eval, bool preferred_only) 
    : OpenList<Entry>(preferred_only), lowest_bucket(9999999), size(0), evaluator(eval) {
}

template<class Entry>
BucketOpenList<Entry>::~BucketOpenList() {
}

template<class Entry>
int BucketOpenList<Entry>::insert(const Entry &entry) {
    if (OpenList<Entry>::only_preferred && !last_preferred)
        return 0;
    int key = last_evaluated_value;
    assert(key >= 0);
    if(key >= buckets.size())
        buckets.resize(key + 1);
    if(key < lowest_bucket)
        lowest_bucket = key;
    buckets[key].push_back(entry);
    size++;
    return 1;
}

template<class Entry>
Entry BucketOpenList<Entry>::remove_min() {
    assert(size > 0);
    while(buckets[lowest_bucket].empty())
    lowest_bucket++;
    size--;
    Entry result = buckets[lowest_bucket].front();
    buckets[lowest_bucket].pop_front();
    return result;
}

template<class Entry>
bool BucketOpenList<Entry>::empty() const {
    return size == 0;
}

template<class Entry>
void BucketOpenList<Entry>::clear() {
    buckets.clear();
    lowest_bucket = 9999999;
    size = 0;
}

template<class Entry>
void BucketOpenList<Entry>::evaluate(EvalInfo const* info, bool preferred) {
    get_evaluator()->evaluate(info, preferred);
    last_evaluated_value = get_evaluator()->get_value();
    last_preferred = preferred;
    dead_end = get_evaluator()->is_dead_end();
    dead_end_reliable = get_evaluator()->dead_end_is_reliable();
}

template<class Entry>
bool BucketOpenList<Entry>::is_dead_end() const {
    return dead_end;
}

template<class Entry>
bool BucketOpenList<Entry>::dead_end_is_reliable() const {
    return dead_end_reliable;
}

#endif
