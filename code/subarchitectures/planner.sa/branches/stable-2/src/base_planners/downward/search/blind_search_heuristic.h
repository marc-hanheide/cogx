#ifndef BLIND_SEARCH_HEURISTIC_H
#define BLIND_SEARCH_HEURISTIC_H

#include "heuristic.h"

class BlindSearchHeuristic : public Heuristic {
protected:
    virtual void initialize();
    virtual int compute_heuristic(const State &state);
public:
    BlindSearchHeuristic();
    ~BlindSearchHeuristic();
};

#endif
