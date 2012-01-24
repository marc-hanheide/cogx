#ifndef WA_STAR_H
#define WA_STAR_H

#include "general_eager_best_first_search.h"

class WeightedAStar: public GeneralEagerBestFirstSearch {
protected:
    int weight;
    virtual void initialize();
public:
    WeightedAStar(int w);
    virtual ~WeightedAStar();
};

#endif
