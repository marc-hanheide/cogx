#ifndef G_EVALUATOR_H
#define G_EVALUATOR_H

#include "scalar_evaluator.h"

class GEvaluator : public ScalarEvaluator {
private:
    int value;

public:
    GEvaluator();
    ~GEvaluator();

    void evaluate(int g, bool preferred);
    bool is_dead_end() const;
    bool dead_end_is_reliable() const;
    int get_value() const;
};

#endif


