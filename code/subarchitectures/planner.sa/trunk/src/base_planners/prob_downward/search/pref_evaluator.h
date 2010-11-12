#ifndef PREF_EVALUATOR_H
#define PREF_EVALUATOR_H

#include "scalar_evaluator.h"

class PrefEvaluator : public ScalarEvaluator {
private:
    bool value_preferred;

public:
    PrefEvaluator();
    ~PrefEvaluator();

    void evaluate(EvalInfo const* info, bool preferred);
    bool is_dead_end() const;
    bool dead_end_is_reliable() const;
    int get_value() const;
};

#endif


