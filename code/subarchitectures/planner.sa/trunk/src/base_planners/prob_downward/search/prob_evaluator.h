#ifndef PROB_EVALUATOR_H
#define PROB_EVALUATOR_H

#include "scalar_evaluator.h"

#include <vector>

class ProbEvaluator : public ScalarEvaluator {
private:
	ScalarEvaluator * g_eval;
	ScalarEvaluator * h_eval;
	int value;
	bool dead_end;
	bool dead_end_reliable;

public:
    ProbEvaluator();
    ~ProbEvaluator();

	void set_evaluators(ScalarEvaluator *g, ScalarEvaluator *h);

	void evaluate(EvalInfo const* info, bool preferred);
	bool is_dead_end() const;
	bool dead_end_is_reliable() const;
    int get_value() const;
};

#endif


