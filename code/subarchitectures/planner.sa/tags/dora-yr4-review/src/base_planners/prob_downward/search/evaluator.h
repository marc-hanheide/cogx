#ifndef EVALUATOR_H
#define EVALUATOR_H

class Operator;
class State;
class EvalInfo;

class Evaluator {
public:
    virtual ~Evaluator() {}

	virtual void evaluate(EvalInfo const* info, bool preferred) = 0;
	virtual bool is_dead_end() const = 0;
	virtual bool dead_end_is_reliable() const = 0;
};

#endif
