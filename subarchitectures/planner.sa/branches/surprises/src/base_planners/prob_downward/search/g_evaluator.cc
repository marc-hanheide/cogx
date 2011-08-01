#include "g_evaluator.h"
#include "search_space.h"

GEvaluator::GEvaluator() {
}

GEvaluator::~GEvaluator(){
}

void GEvaluator::evaluate(EvalInfo const* info, bool) {
    value = info->get_g();
}

bool GEvaluator::is_dead_end() const {
    return false;
}

bool GEvaluator::dead_end_is_reliable() const {
    return true;
}

int GEvaluator::get_value() const {
    return value;
}


