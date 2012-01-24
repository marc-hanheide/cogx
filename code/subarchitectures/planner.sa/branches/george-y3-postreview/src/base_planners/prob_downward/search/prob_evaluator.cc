#include "prob_evaluator.h"
#include "search_space.h"
#include "operator.h"

#include <limits>

ProbEvaluator::ProbEvaluator() {
}

ProbEvaluator::~ProbEvaluator(){
}

void ProbEvaluator::set_evaluators(ScalarEvaluator *g, ScalarEvaluator *h) {
    g_eval = g;
    h_eval = h;
}

void ProbEvaluator::evaluate(EvalInfo const* info, bool preferred) {
    dead_end = false;
    dead_end_reliable = false;
    g_eval->evaluate(info, preferred);
    h_eval->evaluate(info, preferred);
    // value = g_eval->get_value() + g_multiplier *  info->get_p() * h_eval->get_value();
    value = g_eval->get_value() + g_multiplier * h_eval->get_value();
    // if (info->op)
    //     cout << "f: " << value <<  ", g: " << info->g << " (" << info->c << ", " << info->p << ")  h:" << h_eval->get_value() << "  by " << info->op->get_name() << endl;
    // else
    //     cout << "f: " << value <<  ", g: " << info->g << " (" << info->c << ", " << info->p << ")  h:" << h_eval->get_value() << endl;
    // cout << g_eval->get_value() <<" + "<<g_multiplier<<" * "<<h_eval->get_value() << " = " <<value << endl;
    // if (info->get_p() <= 0.01) {
    //     // cerr << "dead"<<endl;
    //     dead_end = true;
    //     dead_end_reliable = true;
    // }
    if (g_eval->is_dead_end()) {
        value = std::numeric_limits<int>::max();
        dead_end = true;
        if (g_eval->dead_end_is_reliable()) {
            dead_end_reliable = true;
        }
    }
    if (h_eval->is_dead_end()) {
        value = std::numeric_limits<int>::max();
        dead_end = true;
        if (h_eval->dead_end_is_reliable()) {
            dead_end_reliable = true;
        }
    }
}

bool ProbEvaluator::is_dead_end() const {
    return dead_end;
}

bool ProbEvaluator::dead_end_is_reliable() const {
    return dead_end_reliable;
}

int ProbEvaluator::get_value() const {
    return value;
}


