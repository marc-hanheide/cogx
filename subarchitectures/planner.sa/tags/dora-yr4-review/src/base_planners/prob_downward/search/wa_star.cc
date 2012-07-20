#include "wa_star.h"
#include "open_lists/standard_scalar_open_list.h"
#include "open_lists/alternation_open_list.h"
#include "open_lists/tiebreaking_open_list.h"
#include "scalar_evaluator.h"
#include "heuristic.h"
#include "g_evaluator.h"
#include "sum_evaluator.h"
#include "weighted_evaluator.h"
#include "prob_evaluator.h"
#include <vector>

WeightedAStar::WeightedAStar(int w):
    GeneralEagerBestFirstSearch(true), weight(w) {
    // TODO Auto-generated constructor stub

}

WeightedAStar::~WeightedAStar() {
    // TODO Auto-generated destructor stub
}


void WeightedAStar::initialize() {
    //TODO children classes should output which kind of search
    cout << "Conducting lazy weighted A* search, weight = " << weight <<  endl;

    GEvaluator *g = new GEvaluator();

    if ((heuristics.size() == 1) &&  (preferred_operator_heuristics.size() == 0)) {
        //SumEvaluator *f = new SumEvaluator();
        ProbEvaluator *f = new ProbEvaluator();
        WeightedEvaluator *w = new WeightedEvaluator(heuristics[0], weight);
        f->set_evaluators(g, w);
        //f->add_evaluator(g);
        //f->add_evaluator(w);
        open_list = new StandardScalarOpenList<state_var_t *>(f);
    }
    else {
        vector<OpenList<state_var_t *>*> inner_lists;
        for (int i = 0; i < estimate_heuristics.size(); i++) {
            ProbEvaluator *f = new ProbEvaluator();
            //SumEvaluator *f = new SumEvaluator();
            WeightedEvaluator *w = new WeightedEvaluator(estimate_heuristics[i], weight);
            f->set_evaluators(g, w);
            // f->add_evaluator(g);
            // f->add_evaluator(w);
            inner_lists.push_back(new StandardScalarOpenList<state_var_t *>(f, false));
            if (preferred_operator_heuristics.size() > 0) {
                inner_lists.push_back(new StandardScalarOpenList<state_var_t *>(f, true));
            }
        }
        open_list = new AlternationOpenList<state_var_t *>(inner_lists);
    }

    GeneralEagerBestFirstSearch::initialize();
}
