#ifndef SEARCH_ENGINE_H
#define SEARCH_ENGINE_H

#include <vector>

class Heuristic;

#include "operator.h"

class SearchEngine {
public:
    typedef std::vector<const Operator *> Plan;
private:
    bool solved;
    Plan plan;
protected:
    enum {FAILED, SOLVED, IN_PROGRESS};
    virtual void initialize() {}
    virtual int step() = 0;

    void set_plan(const Plan &plan);
public:
    SearchEngine();
    virtual ~SearchEngine();
    virtual void statistics() const;
    virtual void add_heuristic(Heuristic *heuristic, bool use_estimates,
                               bool use_preferred_operators) = 0;
    bool found_solution() const;
    const Plan &get_plan() const;
    void search();
};

#endif
