#ifndef CAUSAL_GRAPH_H
#define CAUSAL_GRAPH_H

#include <algorithm>
#include <iosfwd>
#include <map>
#include <set>
#include <vector>
using namespace std;


class CausalGraph {
  vector<vector<int> > arcs;
  vector<vector<int> > inverse_arcs;
  vector<vector<int> > edges;
  vector<vector<int> > inverse_closures;
  void calc_closure(vector<vector<int> >& base, vector<set<int> >& result);
public:
  CausalGraph(istream &in);
  ~CausalGraph() {}
  const vector<int> &get_successors(int var) const;
  const vector<int> &get_predecessors(int var) const;
  const vector<int> &get_neighbours(int var) const;
  const vector<int> &get_pred_closure(int var) const;
  void dump() const;
};

#endif
