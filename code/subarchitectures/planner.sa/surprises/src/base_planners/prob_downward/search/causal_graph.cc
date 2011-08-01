#include "causal_graph.h"
#include "globals.h"

#include <iostream>
#include <cassert>
using namespace std;

CausalGraph::CausalGraph(istream &in) {
  check_magic(in,"begin_CG");
  int var_count = g_variable_domain.size();
  arcs.resize(var_count);
  inverse_arcs.resize(var_count);
  edges.resize(var_count);
  inverse_closures.resize(var_count);
  for(int from_node = 0; from_node < var_count; from_node++) {
    int num_succ;
    in >> num_succ;
    arcs[from_node].reserve(num_succ);
    for(int j = 0; j < num_succ; j++) {
      // weight not needed so far
      int to_node, weight;
      in >> to_node;
      in >> weight;
      arcs[from_node].push_back(to_node);
      inverse_arcs[to_node].push_back(from_node);
      edges[from_node].push_back(to_node);
      edges[to_node].push_back(from_node);
    }
  }
  check_magic(in, "end_CG");

  for(int i = 0; i < var_count; i++) {
    sort(edges[i].begin(), edges[i].end());
    edges[i].erase(unique(edges[i].begin(), edges[i].end()), edges[i].end());
  }
  
  vector<set<int> > temp;
  temp.resize(var_count);
  calc_closure(inverse_arcs, temp);
  for(int i = 0; i < var_count; i++) {
      inverse_closures[i].insert(inverse_closures[i].begin(), temp[i].begin(), temp[i].end());
      if (g_debug && !inverse_closures[i].empty()) {
          cout << "predecessors of " << g_variable_name[i] << endl;
          for (int j = 0; j < inverse_closures[i].size(); j++) {
              cout << "    " << g_variable_name[inverse_closures[i][j]] << endl;
          }
      }
  }
}

void CausalGraph::calc_closure(vector<vector<int> >& base, vector<set<int> >& result) {
    bool changed = true;
    for (int i = 0; i < base.size(); i++) {
        result[i].insert(base[i].begin(), base[i].end());
    }
    while (changed) {
        changed = false;
        for (int i = 0; i < result.size(); i++) {
            int before = result[i].size();
            for (int j = 0; j < base[i].size(); j++) {
                result[i].insert(result[base[i][j]].begin(), result[base[i][j]].end());
            }
            if (result[i].size() > before) {
                changed = true;
            }
        }
    }
}

const vector<int> &CausalGraph::get_successors(int var) const {
  return arcs[var];
}

const vector<int> &CausalGraph::get_predecessors(int var) const {
  return inverse_arcs[var];
}

const vector<int> &CausalGraph::get_neighbours(int var) const {
  return edges[var];
}

const vector<int> &CausalGraph::get_pred_closure(int var) const {
    return inverse_closures[var];
}

void CausalGraph::dump() const {
  cout <<"Causal graph: "<< endl;
  for(int i = 0; i < arcs.size(); i++) {
    cout << "dependent on var " << g_variable_name[i] << ": " << endl;
    for(int j = 0; j < arcs[i].size(); j++)
      cout <<"  "<< g_variable_name[arcs[i][j]] << ","; 
    cout << endl;
    }
}

// TODO: put acyclicity in input
//bool CausalGraph::is_acyclic() const {
//  return acyclic;
//}

