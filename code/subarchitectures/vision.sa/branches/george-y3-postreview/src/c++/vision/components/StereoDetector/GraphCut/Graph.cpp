/**
 * @file Graph.cpp
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief Create graph from 3D features in KinectCore.
 */

#include <vector>
#include <stdio.h>
#include <cstdlib>
#include <math.h>
#include "Graph.h"


namespace Z
{

/**
 * @brief Constructor of Graph
 */
Graph::Graph(CalculateRelations *r)
{
  relations = r;
}


/**
 * @brief Destructor of GraphCut
 */
Graph::~Graph()
{}


/**
 * @brief Build graph from results of the SVM-predictor.
 * @param e Vector of edges
 * @param num_edges Number of created edges.
 */
void Graph::BuildFromSVM(std::vector<E::Edge> &e, unsigned &num_edges)
{
  std::vector<Relation> rel;
  relations->getRelations(rel);
  
  for(unsigned i=0; i< rel.size(); i++)
  {
    E::Edge e;
    e.a = rel[i].id_0;
    e.b = rel[i].id_1;
    e.type = 1;
    e.w = 1- rel[i].rel_probability[1];           // it's the weight for joining elements!!! (1-p(x))
// printf("  Graph::BuildFromSVM: New edge (type: %u): %u-%u: %8.8f\n", rel[i].type, e.a, e.b, e.w);
    edges.push_back(e);
  }
  
  // TODO Check connectivity of graph
  
  num_edges = edges.size();
  e = edges;
}

} 











