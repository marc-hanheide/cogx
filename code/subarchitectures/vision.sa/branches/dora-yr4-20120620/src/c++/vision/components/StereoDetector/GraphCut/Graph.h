/**
 * @file Graph.h
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief Create graph from 3D features in KinectCore.
 */

#ifndef Z_GRAPH_H
#define Z_GRAPH_H

#include <vector>
// #include "KinectCore.h"
#include "CalculateRelations.h"
// #include "Learner.h"
#include "Patch3D.h"
#include "Edge.h"

namespace Z
{

/**
 * @brief Class Graph
 */
class Graph
{
private:
  CalculateRelations *relations;    // Relations between features
  
  std::vector<E::Edge> edges;       // edges of the graph (wiht node numbers and probability)

public:
  
private:

public:
  Graph(CalculateRelations *r);
  ~Graph();
  
  void BuildFromSVM(std::vector<E::Edge> &e, unsigned &num_edges);

};

}

#endif

