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
#include "KinectCore.h"
#include "CalculateRelations.h"
#include "Learner.h"
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
  KinectCore *kcore;                // KinectCore with 3D information
  CalculateRelations *relations;    // Relations between features
  Learner *learner;                 // Results from the learner
  
  std::vector<E::Edge> edges;       // edges of the graph (wiht node numbers and probability)

public:
  

private:
  void PatchPatchEdge();

  float ProximityPP(Patch3D *p0, Patch3D *p1);
  float ColorSimilarityPP(Patch3D *p0, Patch3D *p1);
  float CoplanarityPP(Patch3D *p0, Patch3D *p1);

public:
  Graph(KinectCore *kc, Learner *l, CalculateRelations *r);
  ~Graph();
  
  void BuildFromSVM(std::vector<E::Edge> &e, unsigned &num_edges);
  void Build(std::vector<E::Edge> &e, unsigned &num_edges);


};

}

#endif

