/**
 * @file GraphCut.h
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief GraphCut algorithm to cut graphs into pieces for segmentation.
 */

#ifndef Z_GRAPHCUT_H
#define Z_GRAPHCUT_H

#include <set>

#include "KinectCore.h"
#include "CalculateRelations.h"
#include "Learner.h"
#include "Graph.h"

#include "Edge.h"
#include "disjoint-set.h"

#include <../../../VisionUtils.h>

namespace Z
{

// THRESHOLD_CONSTANT: 1. - 10.? => 2.
// CAREFULL: Float value!!!
// - with one line for each patch => 3.0
// - only patches => 1.0
#define THRESHOLD_CONSTANT 0.5
#define MIN_SIZE 1                /// minimum size of element-sets

#define THRESHOLD(size, c) (c/size)

/**
 * @brief Class GraphCut
 */
class GraphCut
{
public:
  
private:
  KinectCore *kcore;
  CalculateRelations *relations;
  Graph *graph;
  
  E::Edge *edges;         // Edges between the nodes, representing a probability
  unsigned num_edges;     // number of edges
  universe *u;            // universe to cut graph
  
  bool initialized;       // true, if graph is initialized

public:
  GraphCut(KinectCore *kc, CalculateRelations *r);
  ~GraphCut();
  
  
  bool Initialize();
  void Cut();
  void CopyGroupIDToFeatures();

};

}

#endif

