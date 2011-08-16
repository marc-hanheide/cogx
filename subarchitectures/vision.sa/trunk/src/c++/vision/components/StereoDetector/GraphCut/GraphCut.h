/**
 * @file GraphCut.h
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief GraphCut algorithm to cut graphs into pieces for segmentation.
 */

#ifndef Z_GRAPHCUT_H
#define Z_GRAPHCUT_H

#include "KinectCore.h"
#include "CalculateRelations.h"
#include "Learner.h"
#include "Graph.h"

#include "Edge.h"
#include "disjoint-set.h"

#include <../../../VisionUtils.h>
#include "TomGineThread.hh"

namespace Z
{

// THRESHOLD_CONSTANT: 3 - xx
#define THRESHOLD_CONSTANT 2
#define MIN_SIZE 2

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
  Learner *learner;
  Graph *graph;
  
  E::Edge *edges;         // Edges between the nodes, representing a probability
  unsigned num_edges;     // number of edges
  universe *u;            // universe to cut graph
  
  bool initialized;       // true, if graph is initialized

public:
  GraphCut(KinectCore *kc, Learner *l, CalculateRelations *r);
  ~GraphCut();
  
  
  bool Initialize();
  void Cut();
  void Show(TGThread::TomGineThread *tgRenderer);

};

}

#endif

