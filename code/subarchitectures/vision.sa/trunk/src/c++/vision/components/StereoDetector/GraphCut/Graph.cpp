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

#define MIN_PROBABILITY 0.01

/**
 * @brief Constructor of Graph
 */
Graph::Graph(KinectCore *kc, Learner *l, CalculateRelations *r)
{
  kcore = kc;
  relations = r;
  learner = l;
}


/**
 * @brief Destructor of GraphCut
 */
Graph::~Graph()
{
}


/**
 * @brief Build graph from 3D features
 */
void Graph::BuildFromSVM(std::vector<E::Edge> &e, unsigned &num_edges)
{
//   PatchPatchEdge();   // Build the edges between patches
  printf("Graph::BuildFromSVM: Time to implement!\n");
  
  std::vector<Relation> allRelations;
  relations->GetRelations(allRelations);
  
  for(unsigned i=0; i< allRelations.size(); i++)
  {
    E::Edge e;
    e.a = allRelations[i].id_0;
    e.b = allRelations[i].id_1;
    e.type = 1;
//     if(allRelations[i].prediction == 0)
    e.w = allRelations[i].rel_probability[1];           /// TODO TODO Wird das Gewicht bei Graph-Cut umgekehrt zur Wahrscheinlichkeit angegeben?
//     else
//      e.w = allRelations[i].rel_probability[1];                 /// TODO TODO Wird das Gewicht bei Graph-Cut umgekehrt zur Wahrscheinlichkeit angegeben?
printf("  New edge: %u-%u: %4.3f\n", e.a, e.b, e.w);
    edges.push_back(e);
  }
  
  // TODO Now check connectivity of graph
  
  
  num_edges = edges.size();
  e = edges;
}

/**
 * @brief Build graph from 3D features
 */
void Graph::Build(std::vector<E::Edge> &e, unsigned &num_edges)
{
printf("Graph::Build: Antiquated: Time to change to svm!\n");
  PatchPatchEdge();   // Build the edges between patches
  num_edges = edges.size();
  e = edges;
}

/**
 * @brief Create a edge between two patch nodes with following principles:
 * - proximity
 * - color similarity
 * - coplanarity
 */
void Graph::PatchPatchEdge()
{
  int nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
  for(unsigned i=0; i<nrPatches-1; i++)
  {
    Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=i+1; j<nrPatches; j++)
    {
      Patch3D *px = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);

      float pProximity = ProximityPP(p0, px);
      float pColor = ColorSimilarityPP(p0, px);
      float pCoplanarity = CoplanarityPP(p0, px);
      
      float probability = pProximity * (1-(1-pColor)*(1-pCoplanarity));    /// TODO How to connect these probabilities?

      if(probability > MIN_PROBABILITY)
      {
printf(" Graph::PatchPatchEdge: (%u-%u): pr/col/cop : sum => %4.3f/%4.3f/%4.3f : %4.3f\n", p0->GetNodeID(), px->GetNodeID(), pProximity, pColor, pCoplanarity, probability);
        // create edge
        E::Edge e;
        e.a = p0->GetNodeID();
        e.b = px->GetNodeID();
        e.type = 1;             // patch-patch edge
        e.w = 1-probability;               /// TODO TODO Wird das Gewicht bei Graph-Cut umgekehrt zur Wahrscheinlichkeit angegeben?
        edges.push_back(e);
      }      
    }
  }
}

/**
 * @brief Get the probability value for proximity between patches
 * @param p0 First Patch3D
 * @param p1 Second Patch3D
 * @return Returns the probability
 */
float Graph::ProximityPP(Patch3D *p0, Patch3D *p1)
{
  double proximity = p0->IsClose(p1);
  return learner->GetPProximityPP(proximity);
}

/**
 * @brief Get the probability value for proximity between patches
 * @param p0 First Patch3D
 * @param p1 Second Patch3D
 * @return Returns the probability
 */
float Graph::ColorSimilarityPP(Patch3D *p0, Patch3D *p1)
{
  double color = p0->CompareColor(p1);
  return learner->GetPColorSimilarityPP(color);
}


/**
 * @brief Get the probability value for proximity between patches
 * @param p0 First Patch3D
 * @param p1 Second Patch3D
 * @return Returns the probability
 */
float Graph::CoplanarityPP(Patch3D *p0, Patch3D *p1)
{
  double normal, distance;
  p0->CalculateCoplanarity(p1, normal, distance);
  return learner->GetPCoplanarityPP(normal);
}



} 











