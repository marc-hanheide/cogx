/**
 * @file GraphCut.cpp
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief GraphCut algorithm to cut graphs into pieces for segmentation.
 */

#include<vector>
#include<stdio.h>
#include<cstdlib>
#include<math.h>
#include "GraphCut.h"

namespace Z
{

/**
 * @brief Constructor of GraphCut
 */
GraphCut::GraphCut(KinectCore *kc, CalculateRelations *r)
{
  kcore = kc;
  relations = r;
  initialized = false;
}


/**
 * @brief Destructor of GraphCut
 */
GraphCut::~GraphCut()
{
}

/**
 * @brief Initialize the graph and prepare for cutting 
 * @return Returns true, if cutter is ready to cut the graph.
 */
bool GraphCut::Initialize()
{
  graph = new Graph(relations);                     /// TODO kcore und learner sollten nicht mehr notwendig sein!!!

  std::vector<E::Edge> e;
  graph->BuildFromSVM(e, num_edges);

  edges = new E::Edge[num_edges];
  for(unsigned i=0; i<num_edges; i++)
    edges[i] = e[i];

  if(num_edges == 0)
  { 
    initialized = false;
    return false;
  } 
  else
  {
    u = new universe(num_edges);
    initialized = true;
printf("GraphCut::Initialize: num_edges: %u\n", num_edges);
  }
  return true;
}


/**
 * @brief Compare edges for sort function.
 * @param a Edge a
 * @param b Edge b
 * @return Returns true if a.w < b.w
 */
bool smallerEdge(const E::Edge &a, const E::Edge &b)
{
  return a.w < b.w;
}


/**
 * @brief Graph Cutting
 */
void GraphCut::Cut()
{
  if(!initialized)
  {
    printf("GraphCut::Cut: Error: Graph is not initialized!\n");
    return;
  }

printf("GraphCut::Cut: start\n");

// printf("Edges again: %u\n", num_edges);
// for(unsigned i=0; i<num_edges; i++)
// {
//   E::Edge *e = &edges[i];
//   printf("  %i - %i => %4.3f\n", e->a, e->b, e->w); 
// }
    
  /// sort edges by weight  
  std::sort(edges, edges + num_edges, smallerEdge);

  /// init thresholds
  float *threshold = new float[num_edges];
  for (int i = 0; i < num_edges; i++)
    threshold[i] = THRESHOLD(1, THRESHOLD_CONSTANT);
  
printf("THRESHOLD: %4.3f\n", threshold[0]);

// printf("GraphCut::Cut: start 2: num_sets: %u\n", u->num_sets());

/// PRINT all edges:

  
  
  /// for each edge, in non-decreasing weight order...
  for (int i = 0; i < num_edges; i++) 
  {
    
u->printAll();
for (int i = 0; i < num_edges; i++) 
{
  E::Edge *pedge = &edges[i];
  printf("all edges: %u: %u-%u with thrd: %4.3f => universe: %u-%u\n", i, pedge->a, pedge->b, threshold[i], u->find(pedge->a), u->find(pedge->b));
}
    
    
printf("\nstart with edge %u: ", i);
    E::Edge *pedge = &edges[i];
    
    // components conected by this edge
    int a = u->find(pedge->a);
    int b = u->find(pedge->b);
    
printf("node: %u-%u / universe: %u-%u: ", pedge->a, pedge->b, a, b);
    
    if (a != b) 
    {
printf("weight: %4.3f and thds: %4.3f-%4.3f\n", pedge->w, threshold[a], threshold[b]);
      if ((pedge->w <= threshold[a]) && (pedge->w <= threshold[b])) 
      {
        u->join(a, b);
        a = u->find(a);
printf("  => join: threshold[%u] = %4.3f => ", a, threshold[a]);
        threshold[a] = pedge->w + THRESHOLD(u->size(a), THRESHOLD_CONSTANT);
printf("%4.3f (size: %u)\n", threshold[a], u->size(a));
      }
    }
  }

// printf("GraphCut::Cut: end 4\n");
// 
//   /// post process small components (defined min_size!!!)
//   for (int i = 0; i < num_edges; i++) 
//   {
//     int a = u->find(edges[i].a);
//     int b = u->find(edges[i].b);
//     if ((a != b) && ((u->size(a) < MIN_SIZE) || (u->size(b) < MIN_SIZE)))
//       u->join(a, b);
//   }
/// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO Was sagen hier die NumComponents???
  int num_components = u->num_sets();
  printf("GraphCut::Cut: number of components: %u\n", num_components);
// 
// 

/// print results
/*printf("\nGraphCut: Cut => We have %i components!!!\n", num_components);
if(num_edges > 0)
{  
  int nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
  for(unsigned i=0; i<nrPatches; i++)
  {
    int comp = u->find(i);
    printf("GraphCut: Patch %u => %u\n", i, comp);
  }
  int nrLines = kcore->NumGestalts3D(Gestalt3D::LINE);
  unsigned nrLinesStart = (kcore->Gestalts3D(Gestalt3D::LINE, 0))->GetNodeID();
  for(unsigned i=nrLinesStart; i<nrLinesStart+nrLines; i++)
  {
    int comp = u->find(i);
    printf("GraphCut: Line %u => %u\n", i, comp);
  }
}*/ 
  
// printf("GraphCut::Cut: end\n");

  /// free up
  delete threshold;
//   delete [] edges;
  initialized = false;
}


/**
 * @brief Copy the group number of the graph cut for each feature.
 * Each feature has to be a node in the graph!!!
 */
void GraphCut::CopyGroupIDToFeatures()
{  
// printf("GraphCut::CopyGroupIDToFeatures: start!\n");
  std::set<unsigned> graphCutLabels;
  set<unsigned>::iterator it;
  
  for(int type=0; type < Gestalt3D::MAX_TYPE; type++)
  {
// printf("GraphCut::CopyGroupIDToFeatures: start 2\n");
    Gestalt3D::Type t = (Gestalt3D::Type) type;
    for(unsigned i=0; i < kcore->NumGestalts3D(t); i++)
    {
// printf("GraphCut::CopyGroupIDToFeatures: start 3\n");
      // get node id of feature
      unsigned nodeID = kcore->Gestalts3D(t, i)->GetNodeID();
// printf("GraphCut::CopyGroupIDToFeatures: start 3: nodeID: %u\n", nodeID);

      // find node in graph
      if(nodeID != -1)                                                            /// TODO Wie kann eine NodeID = -1 sein? Alle werden durchnummeriert?
      {
// printf("GraphCut::CopyGroupIDToFeatures: start 4: nodeID: %u\n", nodeID);

        int cut_id = u->find(nodeID);
// printf("GraphCut::CopyGroupIDToFeatures: start 4: found cut_id: %u\n", cut_id);
        kcore->Gestalts3D(t, i)->SetGraphCutLabel(cut_id);
        
        if(graphCutLabels.find(cut_id) == graphCutLabels.end())
          graphCutLabels.insert(cut_id);
      }
    }
  }
  kcore->SetGraphCutGroups(graphCutLabels);
// printf("GraphCut::CopyGroupIDToFeatures: end!\n");
}

} 











