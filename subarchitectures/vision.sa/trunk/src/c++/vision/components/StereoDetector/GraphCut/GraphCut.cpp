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
GraphCut::GraphCut(KinectCore *kc, Learner *l, CalculateRelations *r)
{
  kcore = kc;
  relations = r;
  learner = l;
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
  // Build graph and universe
  graph = new Graph(kcore, learner, relations);

  std::vector<E::Edge> e;
//   graph->Build(e, num_edges);
  graph->BuildFromSVM(e, num_edges);
printf("GraphCut::Initialize: we have %u edges!\n", num_edges);
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
    u = new universe(num_edges);;
    initialized = true;
  }
  return true;
}


/**
 * @brief Compare edges
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

printf("Edges again: %u\n", num_edges);
for(unsigned i=0; i<num_edges; i++)
{
  E::Edge *e = &edges[i];
  printf("  %i - %i => %4.3f\n", e->a, e->b, e->w); 
}
    
  /// sort edges by weight  
  std::sort(edges, edges + num_edges, smallerEdge);

printf("GraphCut::Cut: start 2\n");

  /// init thresholds
  float *threshold = new float[num_edges];
  for (int i = 0; i < num_edges; i++)
    threshold[i] = THRESHOLD(1, THRESHOLD_CONSTANT);

printf("GraphCut::Cut: start 3\n");
  
  /// for each edge, in non-decreasing weight order...
  for (int i = 0; i < num_edges; i++) 
  {
printf("GraphCut::Cut: start 3-1\n");
    E::Edge *pedge = &edges[i];
printf("GraphCut::Cut: start 3-2: find %u and %u\n", pedge->a, pedge->b);
    
    // components conected by this edge
    int a = u->find(pedge->a);
printf("  %u found!\n", pedge->a);
    int b = u->find(pedge->b);
printf("  %u found!\n", pedge->b);
    
    
printf("GraphCut::Cut: start 3-3\n");
    if (a != b) 
    {
printf("GraphCut::Cut: start 3-3-1\n");
      if ((pedge->w <= threshold[a]) && (pedge->w <= threshold[b])) 
      {
        u->join(a, b);
        a = u->find(a);
        threshold[a] = pedge->w + THRESHOLD(u->size(a), THRESHOLD_CONSTANT);
      }
printf("GraphCut::Cut: start 3-3-2 end\n");
    }
printf("GraphCut::Cut: start 3-4-end\n");

  }

printf("GraphCut::Cut: start 4\n");

  /// post process small components (defined min_size!!!)
  for (int i = 0; i < num_edges; i++) 
  {
    int a = u->find(edges[i].a);
    int b = u->find(edges[i].b);
    if ((a != b) && ((u->size(a) < MIN_SIZE) || (u->size(b) < MIN_SIZE)))
      u->join(a, b);
  }
  int num_components = u->num_sets();

printf("GraphCut::Cut: start 5\n");


/// print results
printf("GraphCut: Cut => We have %i components!!!\n", num_components);
if(num_edges > 0)
{  
  int nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
  for(unsigned i=0; i<nrPatches; i++)
  {
    int comp = u->find(i);
    printf("Patch %u => %u\n", i, comp);
  }
} 
  
printf("GraphCut::Cut: end\n");

  /// free up
  delete threshold;
//   delete [] edges;
  initialized = false;
}



/**
 * @brief Show patches on the tgRenderer
 */
void GraphCut::Show(TGThread::TomGineThread *tgRenderer)
{
// printf("Patch %u => %u\n", i, comp_i);

  std::vector<int> groups;
  if(num_edges > 0)
  {  
    int nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
    for(unsigned i=0; i<nrPatches; i++)
    {
      int comp_i = u->find(i);
      
// printf("Patch %u => %u\n", i, comp_i);
      // check if this group already exists
      bool isGroup = false;
      for(unsigned g=0; g<groups.size(); g++)
        if(groups[g] == comp_i)
          isGroup = true;
      
      if(!isGroup)
      {
        groups.push_back(comp_i);
        Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
        float color = GetRandomColor();
        p0->DrawGestalt3D(tgRenderer, true, color);
        
        for(unsigned j=i+1; j<nrPatches; j++)
        {
// printf("Check i-j: %u-%u\n", i, j);
          int comp_j = u->find(j);
          Patch3D *p1 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);
        
          if(comp_i == comp_j)
            p1->DrawGestalt3D(tgRenderer, true, color);
        }
      }
    }
  }
}

} 











