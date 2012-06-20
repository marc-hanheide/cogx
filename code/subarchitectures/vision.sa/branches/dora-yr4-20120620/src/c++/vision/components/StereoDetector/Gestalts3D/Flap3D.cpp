/**
 * @file Flap3D.cpp
 * @author Andreas Richtsfeld
 * @date January 2011
 * @version 0.1
 * @brief Base class for stereo calculated Flaps in 3D.
 */

#include "Flap3D.h"
#include "Flap.hh"

namespace Z
{

/**
 * @brief Constructor of class Corner3D.
 */
Flap3D::Flap3D() : Gestalt3D(Gestalt3D::FLAP)
{
  pointsCalculated = false;
}

/**
 * @brief Calculate the 6 corner points of the flap.
 * TODO Die äußeren vier Punkte sollten auf die Fläche projeziert werden, die von den inneren 
 * Punkten und dem Punkt zwischen den beiden äußeren aufgespannt wird!
 */
void Flap3D::CalcIdealFlap()
{
  if(pointsCalculated) return;
    
  // Estimate the joint junctions
  point[0].p = (surf[0].vertices[0].p + surf[1].vertices[3].p)/2.;
  point[3].p = (surf[0].vertices[3].p + surf[1].vertices[0].p)/2.;
  
  // Calculate the outer points of the flap
  point[1].p = surf[0].vertices[1].p;
  point[2].p = surf[0].vertices[2].p;
  point[4].p = surf[1].vertices[1].p;
  point[5].p = surf[1].vertices[2].p;

  pointsCalculated = true;
}

/**
 * @brief Calculate the 6 corner points of the flap.
 * TODO Die äußeren vier Punkte sollten auf die Fläche projeziert werden, die von den inneren 
 * Punkten und dem Punkt zwischen den beiden äußeren aufgespannt wird!
 */
bool Flap3D::GetPoints(Vertex3D p[6])
{
  for(unsigned i=0; i<6; i++)
    p[i] = point[i];
  return pointsCalculated;
}

/**
 * @brief Get the nodes vector for this 3D Gestalt.
 * @param nodes Vector with n nodes, consisting of 3 position values.
 * @return Returns true, if nodes from this 3D Gestalt are available.
 */
bool Flap3D::GetLinks(std::vector<GraphLink> &links)
{
// printf("Flap3D::GetLinks: sig: %4.3f\n", sig);

 for(unsigned id=0; id<6; id++)
  {
    unsigned id2 = id+1;
    if(id2 >= 6) id2 = 0;
    
    GraphLink l;
    l.node[0].x = point[id].p.x;
    l.node[0].y = point[id].p.y;
    l.node[0].z = point[id].p.z;
    l.node[1].x = point[id2].p.x;
    l.node[1].y = point[id2].p.y;
    l.node[1].z = point[id2].p.z;
    l.probability = 0.9;
    links.push_back(l);
  }
  
  // link between the two rectangles
  GraphLink l;
  l.node[0].x = point[0].p.x;
  l.node[0].y = point[0].p.y;
  l.node[0].z = point[0].p.z;
  l.node[1].x = point[3].p.x;
  l.node[1].y = point[3].p.y;
  l.node[1].z = point[3].p.z;
  l.probability = 0.9;
  links.push_back(l);

  return true;
}

}


