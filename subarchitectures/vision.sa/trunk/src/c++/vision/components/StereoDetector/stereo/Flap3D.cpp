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
}


