/**
 * @file Rectangle3D.cpp
 * @author Andreas Richtsfeld
 * @date January 2011
 * @version 0.1
 * @brief Base class for stereo calculated Rectangles in 3D.
 */

#include "Rectangle3D.h"

namespace Z
{

/**
 * @brief Constructor of Class Rectangle3D
 */
Rectangle3D::Rectangle3D() : Gestalt3D(Gestalt3D::RECTANGLE)
{
}

/**
 * @brief Calculate significance value for the new estimated stereo Gestalt.
 * @param sigLeft Significance value of the Gestalt from the left image.
 * @param sigRight Significance value of the Gestalt from the right image.
 */
void Rectangle3D::CalculateSignificance(double sigLeft, double sigRight)
{
  sig = 1- (1-sigLeft) * (1-sigRight);
}

/**
 * @brief Get the nodes vector for this 3D Gestalt.
 * @param nodes Vector with n nodes, consisting of 3 position values.
 * @return Returns true, if nodes from this 3D Gestalt are available.
 */
bool Rectangle3D::GetLinks(vector<GraphLink> &links)
{
//   printf("Rectangle3D::GetLinks: sig: %4.3f\n", sig);

  for(unsigned id=0; id<surf.vertices.Size(); id++)
  {
    unsigned id2 = id+1;
    if(id2 >= surf.vertices.Size()) id2 = 0;
    
    GraphLink l;
    l.node[0].x = surf.vertices[id].p.x;
    l.node[0].y = surf.vertices[id].p.y;
    l.node[0].z = surf.vertices[id].p.z;
    l.node[1].x = surf.vertices[id2].p.x;
    l.node[1].y = surf.vertices[id2].p.y;
    l.node[1].z = surf.vertices[id2].p.z;
    l.probability = sig;
    links.push_back(l);
  }
  return true;
}

}


