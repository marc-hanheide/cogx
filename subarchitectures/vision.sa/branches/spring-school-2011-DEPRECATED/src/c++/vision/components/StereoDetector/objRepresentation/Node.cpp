/**
 * @file Node.cpp
 * @author Andreas Richtsfeld
 * @date March 2011
 * @version 0.1
 * @brief Node of a object representation.
 */


#include "Node.h"

namespace Z
{
 
/**
 * @brief Constructor for a new node
 * @param _id Unique id of the node
 * @param p Position of the new node
 */
Node::Node(unsigned _id, cv::Point3d p, double sig)
{
  id = _id;
  flag_delete = false;
  flag_considered = false;
  
//   allPos.clear();
  allPos.push_back(p);
  allPosSig.push_back(sig);
  pos = p;
}

/**
 * @brief Get the 2D distance between two nodes. 
 * We do not consider the z-axis.
 * @param point 3D point
 * @return Returns the distance to the point.
 */
double Node::Distance2D(const cv::Point3d point)
{
  cv::Point3d dist = point - pos;
  return sqrt(dist.x*dist.x + dist.y*dist.y);
}
  
/**
 * @brief Get the distance from the node to another node.
 * @param point 3D point
 * @return Returns the distance to the point.
 */
double Node::Distance3D(const cv::Point3d point)
{
  cv::Point3d dist = this->pos - point;
  double length = sqrt(dist.x*dist.x + dist.y*dist.y + dist.z*dist.z);
  return length;
}


/**
 * @brief Update the position of the node. Calculate the mean from all matched node positions.
 * @param maxSize Maximum size of the vector arrays to avoid memory overload!
 */
void Node::CheckVectorSize(unsigned maxSize)
{
  if(allPos.size() > maxSize)
    allPos.erase(allPos.begin());
  if(allPosSig.size() > maxSize)
    allPosSig.erase(allPosSig.begin());
}


/**
 * @brief Update the position of the node. Calculate the mean from all matched node positions.
 * @param p New position of node
 * @param sig Significance of the link from the node
 */
void Node::UpdatePosition(const cv::Point3d p, double sig)
{
  allPos.push_back(p);
  allPosSig.push_back(sig);

  CheckVectorSize(15);

  pos.x=0.; pos.y=0.; pos.z=0.;
  double sigSum = 0.;
  for(unsigned i=0; i<allPos.size(); i++)
  {
    pos.x += allPos[i].x  * allPosSig[i];
    pos.y += allPos[i].y  * allPosSig[i];
    pos.z += allPos[i].z  * allPosSig[i];
    sigSum += allPosSig[i];
  }
  pos.x /= sigSum;
  pos.y /= sigSum;
  pos.z /= sigSum;
}

  
}
