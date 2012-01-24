/**
 * @file Node.h
 * @author Andreas Richtsfeld
 * @date March 2011
 * @version 0.1
 * @brief Node of a object representation.
 */

#ifndef Z_OBJREP_NODE_H
#define Z_OBJREP_NODE_H

#include <cstdio>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Z
{
  
  
/**
 * @brief Class Node: Node of a object representation
 */
class Node
{
private:   
  unsigned id;                       /// Unique id of the node

  std::vector<cv::Point3d> allPos;   /// 3D position of the node
  std::vector<double> allPosSig;     /// Significance value for each 3D position of the node (from the link)

  cv::Point3d pos;                   /// 3D position of the node
  
  bool flag_delete;                  /// True, if no link for this node is available.
  bool flag_considered;              /// True, if already in an object graph.

  void CheckVectorSize(unsigned maxSize);

public:
  Node(unsigned _id, cv::Point3d p, double sig);
  
  unsigned ID() {return id;}

  double Distance2D(const cv::Point3d point);
  double Distance3D(const cv::Point3d point);
  const cv::Point3d GetPosition() {return pos;}
  void UpdatePosition(const cv::Point3d p, double sig);			/// TODO Hier wird einfach der Mittelwert aus der neuem und alten Position errechnet!
  
  void SetDeleteFlag()   { flag_delete = true; }
  void UnsetDeleteFlag() { flag_delete = false; }
  bool GetDeleteFlag()   { return flag_delete; }

  void SetUnconsidered() { flag_considered = false; }
  void SetConsidered()   { flag_considered = true; }
  bool IsConsidered()    { return flag_considered; }
};

}

#endif
