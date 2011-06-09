/**
 * @file ObjRep.h
 * @author Andreas Richtsfeld
 * @date March 2011
 * @version 0.1
 * @brief Object representation as probabilistic graph, consisting of nodes and links.
 */

#ifndef Z_OBJREP_H
#define Z_OBJREP_H

#include <vector>
#include <cstdio>
#include "Math.hh"
#include "Node.h"
#include "Link.h"
#include "StereoCore.h"

namespace Z
{
 
// Minimum distance between existing node and new candidate, to be the same node (*10 in z-direction)
static const double DISTANCE_THRESHOLD = 0.01;

// Threshold for decreasing the probability of each link after each frame.
// NOTE Be carefull: This is also the minimum threshold for staying more than one frame!
static const double PROBABILITY_DEC_THRESHOLD = 0.2;
  
/**
 * @brief Class ObjRep: Object Representation as graph, consisting of nodes and links.
 */
class ObjRep
{
private:
  std::vector<Node> nodes;              ///< Graph nodes: points which are connected via links
  std::vector<Link> links;              ///< Graph links: linking probability between two nodes

  unsigned GetUniqueID();
  void PrintGraph();

  void AddLink(GraphLink &link);

  Node GetNode(unsigned id);
  unsigned GetNodePos(unsigned id);
  bool SetNodeConsidered(unsigned id);
  void SetNodesUnconsidered();
  bool IsNodeConsidered(unsigned id);

  void UnsetNodeDeleteFlag(unsigned nodeID);
  void DeleteCheckedNodes();
  void DeleteUnusedNodes();
  unsigned IsNode(const cv::Point3d position);

  void SetLinksUnconsidered();

  void CheckLinkProbability();
  void DeleteCheckedLinks();
  unsigned IsLink(unsigned node_0, unsigned node_1);
  void UpdateLink();
  
public:
  ObjRep();

  void Process(Z::StereoCore *sc);
  bool GetGraphModel(std::vector<cv::Point3d> &first, std::vector<cv::Point3d> &second, std::vector<double> &probability);
  bool GetObjectGraphModel(std::vector< std::vector<cv::Point3d> > &first, 
			   std::vector< std::vector<cv::Point3d> > &second,
			   std::vector< std::vector<double> > &probability, 
			   std::vector< std::vector<std::string> > &link, 
			   std::vector< std::vector<std::string> > &node_0, 
			   std::vector< std::vector<std::string> > &node_1);

};
}

#endif