/**
 * @file Link.h
 * @author Andreas Richtsfeld
 * @date March 2011
 * @version 0.1
 * @brief Link of a object representation.
 */

#ifndef Z_OBJREP_LINK_H
#define Z_OBJREP_LINK_H

#include <cstdio>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Node.h"

namespace Z
{
  
/**
 * @brief Class Link: Link of a object representation.
 */
class Link
{
private:    
  unsigned nodeID[2];                   ///< Unique node id to identify the assigned nodes
  
  std::vector<double> reverseProb;      ///< The reverse probabilities of all links between this nodes.
  double probability;                   ///< Probability of the link between the two nodes.
  
  bool flag_delete;                     /// True, if link is marked for deletion
  bool flag_considered;                 ///< True, if already considered in graph (helper flag)
  
public:
  Link(double prob);
  
  void SetNode(unsigned side, unsigned id) {nodeID[side] = id;}
  unsigned GetNodeID(int which)  {return nodeID[which];}
  
  void UpdateProbability(double prob);
  double GetProbability() {return probability;}
  bool DecreaseProbability(double value);
  
  void SetUnconsidered() {flag_considered = false;}
  void SetConsidered() {flag_considered = true;}
  bool IsConsidered() {return flag_considered;}
    
  void SetDeleteFlag() {flag_delete = true;}
  void UnsetDeleteFlag() {flag_delete = false;}
  bool GetDeleteFlag() {return flag_delete;}

};

}

#endif
