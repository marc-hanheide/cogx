/**
 * @file Link.cpp
 * @author Andreas Richtsfeld
 * @date March 2011
 * @version 0.1
 * @brief Link of a object representation.
 */


#include "Link.h"

namespace Z
{
 
/**
 * @brief Constructor for a new link
 */
Link::Link(double prob)
{
  probability = prob;
  reverseProb.push_back(1-prob);
  
  flag_delete = false;
  flag_considered = false;
}


/**
 * @brief Update the probability value of a link.
 * We calculate the parallel-probability of all links between this node 
 * (from the reverse probabilities).
 * @param prob New probability
 */
void Link::UpdateProbability(double prob)
{
  reverseProb.push_back((1-prob));
  double newProb = 1;
  for(unsigned i=0; i<reverseProb.size(); i++)
    newProb *= reverseProb[i];
  probability = 1 - newProb;
}


/**
 * @brief Add a new link to the object representation.
 * @param value Decrease probability about this value.
 * @return Return true, if probability is still higher than 0.
 */
bool Link::DecreaseProbability(double value)
{
  probability -= value;
  if(probability < 0.0) return false;
  else return true;
}
  
}
