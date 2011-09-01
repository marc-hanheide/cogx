/**
 * $Id$
 */


#include "IMCluster.hh"

namespace P 
{

void DeleteIMClusters(std::vector<IMCluster *> &clusters)
{
  for (unsigned i=0; i<clusters.size(); i++)
    delete clusters[i];
  clusters.clear();
}



}

