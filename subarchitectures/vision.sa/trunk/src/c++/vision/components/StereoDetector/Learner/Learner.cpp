/**
 * @file Learner.cpp
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Learn about image features
 */

#include <vector>
#include "Learner.h"
#include "Patch3D.h"
#include "Histogram.h"

namespace Z
{

/**
 * @brief Constructor of Learner
 */
Learner::Learner()
{
}


/**
 * @brief Destructor of Learner
 */
Learner::~Learner()
{
}

/**
 * @brief Process the learning algorithms
 */
void Learner::Process(KinectCore *kc)
{
  kcore = kc;

  LearnClosenessBetweenPatches();
  LearnColorSimilarityBetweenPatches();
}

/**
 * @brief Learn about the closeness between patches.
 * What is close???
 */
void Learner::LearnClosenessBetweenPatches()
{
  std::vector<double> data;
  Histogram *hist;
  
  printf("Learner::LearnClosenessBetweenPatches: TODO: Time to implement!\n");
  /// Get all pairs of patches
  int nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
  for(unsigned i=0; i<nrPatches; i++)
  {
    Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=i+1; j<nrPatches; j++)
    {
      Patch3D *p1 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);
      
      /// We have now a patch pair: Do they belong together? TRUE/FALSE example?
      
      
      /// Calculate the color similarity value
      double closeness = p0->IsClose(p1);
      printf("closeness[%u][%u]: %4.2f\n", i, j, closeness);

      data.push_back(closeness);
    }
  }
  
  hist = new Histogram(10, data);
}


/**
 * @brief Learn about the color similarity between patches.
 */
void Learner::LearnColorSimilarityBetweenPatches()
{
  printf("Learner::LearnColorSimilarityBetweenPatches: TODO: Time to finish implementation!\n");
  
  /// Get all pairs of patches
  int nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
  for(unsigned i=0; i<nrPatches; i++)
  {
    Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=i+1; j<nrPatches; j++)
    {
      Patch3D *p1 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);
    
      /// We have now a patch pair: Do they belong together? TRUE/FALSE example?
      
      
      /// Calculate the color similarity value
      double colorSimilarity = p0->CompareColor(p1);
      printf("colorSimilarity[%u][%u]: %4.2f\n", i, j, colorSimilarity);
    }
  }
}

} 











