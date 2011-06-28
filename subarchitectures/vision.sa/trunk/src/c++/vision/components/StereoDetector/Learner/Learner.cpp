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
  delete kcore;
  delete closenessHisto;
  delete colorHisto;
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
  std::vector<double> dataX;
  
  /// Get all pairs of patches
  int nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
  printf("Learner::LearnClosenessBetweenPatches: number of patches: %u\n", nrPatches);

  for(unsigned i=0; i<nrPatches-1; i++)
  {
    Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=i+1; j<nrPatches; j++)
    {
//       Patch3D *p1 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);
      Patch3D *px = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);

      /// We have now a patch pair: Do they belong together? TRUE/FALSE example?
      
      
      /// Calculate the color similarity value
      double closeness = p0->IsClose(px);
//       double closeness = p0->CompareColor(px);

//       printf("closeness[%u][%u]: %4.2f\n", i, j, closeness);

      dataX.push_back(closeness);
    }
  }
//   Histogram hist(10, data);
  closenessHisto = new Histogram(20, dataX);

printf("LearnClosenessBetweenPatches: Fehler?\n");
}


/**
 * @brief Learn about the color similarity between patches.
 */
void Learner::LearnColorSimilarityBetweenPatches()
{
  std::vector<double> data2;
  
  /// Get all pairs of patches
  int nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
//   printf("Learner::LearnColorSimilarityBetweenPatches: number of patches: %u\n", nrPatches);
  
  for(unsigned i=0; i<nrPatches-1; i++)
  {
// printf("Learn: Fehler: 1\n");
    Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=i+1; j<nrPatches; j++)
    {
// printf("Learn: Fehler: 1 -1\n");

      Patch3D *p1 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);
// printf("Learn: Fehler: 1 -2\n");

      /// We have now a patch pair: Do they belong together? TRUE/FALSE example?
      
      
      /// Calculate the color similarity value
      double colorSimilarity = p0->CompareColor(p1);
//       printf("colorSimilarity[%u][%u]: %4.2f\n", i, j, colorSimilarity);
      
      data2.push_back(colorSimilarity);
    }
  }
//   Histogram hist(10, data2);
  colorHisto = new Histogram(20, data2);
  
printf("LearnColorSimilarityBetweenPatches: Passiert hier der Fehler bevor das histogram geprinted wird?\n");
}

} 











