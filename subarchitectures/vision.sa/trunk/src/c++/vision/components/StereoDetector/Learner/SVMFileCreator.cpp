/**
 * @file SVMFileCreator.cpp
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Store relations between features in a file for svm-training.
 */

#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include "SVMFileCreator.h"
#include "Patch3D.h"

namespace Z
{

/**
 * @brief Constructor of SVMFileCreator
 */
SVMFileCreator::SVMFileCreator()
{
  relations = new CalculateRelations();
}


/**
 * @brief Destructor of SVMFileCreator
 */
SVMFileCreator::~SVMFileCreator()
{
}

/**
 * @brief Process the relation extraction algorithm
 */
void SVMFileCreator::Process(pclA::PlanePopout *pp, KinectCore *kc, TGThread::TomGineThread *tgR)
{
  kcore = kc;
  planePopout = pp;
//   tgRenderer = tgR;
  
  // get number of patches and the labels
  unsigned nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
  for(unsigned i=0; i<nrPatches; i++)
  {
    Patch3D *p = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    p->SetObjectLabel(planePopout->IsInSOI(p->GetCenter3D()));
  }

  std::vector<Relation> relation_vector;
  relations->Initialize(kc);
  relations->CalcRelations(relation_vector);
  
  // add relations to file!
  WriteResults2File(relation_vector);
}

/** 
 * @brief Write the results of the relation calculation to a file
 */
void SVMFileCreator::WriteResults2File(std::vector<Relation> &rel)
{
  FILE *file = fopen("SVM-Trainingsset.txt", "w");
    for(unsigned i=0; i<rel.size(); i++)
  {
    fprintf(file,"%u ", rel[i].groundTruth);
    for(unsigned j=0; j<rel[i].rel_value.size(); j++)
      fprintf(file,"%u:%6.5f ", j, rel[i].rel_value[j]);
    fprintf(file,"\n");
  }
  fclose(file);
}

/** 
 * @brief Read the results of the learning algorithm from a file.
 */
// void SVMFileCreator::ReadResultsFromFile()
// {
//   double p_mean, p_var, p_devi, n_mean, n_var, n_devi;
//   std::ifstream myfile ("LearnedProperties.txt");
//   for(unsigned i=0; i<numProperties; i++) 
//   {
//     if(!myfile.eof())
//     {
//       myfile >> p_mean >> p_var >> p_devi >> n_mean >> n_var >> n_devi;
// printf("probs %u: %6.5f %6.5f %6.5f %6.5f %6.5f %6.5f\n", i, p_mean, p_var, p_devi, n_mean, n_var, n_devi);
//     }
//     if(i==0) proximityPP->SetDistributionValues(p_mean, p_var, p_devi, n_mean, n_var, n_devi);
//     else if(i==1) colorPP->SetDistributionValues(p_mean, p_var, p_devi, n_mean, n_var, n_devi);
//     else if(i==2) coplanarityPatchesNormals->SetDistributionValues(p_mean, p_var, p_devi, n_mean, n_var, n_devi);
//   }
// }

} 











