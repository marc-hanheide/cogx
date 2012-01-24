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
void SVMFileCreator::Process(KinectCore *kc)
{
printf("SVMFileCreator::Process: start!\n");
  kcore = kc;
  kcore->GetCameraParameters(cam_fx, cam_fy, cam_cx, cam_cy);
  
  std::vector<Relation> relation_vector;
  relations->Initialize(kc, cam_fx, cam_fy, cam_cx, cam_cy);                  /// TODO Move to constructor???
  relations->CalcSVMRelations(relation_vector);
  relations->PrintRelations();
  
  // add relations to file!
  WriteResults2File(relation_vector);
printf("SVMFileCreator::Process: end!\n");
}

/** 
 * @brief Write the results of the relation calculation to a file
 */
void SVMFileCreator::WriteResults2File(std::vector<Relation> &rel)
{
  FILE *PPfile = fopen("PP-Trainingsset.txt", "w");
  FILE *PLfile = fopen("PL-Trainingsset.txt", "w");
  FILE *LLfile = fopen("LL-Trainingsset.txt", "w");
  for(unsigned i=0; i<rel.size(); i++)
  {
    if(rel[i].type == 1)
    {
      fprintf(PPfile,"%u ", rel[i].groundTruth);
      for(unsigned j=0; j<rel[i].rel_value.size(); j++)
        fprintf(PPfile,"%u:%6.5f ", j, rel[i].rel_value[j]);
      fprintf(PPfile,"\n");
    }
    if(rel[i].type == 2)
    {
      fprintf(PLfile,"%u ", rel[i].groundTruth);
      for(unsigned j=0; j<rel[i].rel_value.size(); j++)
        fprintf(PLfile,"%u:%6.5f ", j, rel[i].rel_value[j]);
      fprintf(PLfile,"\n");
    }
    if(rel[i].type == 3)
    {
      fprintf(LLfile,"%u ", rel[i].groundTruth);
      for(unsigned j=0; j<rel[i].rel_value.size(); j++)
        fprintf(LLfile,"%u:%6.5f ", j, rel[i].rel_value[j]);
      fprintf(LLfile,"\n");
    }
  }
  fclose(PPfile);
  fclose(PLfile);
  fclose(LLfile);
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











