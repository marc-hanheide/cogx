/**
 * @file CalculateRelations.cpp
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Store relations between features in a file for svm-training.
 */

#include "CalculateRelations.h"

namespace Z
{
  
/**
 * @brief Constructor of class CalculateRelations.
 */
CalculateRelations::CalculateRelations()
{
  isInitialized = false;
}

/**
 * @brief Constructor of class CalculateRelations.
 */
void CalculateRelations::Initialize(KinectCore *k)
{
  kcore = k;
  isInitialized = true;

  allRelations.resize(0);
  ppRelations.resize(0);
  plRelations.resize(0);
}


  
void CalculateRelations::CalcRelations(std::vector<Relation> &rel)
{
printf("CalculateRelations::CalcRelations: start\n");
  CalcPatchPatchRelations(rel);
  // ...  TODO TODO TODO
  rel = allRelations;
}



void CalculateRelations::CalcPatchPatchRelations(std::vector<Relation> &rel)      /// TODO TODO TODO We need here the ground truth!
{
  std::vector<double> positive, negative;
  unsigned nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
  for(unsigned i=0; i<nrPatches-1; i++)
  {
    Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=i+1; j<nrPatches; j++)
    {
      Patch3D *px = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);
      
// printf("  Calculate PP-Relation: %u-%u (%u-%u)\n", p0->GetNodeID(), px->GetNodeID(), p0->GetObjectLabel(), px->GetObjectLabel());
      // calculate relation values
      double proximity = p0->IsClose(px);
      double colorSimilarity = p0->CompareColor(px);
      double normal_angle, plane_distance; 
      p0->CalculateCoplanarity(px, normal_angle, plane_distance);
      
      // check if it belongs to the same object
      if(p0->GetObjectLabel() != 0 && p0->GetObjectLabel() == px->GetObjectLabel())
      {
// printf("  => true example!\n");
        Relation r;
        r.groundTruth = 1;
        r.prediction = -1;
        r.type = 1;
        r.id_0 = p0->GetNodeID();
        r.id_1 = px->GetNodeID();
        r.rel_value.push_back(proximity);
        r.rel_value.push_back(colorSimilarity);
        r.rel_value.push_back(normal_angle);
        r.rel_value.push_back(plane_distance);
        ppRelations.push_back(r);
        allRelations.push_back(r);
      }
      else if((p0->GetObjectLabel() != 0 || px->GetObjectLabel() != 0) &&           /// TODO TODO Hier werden nicht 2 versch. Objekte berücksichtigt! z.B. 1-2 labels
               p0->GetObjectLabel() != px->GetObjectLabel())
      {
// printf("  => false example!\n");
        Relation r;
        r.groundTruth = 0;
        r.prediction = -1;
        r.type = 1;
        r.id_0 = p0->GetNodeID();
        r.id_1 = px->GetNodeID();
        r.rel_value.push_back(proximity);
        r.rel_value.push_back(colorSimilarity);
        r.rel_value.push_back(normal_angle);
        r.rel_value.push_back(plane_distance);
        ppRelations.push_back(r);
        allRelations.push_back(r);
      }
    }
  }
// printf("CalculateRelations::CalcPatchPatchRelations: allRelations: %u\n", allRelations.size());
  rel = ppRelations;
}



void CalculateRelations::CalcAllRelations(std::vector<Relation> &rel)      /// TODO TODO TODO Without ground truth!!!
{
  std::vector<double> positive, negative;
  unsigned nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
  for(unsigned i=0; i<nrPatches-1; i++)
  {
    Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=i+1; j<nrPatches; j++)
    {
      Patch3D *px = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);
      
// printf("  Calculate PP-Relation: %u-%u (%u-%u)\n", p0->GetNodeID(), px->GetNodeID(), p0->GetObjectLabel(), px->GetObjectLabel());
      // calculate relation values
      double proximity = p0->IsClose(px);
      double colorSimilarity = p0->CompareColor(px);
      double normal_angle, plane_distance; 
      p0->CalculateCoplanarity(px, normal_angle, plane_distance);
      

      /// TODO TODO TODO Create relation anyway
      Relation r;
      r.groundTruth = -1;
      r.prediction = -1;
      r.type = 1;
      r.id_0 = p0->GetNodeID();
      r.id_1 = px->GetNodeID();
      r.rel_value.push_back(proximity);
      r.rel_value.push_back(colorSimilarity);
      r.rel_value.push_back(normal_angle);
      r.rel_value.push_back(plane_distance);
      ppRelations.push_back(r);
      allRelations.push_back(r);
    }
  }
// printf("CalculateRelations::CalcPatchPatchRelations: allRelations: %u\n", allRelations.size());
  rel = ppRelations;
}


void CalculateRelations::PrintResults()
{
  printf("Results of relation calculation:\n");
  printf("  Number of relations: %u\n", allRelations.size());
  printf("  Accuracy: %4.3f\n", CheckAccuracy());
  for(unsigned i=0; i< allRelations.size(); i++)
  {
    if(allRelations[i].rel_probability.size() >= 1) // check, if we have at least 2 labes (true/false)
    {
      if(allRelations[i].prediction == allRelations[i].groundTruth)
        printf("  %u: %u-%u => gt(%u) - pr(%u) (true) with prob: %4.3f\n", i, allRelations[i].id_0, allRelations[i].id_1, 
            allRelations[i].groundTruth, allRelations[i].prediction, allRelations[i].rel_probability[1]);
      else
        printf("  %u: %u-%u => gt(%u) - pr(%u) (false) with prob: %4.3f\n", i, allRelations[i].id_0, allRelations[i].id_1, 
            allRelations[i].groundTruth, allRelations[i].prediction, allRelations[i].rel_probability[1]);
    }
  }
}

/**
 * @brief Check the accuracy of the prediction if values are available
 * @return Accuracy of prediction.
 */
double CalculateRelations::CheckAccuracy()
{
  unsigned truePredict = 0, falsePredict = 0;
  for(unsigned i=0; i<allRelations.size(); i++)
  {
    if(allRelations[i].groundTruth == allRelations[i].prediction)
      truePredict++;
    else
      falsePredict++;
  }  
  
  if(allRelations.size() > 0)
    return ((double)truePredict / ((double) (truePredict + falsePredict)));
  else
    return 0.0;
}
  
}






















