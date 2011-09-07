/**
 * @file Learner.cpp
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Learn about image features
 */

#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include "Learner.h"
#include "Patch3D.h"

namespace Z
{

/**
 * @brief Constructor of Learner
 */
Learner::Learner()
{
  numProperties = 3;                                  /// currently, we are learning 3 properties
  proximityPP = new LearnPropBase();
  colorPP = new LearnPropBase();
  coplanarityPatchesNormals = new LearnPropBase();
  coplanarityPatchesDistance = new LearnPropBase();
}


/**
 * @brief Destructor of Learner
 */
Learner::~Learner()
{
  delete kcore;
  delete proximityPP;
  delete colorPP;
  delete coplanarityPatchesNormals;
  delete coplanarityPatchesDistance;
}

/**
 * @brief Process the learning algorithms
 */
void Learner::Process(pclA::PlanePopout *pp, KinectCore *kc, TomGine::tgTomGineThread *tgR)
{
  kcore = kc;
  planePopout = pp;
  tgRenderer = tgR;
  
  // Learn the different probabilities of the Gestalt principle values
  LearnProximityPP();
  LearnColorSimilarityBetweenPatches();
  LearnCoplanarityBetweenPatches();
// printf("  Learner::Process: Learn end\n");
}


/**
 * @brief Learn the proximity probability function between patches.
 */
void Learner::LearnProximityPP()
{
  std::vector<double> positive, negative;
  for(unsigned i=0; i<nrPatches-1; i++)
  {
    Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=i+1; j<nrPatches; j++)
    {
      Patch3D *px = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);
      double closeness = p0->CalculateProximity(px);

      if(p0->GetObjectLabel() != 0 && p0->GetObjectLabel() == px->GetObjectLabel())
        positive.push_back(closeness);
      else if((p0->GetObjectLabel() != 0 || px->GetObjectLabel() != 0) &&                     /// TODO Hier werden nicht verschiedene Objekte berücksichtigt!
               p0->GetObjectLabel() != px->GetObjectLabel())
        negative.push_back(closeness);
    }
  }
  proximityPP->AddValues(positive, negative);
}

/**
 * @brief Get the distribution values for the closeness between patches.
 * @param mean Mean value of the distribution. 
 * @param variance Variance of the distribution.
 * @param st_devi Standard deviation
 */
void Learner::GetPosProximityBetweenPatches(double &mean, double &variance, double &st_devi)
{
  mean = proximityPP->GetPosMeanValue();
  variance = proximityPP->GetPosVariance();
  st_devi = proximityPP->GetPosStDeviation();
}

/**
 * @brief Get the distribution values for the closeness between patches.
 * @param mean Mean value of the distribution. 
 * @param variance Variance of the distribution.
 * @param st_devi Standard deviation
 */
void Learner::GetNegProximityBetweenPatches(double &mean, double &variance, double &st_devi)
{
  mean = proximityPP->GetNegMeanValue();
  variance = proximityPP->GetNegVariance();
  st_devi = proximityPP->GetNegStDeviation();
}


/**
 * @brief Get the probability for a given value.
 * @param val Distance value
 * @return Probability, for that distance that patches belong together.
 */
float Learner::GetPProximityPP(const double &val)
{
  return proximityPP->GetProbability(val) ;
}


/**
 * @brief Learn about the color similarity between patches.
 */
void Learner::LearnColorSimilarityBetweenPatches()
{
  std::vector<double> positive, negative;
  
  for(unsigned i=0; i<nrPatches-1; i++)
  {
    Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=i+1; j<nrPatches; j++)
    {
      Patch3D *px = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);
      double colorSimilarity = p0->CompareColor(px);

      if(p0->GetObjectLabel() != 0 && p0->GetObjectLabel() == px->GetObjectLabel())
        positive.push_back(colorSimilarity);
      else if((p0->GetObjectLabel() != 0 || px->GetObjectLabel() != 0) && 
               p0->GetObjectLabel() != px->GetObjectLabel())
        negative.push_back(colorSimilarity);
    }
  }
  colorPP->AddValues(positive, negative);
}

/**
 * @brief Get the distribution values for the closeness between patches.
 * @param mean Mean value of the distribution. 
 * @param variance Variance of the distribution.
 * @param st_devi Standard deviation
 */
void Learner::GetPosColorSimilarityBetweenPatches(double &mean, double &variance, double &st_devi)
{
  mean = colorPP->GetPosMeanValue();
  variance = colorPP->GetPosVariance();
  st_devi = colorPP->GetPosStDeviation();
}

/**
 * @brief Get the distribution values for the closeness between patches.
 * @param mean Mean value of the distribution. 
 * @param variance Variance of the distribution.
 * @param st_devi Standard deviation
 */
void Learner::GetNegColorSimilarityBetweenPatches(double &mean, double &variance, double &st_devi)
{
  mean = colorPP->GetNegMeanValue();
  variance = colorPP->GetNegVariance();
  st_devi = colorPP->GetNegStDeviation();
}


/**
 * @brief Get the probability for a given value.
 * @param val Color value
 * @return Returns the probability
 */
float Learner::GetPColorSimilarityPP(const double &val)
{
  return colorPP->GetProbability(val) ;
}

/**
 * @brief Learn about the coplanarity between patches.
 * TODO What is exactly coplanarity? 
 * + Angle between patch-normals
 *   Problem: Patches are not on the same plain => 
 *   Solution: We consider also the distance between the normals (after rotation to the mean)
 * + Distance between patches, after rotation of both batches to the mean normal.
 * 
 * => At the end we get two values: angle between patches and the distance
 */
void Learner::LearnCoplanarityBetweenPatches()
{
  std::vector<double> pos_normal, neg_normal;
  std::vector<double> pos_distance, neg_distance;

  // Get angle between normals and distance and save it
  for(unsigned i=0; i<nrPatches-1; i++)
  {
    Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);

    // TODO show normals on tgRenderer
//     cv::Point3f nor, cen;
//     nor = p0->GetPlaneNormal(); 
//     cen = p0->GetCenter3D();
//     tgRenderer->AddLine3D(cen.x, cen.y, cen.z, cen.x+(nor.x/10), cen.y+(nor.y/10), cen.z+(nor.z/10), 0, 255, 0);
//     tgRenderer->AddPoint3D(cen.x, cen.y, cen.z, 255, 0, 0);
    
    // learn 
    for(unsigned j=i+1; j<nrPatches; j++)
    {
      Patch3D *px = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);
      
      double normal_angle, plane_distance; 
      p0->CalculateCoplanarity(px, normal_angle, plane_distance);

      if(p0->GetObjectLabel() != 0 && p0->GetObjectLabel() == px->GetObjectLabel())
      {
// printf("Coplanar patch pair found with label: %u => %u, %u: angle/dist: %4.3f / %4.3f\n", patchLabels[i], i, j, normal_angle, plane_distance);
        pos_normal.push_back(normal_angle);
        pos_distance.push_back(plane_distance);
      }
      else if((p0->GetObjectLabel() != 0 || px->GetObjectLabel() != 0) && 
               p0->GetObjectLabel() != px->GetObjectLabel())
      {
// printf("Learner::LearnCoplanarityBetweenPatches: normal angle: %4.3f\n", normal_angle);
if(normal_angle == normal_angle)
        neg_normal.push_back(normal_angle);
else printf("Learner::LearnCoplanarityBetweenPatches: Sometimes is the normal_angle = nan. FIX it NOW!\n");
        neg_distance.push_back(plane_distance);
      }
    }
  }
  coplanarityPatchesNormals->AddValues(pos_normal, neg_normal);
  coplanarityPatchesDistance->AddValues(pos_distance, neg_distance);
}

/**
 * @brief Get the distribution values for the coplanarity between patches.
 * @param mean Mean value of the distribution. 
 * @param variance Variance of the distribution.
 * @param st_devi Standard deviation
 */
void Learner::GetPosCoplanarityNormalsBetweenPatches(double &mean, double &variance, double &st_devi)
{
  mean = coplanarityPatchesNormals->GetPosMeanValue();
  variance = coplanarityPatchesNormals->GetPosVariance();
  st_devi = coplanarityPatchesNormals->GetPosStDeviation();
}

/**
 * @brief Get the distribution values for the coplanarity between patches.
 * @param mean Mean value of the distribution. 
 * @param variance Variance of the distribution.
 * @param st_devi Standard deviation
 */
void Learner::GetNegCoplanarityNormalsBetweenPatches(double &mean, double &variance, double &st_devi)
{
  mean = coplanarityPatchesNormals->GetNegMeanValue();
  variance = coplanarityPatchesNormals->GetNegVariance();
  st_devi = coplanarityPatchesNormals->GetNegStDeviation();
}


/**
 * @brief Get the probability for a given value.
 * @param val Observed value
 * @return Probability
 */
float Learner::GetPCoplanarityPP(const double &val)
{
  return coplanarityPatchesNormals->GetProbability(val) ;
}

/**
 * @brief Get the distribution values for the coplanarity between patches.
 * @param mean Mean value of the distribution. 
 * @param variance Variance of the distribution.
 * @param st_devi Standard deviation
 */
void Learner::GetPosCoplanarityDistanceBetweenPatches(double &mean, double &variance, double &st_devi)
{
  mean = coplanarityPatchesDistance->GetPosMeanValue();
  variance = coplanarityPatchesDistance->GetPosVariance();
  st_devi = coplanarityPatchesDistance->GetPosStDeviation();
}

/**
 * @brief Get the distribution values for the coplanarity between patches.
 * @param mean Mean value of the distribution. 
 * @param variance Variance of the distribution.
 * @param st_devi Standard deviation
 */
void Learner::GetNegCoplanarityDistanceBetweenPatches(double &mean, double &variance, double &st_devi)
{
  mean = coplanarityPatchesDistance->GetNegMeanValue();
  variance = coplanarityPatchesDistance->GetNegVariance();
  st_devi = coplanarityPatchesDistance->GetNegStDeviation();
}


/**
 * @brief Get the probability for a given value.
 * @param val Distance value
 * @param prob Probability, for that distance that patches belong together.
 */
void Learner::GetCoplanarityDistanceBetweenPatchesProbability(const double &val, double &prob)
{
  prob = coplanarityPatchesDistance->GetProbability(val) ;
}

/** 
 * @brief Write the results of the learning algorithm to a file.
 */
void Learner::WriteResults2File()
{
  FILE *file = fopen("LearnedProperties.txt", "w");
  
  double p_mean_p, p_var_p, p_devi_p, n_mean_p, n_var_p, n_devi_p;
  double p_mean_col, p_var_col, p_devi_col, n_mean_col, n_var_col, n_devi_col;
  double p_mean_cop, p_var_cop, p_devi_cop, n_mean_cop, n_var_cop, n_devi_cop;
  GetPosProximityBetweenPatches(p_mean_p, p_var_p, p_devi_p);
  GetNegProximityBetweenPatches(n_mean_p, n_var_p, n_devi_p);
  GetPosColorSimilarityBetweenPatches(p_mean_col, p_var_col, p_devi_col);
  GetNegColorSimilarityBetweenPatches(n_mean_col, n_var_col, n_devi_col);
  GetPosCoplanarityNormalsBetweenPatches(p_mean_cop, p_var_cop, p_devi_cop);
  GetNegCoplanarityNormalsBetweenPatches(n_mean_cop, n_var_cop, n_devi_cop);
  
  fprintf(file,"%6.5f %6.5f %6.5f %6.5f %6.5f %6.5f\n", 
          p_mean_p, p_var_p, p_devi_p, n_mean_p, n_var_p, n_devi_p);    

  fprintf(file,"%6.5f %6.5f %6.5f %6.5f %6.5f %6.5f\n", 
          p_mean_col, p_var_col, p_devi_col, n_mean_col, n_var_col, n_devi_col);    

  fprintf(file,"%6.5f %6.5f %6.5f %6.5f %6.5f %6.5f\n", 
          p_mean_cop, p_var_cop, p_devi_cop, n_mean_cop, n_var_cop, n_devi_cop);    
  
//   fprintf(file, "\n");
  fclose(file);
  
}

/** 
 * @brief Read the results of the learning algorithm from a file.
 */
void Learner::ReadResultsFromFile()
{
  double p_mean, p_var, p_devi, n_mean, n_var, n_devi;
  std::ifstream myfile ("LearnedProperties.txt");
  for(unsigned i=0; i<numProperties; i++) 
  {
    if(!myfile.eof())
    {
      myfile >> p_mean >> p_var >> p_devi >> n_mean >> n_var >> n_devi;
printf("probs %u: %6.5f %6.5f %6.5f %6.5f %6.5f %6.5f\n", i, p_mean, p_var, p_devi, n_mean, n_var, n_devi);
    }
    if(i==0) proximityPP->SetDistributionValues(p_mean, p_var, p_devi, n_mean, n_var, n_devi);
    else if(i==1) colorPP->SetDistributionValues(p_mean, p_var, p_devi, n_mean, n_var, n_devi);
    else if(i==2) coplanarityPatchesNormals->SetDistributionValues(p_mean, p_var, p_devi, n_mean, n_var, n_devi);
  }
}

} 











