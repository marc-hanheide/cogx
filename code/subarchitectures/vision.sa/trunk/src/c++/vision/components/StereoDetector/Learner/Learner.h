/**
 * @file Learner.h
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Learn about image features
 */

#ifndef Z_LEARNER_H
#define Z_LEARNER_H

#include "KinectCore.h"
#include "LearnPropBase.h"

#include "v4r/PCLAddOns/PlanePopout.hh"
#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/PCLAddOns/PCLFunctions.h"
#include "v4r/TomGine/tgTomGineThread.h"

namespace Z
{

/**
 * @brief Class Learner: 
 */
class Learner
{
public:
  
private:
  TomGine::tgTomGineThread *tgRenderer;                      ///< TODO 3D render engine => only for debugging necessary

  KinectCore *kcore;
  pclA::PlanePopout *planePopout;
  
//   Histogram *closenessHisto;       /// TODO Diese Histogramme können später zum verifizieren der Wahrscheinlichkeitsverteilung benutzt werden.
//   Histogram *colorHisto;
  
  int numProperties;
  LearnPropBase *proximityPP;
  LearnPropBase *colorPP;
  LearnPropBase *coplanarityPatchesNormals;
  LearnPropBase *coplanarityPatchesDistance;
  
  int nrPatches;                                  // Number of patches
  
//   void LearnLJunctionsBetweenLines();
//   void LearnParallelities();
//   void LearnCollinearities();
//   void LearnClosures();
//   void LearnRectangles();
  void LearnProximityPP();
  void LearnColorSimilarityBetweenPatches();
  void LearnCoplanarityBetweenPatches();
//   void LearnCoplanaritiesBetweenClosuresAndPatches();
//   void LearnConcavitiesConvexitiesBetweenPatches();
//   void LearnCommonMotion();
  

public:
  Learner();
  ~Learner();
  
  void Process(pclA::PlanePopout *pp, KinectCore *kc, TomGine::tgTomGineThread *tgR);

  void GetPosProximityBetweenPatches(double &mean, double &variance, double &st_devi);
  void GetNegProximityBetweenPatches(double &mean, double &variance, double &st_devi);
  float GetPProximityPP(const double &val);

  void GetPosColorSimilarityBetweenPatches(double &mean, double &variance, double &st_devi);
  void GetNegColorSimilarityBetweenPatches(double &mean, double &variance, double &st_devi);
  float GetPColorSimilarityPP(const double &val);

  void GetPosCoplanarityNormalsBetweenPatches(double &mean, double &variance, double &st_devi);
  void GetNegCoplanarityNormalsBetweenPatches(double &mean, double &variance, double &st_devi);
  float GetPCoplanarityPP(const double &val);

  void GetPosCoplanarityDistanceBetweenPatches(double &mean, double &variance, double &st_devi);
  void GetNegCoplanarityDistanceBetweenPatches(double &mean, double &variance, double &st_devi);
  void GetCoplanarityDistanceBetweenPatchesProbability(const double &val, double &prob);
  
  void WriteResults2File();
  void ReadResultsFromFile();
};

}

#endif

