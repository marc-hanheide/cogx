/**
 * @file CalculateRelations.h
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Calculate relations between different visual features.
 */

#ifndef Z_CALCULATE_RELATIONS_H
#define Z_CALCULATE_RELATIONS_H

#include <vector>
#include "KinectCore.h"
#include "Patch3D.h"

#include "v4r/PCLAddOns/PlanePopout.hh"
#include "v4r/PCLAddOns/utils/PCLUtils.h"
#include "v4r/PCLAddOns/functions/PCLFunctions.h"

namespace Z
{
 
struct Relation
{
  unsigned type;                          ///< Type of relation (Patch-Patch = 1 / ...)
  unsigned id_0;                          ///< id of first feature
  unsigned id_1;                          ///< id of second feature
  std::vector<double> rel_value;          ///< relation values
  std::vector<double> rel_probability;    ///< probabilities of relations
  unsigned groundTruth;                   ///< 0=false / 1=true
  unsigned prediction;                    ///< 0=false / 1=true
};


class CalculateRelations
{
private:
  
  bool isInitialized;                     ///< initialize flag
  KinectCore *kcore;                      ///< Kinect core
  
  std::vector<Relation> allRelations;     ///< all relations between features
  std::vector<Relation> ppRelations;      ///< patch-patch relations              /// TODO Inkonsistent, weil AddPrediction und AddProbability nur bei allRelations eintragen.
  std::vector<Relation> plRelations;      ///< patch-line relations               /// TODO Inkonsistent, weil AddPrediction und AddProbability nur bei allRelations eintragen.
  
  void CalcPatchPatchRelations(std::vector<Relation> &rel);
//   void GetPatchLineRelations(std::vector<Relation> &rel);

public:
  
  CalculateRelations();
  ~CalculateRelations() {}
  
  void Initialize(KinectCore *k);
  
  void CalcRelations(std::vector<Relation> &rel);
  void CalcAllRelations(std::vector<Relation> &rel);
  
  void AddPrediction(unsigned id, double prediction) {allRelations[id].prediction = prediction;}
  void AddProbability(unsigned id, std::vector<double> probability) {allRelations[id].rel_probability = probability;}
  void GetRelations(std::vector<Relation> &rel) {rel = allRelations;}

  void PrintResults();
  double CheckAccuracy();
};

}


#endif