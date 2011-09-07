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
#include "Line3D.h"

#include "v4r/PCLAddOns/PlanePopout.hh"
#include "v4r/PCLAddOns/utils/PCLUtils.h"
#include "v4r/PCLAddOns/functions/PCLFunctions.h"

namespace Z
{
 
struct Relation
{
  unsigned type;                          ///< Type of relation (Patch-Patch = 1 / Patch-Line = 2 / Line-Line = 3 / ...)
  unsigned id_0;                          ///< id of first feature
  unsigned id_1;                          ///< id of second feature
  std::vector<double> rel_value;          ///< relation values (feature vector)
  std::vector<double> rel_probability;    ///< probabilities of correct prediction
  unsigned groundTruth;                   ///< 0=false / 1=true
  unsigned prediction;                    ///< 0=false / 1=true
  bool remove;                            ///< delete flag
};


class CalculateRelations
{
private:
  KinectCore *kcore;                      ///< Kinect core
  
  std::vector<Relation> relations;        ///< all relations between features
  
  void CalcSVMPatchPatchRelations();      ///< Calculate relations between patches
  void CalcSVMPatchLineRelations();       ///< Calculate relations between patches and lines
  void CalcSVMLineLineRelations();        ///< Calculate relations between lines

  bool PLLineInPlaneROI(Patch3D *p, Line3D *l);
  double PLProximity(Patch3D *p, Line3D *l);
  double PLParallelity(Patch3D *p, Line3D *l);
  
  double cam_fx, cam_fy, cam_cx, cam_cy;  ///< Internal camera parameters

public:
  
  CalculateRelations();
  ~CalculateRelations() {}
  
  void Initialize(KinectCore *k, double fx, double fy, double cx, double cy);
  
  void CalcSVMRelations(std::vector<Relation> &rel);
  void CalcTestRelations(std::vector<Relation> &rel);
  void CalcAllRelations(std::vector<Relation> &rel);
  
  void AddPrediction(unsigned id, double prediction) {relations[id].prediction = prediction;}
  void AddProbability(unsigned id, std::vector<double> probability) {relations[id].rel_probability = probability;}
  void GetRelations(std::vector<Relation> &rel) {rel = relations;}

  void ConstrainRelations();

  void PrintResults();
  void PrintRelations();
  double CheckAccuracy();
};

}


#endif