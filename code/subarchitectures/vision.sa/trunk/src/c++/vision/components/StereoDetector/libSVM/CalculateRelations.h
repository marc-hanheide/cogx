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
#include "Segment3D.h"

#include "v4r/PCLAddOns/PlanePopout.hh"
#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/PCLAddOns/PCLFunctions.h"

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
  bool firstCall;                         ///< True for first call

  /// Minimum distances for each patch to each segment (patch/segment)
  std::vector< std::vector<double> > min_distances;                     /// TODO antiquated?
  /// Distances of each edgel to the patches (patch/segement/edgel)
  std::vector< std::vector< std::vector<double> > > distances;          /// TODO antiquated?
  
  bool CalculateSegmentRelations(Patch3D *p0,             ///< Calculate relations between patches, caused by segments
                                 Patch3D *p1,
                                 std::vector<double> &params);

  bool CalculatePPColorRelation(Patch3D *p0,             ///< Calculate color relation between patches
                                Patch3D *p1,
                                std::vector<double> &params);

  bool CalculatePPRelation(Patch3D *p0, 
                           Patch3D *p1, 
                           std::vector<double> &params);

  bool PLLineInPlaneROI(Patch3D *p, Line3D *l);
  double PLProximity(Patch3D *p, Line3D *l);
  double PLParallelity(Patch3D *p, Line3D *l);
  
  double cam_fx, cam_fy, cam_cx, cam_cy;  ///< Internal camera parameters

  // TODO for debugging
  std::vector<cv::Vec4f> pxlsToDraw;
  
public:
  
  CalculateRelations();
  ~CalculateRelations() {}
  
  void Reset();
  void Initialize(KinectCore *k, double fx, double fy, double cx, double cy);
  
  void CalcSVMRelations(std::vector<Relation> &rel);    ///< Relations for SVM learning
  void CalcTestRelations(std::vector<Relation> &rel);   ///< Relations for testing
  void CalcAllRelations(std::vector<Relation> &rel);    ///< Relations for prediction
  
  void AddPrediction(unsigned id, double prediction) {relations[id].prediction = prediction;}
  void AddProbability(unsigned id, std::vector<double> probability) {relations[id].rel_probability = probability;}
  void getRelations(std::vector<Relation> &rel) {rel = relations;}

  void ConstrainRelations();

  void PrintResults();
  void PrintRelations();
  double CheckAccuracy();
  
  // TODO This is for debugging
  void GetPixelsToDraw(std::vector<cv::Vec4f> &pts);
};

}


#endif
