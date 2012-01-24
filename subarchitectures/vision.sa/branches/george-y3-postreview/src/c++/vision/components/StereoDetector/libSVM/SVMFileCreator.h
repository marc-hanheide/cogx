/**
 * @file SVMFileCreator.h
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Store relations between features in a file for svm-training.
 */

#ifndef Z_SVM_FILE_CREATOR_H
#define Z_SVM_FILE_CREATOR_H

#include "KinectCore.h"
#include "CalculateRelations.h"

#include "v4r/PCLAddOns/PlanePopout.hh"
#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/PCLAddOns/PCLFunctions.h"


namespace Z
{

/**
 * @brief Class SVMFileCreator: 
 */
class SVMFileCreator
{
public:
  
private:
  KinectCore *kcore;
  CalculateRelations *relations;

  double cam_fx, cam_fy, cam_cx, cam_cy;

public:
  SVMFileCreator();
  ~SVMFileCreator();
  
  void Process(KinectCore *kc);

  void WriteResults2File(std::vector<Relation> &rel);
};

}

#endif

