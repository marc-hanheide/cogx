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
// #include "LearnPropBase.h"

#include "v4r/PCLAddOns/PlanePopout.hh"
#include "v4r/PCLAddOns/utils/PCLUtils.h"
#include "v4r/PCLAddOns/functions/PCLFunctions.h"

// #include "TomGineThread.hh"

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
  pclA::PlanePopout *planePopout;
  CalculateRelations *relations;


public:
  SVMFileCreator();
  ~SVMFileCreator();
  
  void Process(pclA::PlanePopout *pp, KinectCore *kc, TGThread::TomGineThread *tgR);

  void WriteResults2File(std::vector<Relation> &rel);
//   void ReadResultsFromFile();
};

}

#endif

