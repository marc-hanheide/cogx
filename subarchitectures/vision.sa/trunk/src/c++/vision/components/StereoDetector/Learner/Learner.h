/**
 * @file Learner.h
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Learn about image features
 */

#ifndef Z_LEARNER_H
#define Z_LEARNER_H

// #include <opencv/cxcore.h>
// #include <opencv/cv.h>
// #include <stdexcept>

#include "KinectCore.h"

namespace Z
{

/**
 * @brief Class Learner: 
 */
class Learner
{
public:
  
private:
  KinectCore *kcore;
  
//   void LearnLJunctionsBetweenLines();
//   void LearnParallelities();
//   void LearnCollinearities();
//   void LearnClosures();
//   void LearnRectangles();
  void LearnClosenessBetweenPatches();
  void LearnColorSimilarityBetweenPatches();
//   void LearnCoplanaritiesBetweenPatches();
//   void LearnCoplanaritiesBetweenClosuresAndPatches();
//   void LearnConcavitiesConvexitiesBetweenPatches();
//   void LearnCommonMotion();
  

public:
  Learner();
  ~Learner();
  
  void Process(KinectCore *kc);
  
};

}

#endif

