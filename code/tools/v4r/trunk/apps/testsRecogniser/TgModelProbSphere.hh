/**
 * $Id$
 */

#ifndef P_TG_MODEL_PROB_SPHERE_HH
#define P_TG_MODEL_PROB_SPHERE_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <set>
#include <pthread.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <v4r/PMath/PMatrix.hh>
#include <v4r/PGeometry/Pose.hh>
#include <v4r/TomGine/tgModel.h>
#include <v4r/CModelRecogniser/SphereHistogram.hh>


namespace TomGine 
{


class TgModelProbSphere : public tgModel, public P::SphereHistogram
{
public:
  pthread_mutex_t mut;

  TgModelProbSphere();
  ~TgModelProbSphere();

  inline void Lock(){pthread_mutex_lock(&mut);}
  inline void Unlock(){pthread_mutex_unlock(&mut);}

  virtual void Draw();
};





/*************************** INLINE METHODES **************************/


} //--END--

#endif

