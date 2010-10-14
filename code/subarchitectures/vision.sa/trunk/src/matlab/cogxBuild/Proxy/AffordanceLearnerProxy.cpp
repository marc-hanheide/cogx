/* vim:set fileencoding=utf-8 sw=3 ts=8 et:vim */
/** 
 * @brief Proxy for the AffordanceLearner component.
 *
 * @author Marko Mahnič
 */
#include <stdio.h>
#include <stdarg.h>
#include <string>
#include <cv.h>
#include <highgui.h>
#include <VisionData.hpp>
#include "../../../c++/vision/VisionUtils.h"

#include "MatlabHelper.h"
#include "AffordanceLearnerProxy.h"
#include "libVisualLearnerCtf.h"
#include "Globals.h"
#include "Conversion.h"

using namespace std;
using namespace VisionData;

namespace matlab {

void _get_affordance_features(const VisionData::ProtoObject &Object, mwArray& features)
{
   CheckInit();

   mwArray image, mask, pts3d, patches;
   protoObjectToMwArray(Object, image, mask, pts3d);
   protoObjectToMwArray_Patches(Object, patches);

   mwArray dims = patches.GetDimensions();
   double dim0 = dims.Get(mwSize(1), 1);
   if (dim0 < 1 || (double)patches.Get(mwSize(2), 1, 1) < 1) {
      mwSize dimensions[1] = {0};
      features = mwArray(1, dimensions, mxDOUBLE_CLASS, mxREAL);
      return;
   }

   cogxAffordanceLearner_getFeatures(1, features, image, mask, pts3d, patches);
}

void AL_get_affordance_features(const VisionData::ProtoObject &Object, std::vector<double> &features)
{
   features.clear();

   mwArray mwfeatures;
   _get_affordance_features(Object, mwfeatures);

   mwArray dims = mwfeatures.GetDimensions();
   double dim0 = dims.Get(mwSize(1), 1);
   for (int i = 0; i < dim0; i++) {
      double ftr = mwfeatures.Get(mwSize(1), i+1);
      features.push_back(ftr);
   }
}

void AL_affordance_recognise(const VisionData::ProtoObject &Object, std::string& outAffordance)
{
   CheckInit();

   mwArray affordance, features;
   _get_affordance_features(Object, features);

   cogxAffordanceLearner_recognise(1, affordance, features);

   mwArray dims = affordance.GetDimensions();
   double dim0 = dims.Get(mwSize(1), 1);

   if (dim0 > 0) {
      //mwString mws = affordance.Get(mwSize(1), 1).ToString();
      mwString mws = affordance.ToString();
      outAffordance = string((const char*)mws);
   }
   else outAffordance = "";
}

}
