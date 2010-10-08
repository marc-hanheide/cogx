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

void AL_get_affordance_features(const VisionData::ProtoObject &Object, void **something_comes_out)
{
   CheckInit();

   mwArray features, image, mask, pts3d;
   protoObjectToMwArray(Object, image, mask, pts3d);

   cogxAffordanceLearner_getFeatures(1, features, image, mask, pts3d);
}

}
