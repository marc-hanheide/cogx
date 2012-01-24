/* vim:set fileencoding=utf-8 sw=3 ts=8 et:vim */
/** 
 * @brief Proxy for the AffordanceLearner component.
 *
 * @author Marko Mahnič
 */
#include <stdio.h>
#include <stdarg.h>
#include <string>
#include <VisionData.hpp>
#include "../AffordanceLearnerProxy.h"

using namespace std;
using namespace VisionData;

namespace matlab {

void AL_get_affordance_features(const VisionData::ProtoObject &Object, std::vector<double> &features)
{
   features.clear();
}

void AL_affordance_recognise(const VisionData::ProtoObject &Object, std::string& outAffordance)
{
   outAffordance = "slide";
}

}
