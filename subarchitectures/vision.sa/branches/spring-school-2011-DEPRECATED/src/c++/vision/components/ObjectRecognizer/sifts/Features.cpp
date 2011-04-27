
#include "Features.h"

namespace cogx { namespace vision {

int CFeatureMatch::cmpDistance(CFeatureMatch* pa, CFeatureMatch* pb)
{
   if (pa->distance < pb->distance) return -1;
   if (pa->distance > pb->distance) return 1;
   return 0;
}

bool CFeatureMatch::isLowerDistance(CFeatureMatch a, CFeatureMatch b)
{
   if (a.distance < b.distance) return true;
   return false;
}

}} // namespace
