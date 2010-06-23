#ifndef MATCHER_TZWIWD1K
#define MATCHER_TZWIWD1K

#include "Features.h"

namespace cogx { namespace vision {

class CSiftMatcherCudaSift: public CSiftMatcher
{
public:
   virtual void matchSiftDescriptors(TSiftVector& a, TSiftVector& b,
         TFeatureMatchVector& matches);
   virtual void matchSiftDescriptors(TSiftVector& a, std::vector<TSiftVector*>& bb,
         std::vector<TFeatureMatchVector*>& matches);
};

}} // namespace
#endif /* end of include guard: MATCHER_TZWIWD1K */
