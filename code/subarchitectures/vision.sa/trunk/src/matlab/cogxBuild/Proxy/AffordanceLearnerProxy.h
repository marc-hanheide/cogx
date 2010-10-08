// NOTE: Don't reference any CAST structues in this Matlab proxy!
// Otherwise the proxy needs to be rebuilt on every CAST change and
// you need Matlab for that!
// #include <VisionData.hpp>

#ifndef AFFORDANCELEARNERPROXY_9ZWMYR3K
#define AFFORDANCELEARNERPROXY_9ZWMYR3K

namespace matlab {

extern void AL_get_affordance_features(
      const VisionData::ProtoObject &Object,
      void **something_comes_out
      );

} // namespace

#endif /* end of include guard: AFFORDANCELEARNERPROXY_9ZWMYR3K */
