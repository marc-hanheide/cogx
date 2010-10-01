// TODO: Don't reference any CAST structues in this Matlab proxy!
// Otherwise the proxy needs to be rebuilt on every CAST change and
// you need Matlab for that!
// Maybe MatlabData.ice could be allowed.
// #include <VisionData.hpp>

#include "Enumerator.h"
#include <vector>

namespace matlab {
extern void VL_recognise_attributes(
      const VisionData::ProtoObject &Object,
      std::vector<std::string> &labels,
      std::vector<int> &labelConcepts,
      std::vector<double> &probs,
      std::vector<double> &gains);

extern void VL_update_model(
      VisionData::ProtoObject &Object,
      std::vector<std::string> &labels,
      std::vector<double> &weights);

extern void VL_LoadAvModels(const char* filename);

extern void VL_setEnumeration(const cogx::CTypeEnumerator& typeEnum);
extern void VL_setClfStartConfig(const std::string& absConfigPath);

} // namespace
