// TODO: Don't reference any CAST structues in this Matlab proxy!
// Otherwise the proxy needs to be rebuilt on every CAST change and
// you need Matlab for that!
// Maybe MatlabData.ice could be allowed.
// #include <VisionData.hpp>

#include <vector>

extern void VL_recognise_attributes(
      const VisionData::ProtoObject &Object,
      std::vector<int> &labels,
      std::vector<double> &distribution);

extern void VL_update_model(
      VisionData::ProtoObject &Object,
      std::vector<int> &labels,
      std::vector<double> &distribution);

extern void VL_LoadAvModels(const char* filename);
