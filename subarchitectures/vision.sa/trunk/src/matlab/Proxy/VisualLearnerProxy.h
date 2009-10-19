// TODO: Don't reference any CAST structues in this Matlab proxy!
// Otherwise the proxy needs to be rebuilt on every CAST change and
// you need Matlab for that!
// Maybe MatlabData.ice could be allowed.
// #include <VisionData.hpp>

extern void VL_recognise_attributes(VisionData::AttrObject &Attrs, VisionData::ProtoObject &Object);
extern void VL_LoadAvModels(const char* filename);
