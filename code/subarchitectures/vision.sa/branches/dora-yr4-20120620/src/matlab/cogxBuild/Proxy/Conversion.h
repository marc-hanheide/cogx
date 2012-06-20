#ifndef CONVERSION_DU3IV3CF
#define CONVERSION_DU3IV3CF
/** 
 * @author Marko Mahnič
 */

namespace matlab {

extern void protoObjectToMwArray(
    const VisionData::ProtoObject &Object,
    mwArray &image, mwArray &mask, mwArray &points3d);

extern void protoObjectToMwArray_Patches(
    const VisionData::ProtoObject &Object,
    mwArray &patches);

} // namespace

#endif /* end of include guard: CONVERSION_DU3IV3CF */
