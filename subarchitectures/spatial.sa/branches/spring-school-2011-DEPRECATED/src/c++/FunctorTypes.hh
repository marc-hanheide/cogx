#ifndef FUNCTORTYPES_H
#define FUNCTORTYPES_H

namespace SpatialGridMap {
    /*
    This defines how bloxels are modified before the functor is applied.

    Types available:
    Read - Iterates over bloxel reading each one, READ ONLY!
    (these functors should not take a reference, writing to data will cause wierd results)
    Replace - Replace affected area with one large bloxel and write to it
    Iterate - Iterates over bloxel writing new values to them,
    creates new bloxels at edges
    Split - Splits bloxel into many smaller and iterates over them,
    alowing for fine scaled changes
     */

    enum FunctorType { Read, Replace, Iterate, Split };

}; 

#endif
