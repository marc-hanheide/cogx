#ifndef STEREO_ICE
#define STEREO_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>
#include <VisionData.ice>

module Stereo {

  /**
   * A stereo server, serving 3D point clouds from a pair of calibrated stereo
   * cameras.
   */
  interface StereoInterface {
    void getPoints(out VisionData::SurfacePointSeq points);
    void getPointsInSOI(VisionData::SOI soi, out VisionData::SurfacePointSeq points);
  };
};

#endif

