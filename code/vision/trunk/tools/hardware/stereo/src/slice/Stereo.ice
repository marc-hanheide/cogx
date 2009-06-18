#ifndef STEREO_ICE
#define STEREO_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>

module Stereo {

  sequence<cogx::Math::Vector3> Vector3Seq;

  /**
   * A stereo server, serving 3D point clouds from a pair of calibrated stereo
   * cameras.
   */
  interface StereoInterface {
    void getPoints(out Vector3Seq points);
  };
};

#endif

