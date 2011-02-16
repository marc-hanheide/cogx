#ifndef STEREO_ICE
#define STEREO_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>
#include <Video.ice>
#include <VisionData.ice>

module Stereo {

  /**
   * A stereo server, serving 3D point clouds from a pair of calibrated stereo
   * cameras.
   */
  interface StereoInterface {
    void getPoints(bool transformToGlobal, int imgWidth, out VisionData::SurfacePointSeq points);
    void getCompletePoints(bool transformToGlobal, int imgWidth, out VisionData::SurfacePointSeq points);
    void getRectImage(int side, int imgWidth, out Video::Image img);
    void getDisparityImage(int imgWidth, out Video::Image img);
  };
};

#endif

