#ifndef POINT_CLOUD_ICE
#define POINT_CLOUD_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>
#include <Video.ice>

module PointCloud {

  /**
   * A 3D point with a color, as e.g. returned by point cloud server
   * @author Michael Zillich
   */
  struct SurfacePoint {
    cogx::Math::Vector3 p;
    cogx::Math::ColorRGB c;
  };
  sequence<SurfacePoint> SurfacePointSeq;


  /**
   * A point cloud server.
   */
  interface PointCloudInterface {
    void getPoints(bool transformToGlobal, int imgWidth, out PointCloud::SurfacePointSeq points);
    void getCompletePoints(bool transformToGlobal, int imgWidth, out PointCloud::SurfacePointSeq points);
    void getRectImage(int side, int imgWidth, out Video::Image img);
    void getDisparityImage(int imgWidth, out Video::Image img);
  };

};

#endif

