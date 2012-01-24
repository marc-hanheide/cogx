#ifndef POINT_CLOUD_ICE
#define POINT_CLOUD_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>
#include <Video.ice>
#include <Laser.ice>
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

  sequence<int> IntSeq;

  /**
   * A point cloud server.
   */
  interface PointCloudInterface {
    void getPoints(bool transformToGlobal, int imgWidth, out PointCloud::SurfacePointSeq points);
    void getCompletePoints(bool transformToGlobal, int imgWidth, out PointCloud::SurfacePointSeq points);
    void getRectImage(int side, int imgWidth, out Video::Image img);
    void getDepthMap(out cast::cdl::CASTTime time, out IntSeq data);
    void getDisparityImage(int imgWidth, out Video::Image img);
    void getRangePoints(out Laser::Scan2d KRdata); /* kinect range data returns back like laser sensor */
    bool getCameraParameters(int side, out Video::CameraParameters camPars);
    bool isPointInViewCone(cogx::Math::Vector3 point);

    // Check if the point is visible by all cameras that generate the point cloud.
    bool isPointVisible(cogx::Math::Vector3 point);
  };

};

#endif

