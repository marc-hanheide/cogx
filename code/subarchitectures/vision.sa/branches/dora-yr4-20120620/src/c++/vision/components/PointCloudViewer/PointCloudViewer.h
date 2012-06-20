/**
 * @author Andreas Richtsfeld
 * @date April 2011
 * @brief Just receives point clouds and displays them on the TomGine.
 */

#ifndef STEREO_POINT_CLOUD_H
#define STEREO_POINT_CLOUD_H

#include <cast/architecture/ManagedComponent.hpp>
#include <PointCloudClient.h>
#include "TomGineWraper/TomGineThread.hh"
#include <VideoClient.h>


namespace cast
{

class PointCloudViewer : public PointCloudClient,
                         public ManagedComponent
{
private:
  std::vector<PointCloud::SurfacePoint> points;     ///< 3D point vector
  TGThread::TomGineThread *tgRenderer;              ///< 3D render engine
  
  void Points2Cloud(cv::Mat_<cv::Point3f> &cloud, cv::Mat_<cv::Point3f> &colCloud);

protected:

  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

public:
  PointCloudViewer() {tgRenderer = 0;}
  virtual ~PointCloudViewer() {delete tgRenderer;}
};

}

#endif



