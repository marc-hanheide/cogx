/**
 * @author Andreas Richtsfeld
 * @date April 2011
 * @brief Just receives point clouds and displays them on the TomGine.
 */

#include <limits.h>
#include <opencv/highgui.h>
#include <cogxmath.h>
#include "PointCloudViewer.h"
#include "StereoCamera.h"
#include "VideoUtils.h"

#define SHOW_IMAGE

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::PointCloudViewer();
  }
}

namespace cast
{

using namespace std;
using namespace cogx;
using namespace cogx::Math;
using namespace PointCloud;


/**
 * @brief Convert points from point cloud server to opencv matrix.
 * @param cloud Point cloud
 * @param colCloud Color values for the point cloud
 */
void PointCloudViewer::Points2Cloud(cv::Mat_<cv::Point3f> &cloud, cv::Mat_<cv::Point3f> &colCloud)
{
  cloud = cv::Mat_<cv::Point3f>(1, points.size());
  colCloud = cv::Mat_<cv::Point3f>(1, points.size());    

  for(unsigned i = 0; i<points.size(); i++)
  {
    cv::Point3f p, cp;
    p.x = (float) points[i].p.x;
    p.y = (float) points[i].p.y;
    p.z = (float) points[i].p.z;
    cp.x = (uchar) points[i].c.b;	// change rgb to bgr
    cp.y = (uchar) points[i].c.g;
    cp.z = (uchar) points[i].c.r;

    cloud.at<cv::Point3f>(0, i) = p;
    colCloud.at<cv::Point3f>(0, i) = cp;
  }
}


/**
 * @brief Configure
 * @param _config Configuration
 */
void PointCloudViewer::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  // first let the base classes configure themselves
  configureServerCommunication(_config);

  // startup window size
  int winWidth = 640, winHeight = 480;
  // virtual camera has focal length of winwidt, resulting in roughly 53 degree horizontal field of view
  cv::Mat intrinsic = (cv::Mat_<double>(3,3) << winWidth,0,winWidth/2, 0,winWidth,winHeight/2, 0,0,1);
  cv::Mat R = (cv::Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
  cv::Mat t = (cv::Mat_<double>(3,1) << 0,0,0);
  cv::Vec3d rotCenter(0,0,0.4);

  // Initialize 3D render engine 
  tgRenderer = new TGThread::TomGineThread(winWidth, winHeight);
  tgRenderer->SetParameter(intrinsic);
  tgRenderer->SetCamera(R, t, rotCenter);
  tgRenderer->SetCoordinateFrame(0.5);
}


/**
 * @brief start component
 */
void PointCloudViewer::start()
{
  startPCCServerCommunication(*this);
}


/**
 * @brief runComponent
 */
void PointCloudViewer::runComponent()
{
  static int stereoWidth = 320;   // width of stereo point cloud

#ifdef SHOW_IMAGE
  cvNamedWindow("PointCloudViewer Display", CV_WINDOW_AUTOSIZE);
  Video::Image img;
  IplImage *iplImg;
#endif

  while(isRunning())
  {
    points.resize(0);
    getPoints(true, stereoWidth, points);
    cv::Mat_<cv::Point3f> cloud;
    cv::Mat_<cv::Point3f> colCloud;
    Points2Cloud(cloud, colCloud);
    tgRenderer->Clear();
    tgRenderer->SetPointCloud(cloud, colCloud);

#ifdef SHOW_IMAGE
    getRectImage(0, 320, img);
//     getDisparityImage(640, img);
    iplImg = convertImageToIpl(img);
    cvShowImage("PointCloudViewer Display", iplImg);
    cvWaitKey(10);
#endif
  }
#ifdef SHOW_IMAGE
  cvDestroyWindow("PointCloudViewer Display");
#endif
}

}

