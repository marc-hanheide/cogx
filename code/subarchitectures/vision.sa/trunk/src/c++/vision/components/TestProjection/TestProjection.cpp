#include <VideoUtils.h>
#include "TestProjection.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::TestProjection();
  }
}

namespace cast
{

using namespace std;
using namespace cogx;
using namespace cogx::Math;
using namespace PointCloud;

/**
 * @brief Configure
 * @param _config Configuration
 */
void TestProjection::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  // first let the base classes configure themselves
  configureServerCommunication(_config);

  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }
  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream str(it->second);
    str >> camId;
  }
}

/**
 * @brief start component
 */
void TestProjection::start()
{
  startPCCServerCommunication(*this);
 
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);
}

/**
 * @brief runComponent
 */
void TestProjection::runComponent()
{
  static int stereoWidth = 640;   // width of stereo point cloud

  cvNamedWindow("TestProjection", CV_WINDOW_AUTOSIZE);
  Video::Image img;
  IplImage *iplImg;
  vector<PointCloud::SurfacePoint> points;

  while(isRunning())
  {
    points.resize(0);
    getPoints(true, stereoWidth, points);

    videoServer->getImage(camId, img);
    iplImg = convertImageToIpl(img);
    for(size_t i = 0; i < points.size(); i++)
    {
      cogx::Math::Vector2 p = projectPoint(img.camPars, points[i].p);
      cvRectangle(iplImg, cvPoint(p.x - 0, p.y - 0), cvPoint(p.x + 0, p.y + 0),
          CV_RGB(points[i].c.r, points[i].c.g, points[i].c.b));
    }
    cvShowImage("TestProjection", iplImg);
    cvWaitKey(10);
    cvReleaseImage(&iplImg);
  }
  cvDestroyWindow("TestProjection");
}

}

