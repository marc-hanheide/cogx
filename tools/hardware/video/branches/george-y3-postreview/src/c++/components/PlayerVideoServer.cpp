#include "PlayerVideoServer.h"
#include <ConvertImage.h>
#include <cast/core/CASTUtils.hpp>


/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::PlayerVideoServer();
  }
}

using namespace std;

namespace cast
{

Video::CIplImageCache CCameraInfo::m_imageCache;

static const int FRAMERATE_DEFAULT = 100; // 10 fps

PlayerVideoServer::PlayerVideoServer()
{
  m_frameDurationMs = FRAMERATE_DEFAULT;
  m_nextFrameTime = 0;
  m_pPlayer = 0;
  m_playerHost = "localhost";
  m_playerPort = 6665;
}

PlayerVideoServer::~PlayerVideoServer()
{
  if (m_pPlayer) {
    delete m_pPlayer;
    m_pPlayer = 0;
  }
  for (int i = 0; i < m_cameras.size(); i++) {
    if (m_cameras[i]) delete m_cameras[i];
    m_cameras[i] = NULL;
  }
  m_cameras.clear();
}

void PlayerVideoServer::start()
{
  VideoServer::start();
  m_nextFrameTime = m_timer.elapsed();
}

void PlayerVideoServer::configure(const map<string,string> & _config)
  throw(runtime_error)
{
  VideoServer::configure(_config);

  vector<int> dev_nums;
  map<string,string>::const_iterator it;

  if((it = _config.find("--devnums")) != _config.end())
  {
    istringstream str(it->second);
    int dev;
    while(str >> dev)
      dev_nums.push_back(dev);
  }
  else
  {
    // assume 0 as default device
    dev_nums.push_back(0);
  }

  if ((it = _config.find("--player-host")) != _config.end()) {
    m_playerHost = it->second;
  }

  if ((it = _config.find("--player-port")) != _config.end()) {
    istringstream str(it->second);
    str >> m_playerPort;
  }

  if((it = _config.find("--framerate_ms")) != _config.end())
  {
    istringstream str(it->second);
    str >> m_frameDurationMs;
    if (m_frameDurationMs <= 0)
      m_frameDurationMs = FRAMERATE_DEFAULT;
    if (m_frameDurationMs > 10000)
      m_frameDurationMs = 10000;
  }

  if((it = _config.find("--framerate_fps")) != _config.end())
  {
    istringstream str(it->second);
    double fps;
    str >> fps;
    if (fps < 0.1) fps = 0.1;
    if (fps > 100) fps = 100;
    m_frameDurationMs = (long) (1000.0 / fps + 0.5);
  }

#if 0
  // The image size is controlled in a gazebo world file
  if((it = _config.find("--imgsize")) != _config.end())
  {
    istringstream str(it->second);
    str >> captureSize.width >> captureSize.height;
  }
#endif

  init(dev_nums);
}

const std::string PlayerVideoServer::getServerName()
{
  const std::string str("PlayerVideoServer");
  return str;
}


void PlayerVideoServer::init(const vector<int> &dev_nums)
  throw(runtime_error)
{
  if (dev_nums.size() < 1)
    throw runtime_error(exceptionMessage(__HERE__,
          "must specify at least one camera"));

  if(dev_nums.size() != camIds.size())
    throw runtime_error(exceptionMessage(__HERE__,
          "number of devices %d does not match number of camera IDs %d",
          (int)dev_nums.size(), (int)camIds.size()));

  m_pPlayer = new PlayerCc::PlayerClient(m_playerHost, m_playerPort);
  if (! m_pPlayer) {
    throw runtime_error(exceptionMessage(__HERE__,
          "failed to connect to Player '%s:%d'",
          m_playerHost.c_str(), m_playerPort));
  }

  // We will pull the data so we can control the framerate. Should we use PUSH instead?
  m_pPlayer->SetDataMode(PLAYER_DATAMODE_PULL);
  m_pPlayer->SetReplaceRule(true, PLAYER_MSGTYPE_DATA, -1, -1);

  // Initialize camera proxies
  for (int i = 0; i < dev_nums.size(); i++) {
    PlayerCc::CameraProxy* pcampx = new PlayerCc::CameraProxy(m_pPlayer, dev_nums[i]);
    if (! pcampx) {
      throw runtime_error(exceptionMessage(__HERE__,
            "failed to create a camera proxy for camera %d",
            dev_nums[i]));
    }
    m_cameras.push_back(new CCameraInfo(dev_nums[i], pcampx));
  }

  // Wait for cameras
  bool all_active = false;
  int  attempts = 100;
  CCameraInfo* pcam;
  while (! all_active && attempts > 0)
  {
    m_pPlayer->Read();
    attempts--;
    all_active = true;
    for (int i = 0; i < m_cameras.size(); i++) {
      pcam = m_cameras[i];
      if (pcam->m_width > 0) continue;
      all_active = false;
      pcam->readParams();
      log("Camera %d: %dx%d", i, pcam->m_width, pcam->m_height);
    }
  }
  if (! all_active) {
    throw runtime_error(exceptionMessage(__HERE__,
          "failed to connect to some player cameras"));
  }
}

void PlayerVideoServer::grabFramesInternal()
{
  m_pPlayer->Read(); // A blocking read

  timeStats.increment();
  cdl::CASTTime time = getCASTTime();
  for(int i = 0; i < getNumCameras(); i++) {
    m_cameras[i]->m_bFrameCached = false;
    m_cameras[i]->m_grabTime = time;
  }
}

int PlayerVideoServer::getFramerateMilliSeconds()
{
  // frames per second --> milliseconds per frame
  if(timeStats.getRate() > 0.)
    return (int)(1000.0 / timeStats.getRate());

  return 0;
}

void PlayerVideoServer::grabFrames()
{
  grabFramesInternal();
}

void PlayerVideoServer::retrieveFrameInternal(int camIdx, int width, int height,
    Video::Image &frame)
{
  if (camIdx >= m_cameras.size())
    return;
  CCameraInfo* pcam = m_cameras[camIdx];
  if (! pcam) return;

  char id[32];

  // NOTE: No locking is implemented because runComponent already locks everything.

  if (! pcam->m_bFrameCached) {
    // TODO: write-lock retrievedImages
    pcam->retrieveFrame();
    // TODO: unlock retrievedImages
  }

  frame.time = pcam->m_grabTime;
  frame.camId = camIds[camIdx];
  frame.camPars = camPars[camIdx];

  // no size given, use native size
  if((width == 0 || height == 0) || (width == pcam->m_width && height == pcam->m_height)) {
    // TODO: read-lock retrievedImages
    Video::convertImageFromIpl(pcam->m_pRetrievedFrame, frame);
    // TODO: unlock retrievedImages
    // adjust to native size
    // (note that calibration image size need not be the same as currently set
    // native capture size)
    changeImageSize(frame.camPars, pcam->m_width, pcam->m_height);
  }
  else {
    sprintf(id, "img8u3_%d_%d", width, height);
    // use image cache to avoid allocate/deallocating all the time
    // XXX: write-lock cache ?
    IplImage *tmp = m_imageCache.getImage(id, width, height, IPL_DEPTH_8U, 3);
    // TODO: read-lock retrievedImages
    cvResize(pcam->m_pRetrievedFrame, tmp);
    // TODO: unlock retrievedImages
    Video::convertImageFromIpl(tmp, frame);

    // adjust to scaled image size
    changeImageSize(frame.camPars, width, height);
  }
}

void PlayerVideoServer::retrieveFrames(const std::vector<int> &camIds,
    int width, int height, std::vector<Video::Image> &frames)
{
  frames.resize(camIds.size());
  for(size_t j = 0; j < camIds.size(); j++)
  {
    size_t i = getCamIndex(camIds[j]);
    retrieveFrameInternal(i, width, height, frames[j]);
  }
}

void PlayerVideoServer::retrieveFrames(int width, int height,
    std::vector<Video::Image> &frames)
{
  frames.resize(getNumCameras());
  for(int i = 0; i < getNumCameras(); i++)
    retrieveFrameInternal(i, width, height, frames[i]);
}

void PlayerVideoServer::retrieveFrame(int camId, int width, int height,
    Video::Image &frame)
{
  size_t i = getCamIndex(camIds[camId]);
  retrieveFrameInternal(i, width, height, frame);
}

void PlayerVideoServer::retrieveHRFrames(std::vector<Video::Image> &frames)
{
  log("PlayerVideoServer::retrieveHRFrames: not yet implemented.\n");
}

void PlayerVideoServer::getImageSize(int &width, int &height)
{
  if (m_cameras.size() > 0)
  {
    CCameraInfo* pcam = m_cameras[0];
    width = pcam->m_width;
    height = pcam->m_height;
    return;
  }
  width = 0;
  height = 0;
}

using namespace Video;

void PlayerVideoServer::runComponent()
{
  vector<Image> frames;
  string myid = getComponentID();
  const int reportDelay = CMilliTimer::seconds(30);
  long long now = m_timer.elapsed();
  long long tmNextReport = now + CMilliTimer::seconds(5);
  long long tmLastReport = now;
  long long tmNextFrame = now + m_frameDurationMs;
  int sendCount = 0;
  while(isRunning())
  {
    now = m_timer.elapsed();
    if (now < tmNextFrame) {
      long long dt = tmNextFrame - now;
      if (dt > 1) dt /= 2;
      sleepComponent(dt);
      continue;
    }
    tmNextFrame += m_frameDurationMs;
    if (tmNextFrame < now) // Frame was dropped!
    {
      if (now - tmNextFrame > m_frameDurationMs * 5)
        tmNextFrame = now + m_frameDurationMs;
      else
        while (tmNextFrame < now)
          tmNextFrame += m_frameDurationMs;
    }

    // TODO: If I could have this lock after grabFrames() I could avoid the
    // stupid sleep.
    lockComponent();
    grabFrames();

    // TODO: grabFrames may take less than the duration of a frame

    for(size_t i = 0; i < imageReceivers.size(); i++)
    {
      retrieveFrames(imageReceivers[i].camIds, imageReceivers[i].imgWidth, imageReceivers[i].imgHeight, frames);
      if(swapRB)
        for(size_t i = 0; i < frames.size(); i++)
          SwapRedBlueChannel(frames[i]);
      imageReceivers[i].videoClient->receiveImages2(myid, frames);
    }
    unlockComponent();
    // HACK: to let getImages() have chance to lockComponent()
    sleepComponent(10);

    ++sendCount;
    now = m_timer.elapsed();
    if(now > tmNextReport || now < tmLastReport /* timer was reset */)
    {
      int fr = getFramerateMilliSeconds();
      debug("grabbing with %d ms per frame (%.2f frames per second)",
          fr, (fr > 0. ? 1000./fr : 0.));
      if(fr > 0.) realFps = 1000./fr;
      else realFps = 0.;

      double dfr = now - tmLastReport;
      if (dfr > 0 && sendCount > 0)
      {
        debug("sending with %.0f ms per frame (%.2f frames per second)",
            dfr / sendCount, sendCount / dfr * 1000);
      }

      tmLastReport = now;
      tmNextReport = now + reportDelay;
      sendCount = 0;
    }
  }
}


}; // namespace
/* vim:set sw=2 sts=4 ts=8 et:vim */
