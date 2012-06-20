/**
 * @author Marko Mahnič
 * @created March 2011
 *
 */
#include "PcGrabber.h"
#include "StringFmt.h"
#include "VideoGrabber.h"

#include <castutils/Timers.hpp>

#include <sstream>
#include <fstream>

namespace cogxgrabber
{

#ifdef FEAT_VIDEOGRABBER_POINTCLOUD
CPcGrabClient::CPcGrabClient():
  cast::PointCloudClient()
{
  mName = "Point Cloud Grabber " + _str_(mId);
  mbGrabPoints = true;
  mbGrabDepth = false;
  mbGrabRectImage = false;
}

void CPcGrabClient::grab(std::vector<CGrabbedItemPtr>& items)
{
  if (mbGrabPoints) {
    CGrabbedPcPoints *pPoints = new CGrabbedPcPoints();
    getPoints(false, 320, pPoints->mPoints);
    //printf(" ***** getPoints: %ld\n", pPoints->mPoints.size());
    mLastPoints = CGrabbedItemPtr(pPoints);
    items.push_back(mLastPoints);
  }
  if (mbGrabDepth) {
  }
  if (mbGrabRectImage) {
    CGrabbedImage *pImage = new CGrabbedImage();
    getRectImage(0, 640, pImage->mImage);
    mLastRectImage = CGrabbedItemPtr(pImage);
    items.push_back(mLastRectImage);
  }
}

void CPcGrabClient::getPreviews(std::vector<CPreview>& previews,
    int width, int height, bool isGrabbing)
{
  if (!isGrabbing) {
    std::vector<CGrabbedItemPtr> items;
    grab(items); // data will be present also in mLastXXX
  }

  CGrabbedImage depthImg;
  if (mbGrabPoints || mbGrabDepth) {
    getRectImage(-1, 320, depthImg.mImage);
  }

  if (mbGrabPoints) {
    std::string cid = "pc-points-" + _str_(mId);
    previews.push_back(CPreview(cid, width, height));
    CPreview& pv = previews.back();
    pv.deviceName = "pc." + _str_(mId) + ".0";
    pv.deviceInfo = "3D points";

    IplImage* pPreview = pv.getImage();
    IplImage *pOrig = cloneVideoImage(depthImg.mImage);
    cvResize(pOrig, pPreview);
    releaseClonedImage(&pOrig);
  }
  if (mbGrabDepth) {
    std::string cid = "pc-depth-" + _str_(mId);
    previews.push_back(CPreview(cid, width, height));
    CPreview& pv = previews.back();
    pv.deviceName = "pc." + _str_(mId) + ".1";
    pv.deviceInfo = "Depth";

    IplImage* pPreview = pv.getImage();
    IplImage *pOrig = cloneVideoImage(depthImg.mImage);
    cvResize(pOrig, pPreview);
    releaseClonedImage(&pOrig);
  }
  if (mbGrabRectImage) {
    std::string cid = "pc-rectimg-" + _str_(mId);
    previews.push_back(CPreview(cid, width, height));
    CPreview& pv = previews.back();
    pv.deviceName = "pc." + _str_(mId) + ".2";
    pv.deviceInfo = "Rectified ";

    IplImage* pPreview = pv.getImage();
    CGrabbedImage *pGrabbed = dynamic_cast<CGrabbedImage*>(mLastRectImage.get());

    if (! pGrabbed) {
      cvZero(pPreview);
      pv.deviceInfo += "???";
    }
    else {
      pv.deviceInfo += _str_(pGrabbed->mImage.width) + "x" + _str_(pGrabbed->mImage.height);
      IplImage *pOrig = cloneVideoImage(pGrabbed->mImage);
      cvResize(pOrig, pPreview);
      releaseClonedImage(&pOrig);
    }
  }
}

#ifdef FEAT_VISUALIZATION
void CPcGrabClient::configExtraV11n(cogx::display::CDisplayClient& display)
{
  std::string devid = "pc." + _str_(mId) + ".2";
  std::ostringstream ss;
  ss <<  "function render()\nend\n"
    << "setCamera('ppo.robot.head', -1.0, 0, 3.0, 1, 0, -1, 0, 0, 1)\n"
    << "setCamera('ppo.robot.front', 4.0, 0, 4.0, -1, 0, -1, 0, 0, 1)\n"
    << "setCamera('ppo.points.top', 0, 0, 4.0, 0, 0, -1, -1, 0, 0)\n";
  display.setLuaGlObject("grab." + devid, "cameras", ss.str());
}

void CPcGrabClient::displayExtra(cogx::display::CDisplayClient& display)
{
  castutils::CMilliTimer tm(true);
  CGrabbedItemPtr points = mLastPoints;
  std::string devid = "pc." + _str_(mId) + ".2";
  CGrabbedPcPoints* pPoints = dynamic_cast<CGrabbedPcPoints*>(points.get());
  if (!pPoints)
    return;
  if (!pPoints->mPoints.size())
    return;

  int pointCnt = 0;
  std::ostringstream str;
  str.unsetf(std::ios::floatfield); // unset floatfield
  str.precision(3); // set the _maximum_ precision
  str << "function render()\n";
  str << "if not showPoints then return end\n";
  str << "glPointSize(2)\nglBegin(GL_POINTS)\n";
  str << "v=glVertex\nc=glColor\n";

#define FCHN(x) (float)x/255.0
  // (Approximately) Limit the number of points sent to the display server
  int next = rand() % 10 + 5;
  for (auto sp : pPoints->mPoints) {
    next--;
    if (next > 0) {
      continue;
    }
    next = rand() % 10 + 5;
    str << "c("
      << FCHN(sp.c.r) << ","
      << FCHN(sp.c.g) << ","
      << FCHN(sp.c.b) << ")\n";
    str << "v("
      << sp.p.x << ","
      << sp.p.y << ","
      << sp.p.z << ")\n";
    pointCnt++;
  }

#undef FCHN
  str << "glEnd()\n";
  str << "end\n";

  // logging
  long long t1 = tm.elapsed();
  std::string S = str.str();
  long long t2 = tm.elapsed();
  display.setLuaGlObject("grab." + devid, devid, S);
  long long t3 = tm.elapsed();
  if (1) {
    str.str("");
    str.clear();
    str << "<h3>Grabber - SendPoints: " + devid + "</h3>";
    str << "Points: " << pointCnt << "<br>";
    str << "Strlen: " << S.length() << "<br>";
    str << "Generated: " << t1 << "ms from start (in " << t1 << "ms).<br>";
    str << "Converted: " << t2 << "ms from start (in " << (t2-t1) << "ms).<br>";
    str << "Sent: " << t3 << "ms from start (in " << (t3-t2) << "ms).<br>";
    display.setHtml("INFO", "log.grab." + devid, str.str());
  }
}
#endif

class CExtraPcPointSaver: public CExtraSaver
{
public:
  void save()
  {
    // copy from binary to text representation
    std::ifstream fi(tmpFilename, std::ios::binary | std::ios::in);
    std::ofstream fo(finalFilename);
    std::vector<unsigned char> colors; // Depends on Math::ColorRGB
    std::vector<double> locs; // Depends on Math::Vector3
    unsigned long size;
    fi >> size;
    colors.resize(size * 3);
    locs.resize(size * 3);
    fi.read((char*)&colors[0], sizeof(colors[0]) * colors.size());
    fi.read((char*)&locs[0], sizeof(locs[0]) * locs.size());
    fi.close();
    fo << ";;R\tG\tB\tx\ty\tz" << std::endl;
    fo.precision(6);
    for (int i=0; i < size; i++) {
      int ie = i * 3;
      fo << (unsigned int)colors[ie] << "\t"
        << (unsigned int)colors[ie+1] << "\t"
        << (unsigned int)colors[ie+2] << "\t";
      fo << locs[ie] << "\t"
        << locs[ie+1] << "\t"
        << locs[ie+2] << std::endl;
    }
    fo.close();

    remove(tmpFilename.c_str());
  }
};

// Save surface points as R G B X Y Z, tab-separated
CExtraSaverPtr CGrabbedPcPoints::save(const CRecordingInfo& frameInfo, int deviceId)
{
#if 0
  // Saving text: 600-900ms
  std::string fullname = makeFilename(frameInfo, deviceId, ".dat");
  std::ofstream fo(fullname);
  fo << ";;R\tG\tB\tx\ty\tz" << std::endl;
  for (auto sfp : mPoints) {
    fo << (unsigned int)sfp.c.r << "\t" << (unsigned int)sfp.c.g << "\t" << (unsigned int)sfp.c.b << "\t";
    fo.precision(6);
    fo << sfp.p.x << "\t" << sfp.p.y << "\t" << sfp.p.z
      << std::endl;
  }
  fo.close();
  return 0;
#elif 0
  // Saving binary values to stream one by one: 100-200ms
  std::string fullname = makeTempFilename(frameInfo, deviceId, ".dat");
  std::ofstream fo(fullname, std::ios::binary | std::ios::out);
  unsigned long size = mPoints.size();
  fo << size;
  for (auto sfp : mPoints) {
    fo << sfp.c.r << sfp.c.g << sfp.c.b;
    fo << sfp.p.x << sfp.p.y << sfp.p.z;
  }
  fo.close();
#else
  // HACK : Saving binary values as large fields: 10-40ms
  std::string fullname = makeTempFilename(frameInfo, deviceId, ".dat");
  std::ofstream fo(fullname, std::ios::binary | std::ios::out);
  unsigned long size = mPoints.size();
  std::vector<char> colors; // Depends on Math::ColorRGB; save as signed (Ice!), load as unsigned (PointCloud)
  std::vector<double> locs; // Depends on Math::Vector3
  colors.reserve(size * 3);
  locs.reserve(size * 3);
  for (auto sfp : mPoints) {
    colors.push_back(sfp.c.r);
    colors.push_back(sfp.c.g);
    colors.push_back(sfp.c.b);
    locs.push_back(sfp.p.x);
    locs.push_back(sfp.p.y);
    locs.push_back(sfp.p.z);
  }
  fo << size;
  fo.write((const char*)&colors[0], sizeof(colors[0]) * colors.size());
  fo.write((const char*)&locs[0], sizeof(locs[0]) * locs.size());
  fo.close();
#endif

  CExtraSaverPtr saver = CExtraSaverPtr(new CExtraPcPointSaver());
  saver->tmpFilename = fullname;
  saver->finalFilename = makeFilename(frameInfo, deviceId, ".dat");
  return saver;
}
#endif

} // namespace
/* vim: set sw=2 ts=8 et: */
