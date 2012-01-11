
#include "Snapper.h"

#include <../../VisionUtils.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <fstream>

using namespace std;
using namespace VisionData;
using namespace Video;

namespace cast
{

Snapper::Snapper()
{
  m_LastProtoObject = NULL;
  m_snapshotFiles = "xdata/snapshot/soifilter";
  m_snapshotFlags = "psmh"; // points, segmented image, mask, shape descriptor histogram; no video image
  m_idLeftImage = 0;
  m_idRightImage = 1;
  m_bAutoSnapshot = false;

}

void Snapper::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  if((it = _config.find("--snapfiles")) != _config.end())
  {
   istringstream str(it->second);
   str >> m_snapshotFiles;
  }

  if((it = _config.find("--snapvideo")) != _config.end())
  {
   istringstream str(it->second);
   str >> m_idLeftImage;
   str >> m_idRightImage;
   m_snapshotFlags = m_snapshotFlags + "v";
  }

  if((it = _config.find("--autosnap")) != _config.end())
  {
   istringstream str(it->second);
   string val;
   str >> val;
   m_bAutoSnapshot = (val == "" || val == "true");
  }
}

bool Snapper::hasSnapFlag(char ch)
{
  size_t pos = m_snapshotFlags.find_first_of(string(1, ch) + "A");
  return pos >= 0;
}

void Snapper::snapVideo()
{
  if (hasSnapFlag('v')) {
    // These images are only for snapshots; not used otherwise
    videoServer->getImage(m_idLeftImage, m_LeftImage);
    videoServer->getImage(m_idRightImage, m_RightImage);
  }
}

void Snapper::saveImage(const std::string& name, const std::string& path, const Video::Image& image)
{
  try {
    IplImage *iplImg = convertImageToIpl(image);
    if (iplImg) {
      cvSaveImage(path.c_str(), iplImg);
      cvReleaseImage(&iplImg);
      logger->log("Saved %s to '%s'", name.c_str(), path.c_str());
    }
  }
  catch (...) {
    logger->println("Failed to save %s to '%s'", name.c_str(), path.c_str());
  }
}

void Snapper::saveMask(const std::string& name, const std::string& path, const VisionData::SegmentMask& image)
{
  try {
    IplImage *iplImg = convertBytesToIpl(image.data, image.width, image.height, 1);
    if (iplImg) {
      cvSaveImage(path.c_str(), iplImg);
      cvReleaseImage(&iplImg);
      logger->log("Saved %s to '%s'", name.c_str(), path.c_str());
    }
  }
  catch (...) {
    logger->println("Failed to save %s to '%s'", name.c_str(), path.c_str());
  }
}

void Snapper::saveSnapshot()
{
  ProtoObjectPtr pobj(m_LastProtoObject); // A copy to avoid threading problems

  if (pobj.get() == NULL) {
    logger->debug("Snapshot: no proto object saved");
    return;
  }

  time_t rawtime;
  struct tm * timeinfo;
  char buf[80];
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );
  strftime(buf, 80, "%Y%m%d_%H%M%S", timeinfo);

  std::ostringstream ss;
  ss << m_snapshotFiles << buf;
  std::string path = ss.str();

  if (hasSnapFlag('p')) {
    // All 3D points from ProtoObject
    try {
      ofstream fpoints;
      fpoints.exceptions ( ofstream::eofbit | ofstream::failbit | ofstream::badbit );
      fpoints.open(string(path + "p.txt").c_str(), ofstream::out);
      fpoints << ";;r\tg\tb\tx\ty\tz" << endl;
      for (auto it = pobj->points.begin(); it != pobj->points.end(); it++) {
        fpoints << (unsigned int)it->c.r << "\t" << (unsigned int)it->c.g
          << "\t" << (unsigned int)it->c.b << "\t";
        fpoints << it->p.x << "\t" << it->p.y << "\t" << it->p.z << endl;
      }
      fpoints.close();
      logger->log("Saved points to '%sp.txt'", path.c_str());
    }
    catch (...) {
      logger->println("Failed to save points to '%sp.txt'", path.c_str());
    }
  }

  if (hasSnapFlag('p')) { // XXX: currently the same flag as for the 3D points
    // Individual surface patches
    try {
      int cnt=1;
      for (vector<SurfacePatch>::iterator it1 = pobj->surfacePatches.begin(); it1 != pobj->surfacePatches.end(); it1++)
      {

        ofstream fpoints;
        fpoints.exceptions ( ofstream::eofbit | ofstream::failbit | ofstream::badbit );
        std::stringstream filename;
        filename << "patch" << cnt << ".txt";
        fpoints.open(string(path + filename.str()).c_str(), ofstream::out);
        fpoints << ";;r\tg\tb\tx\ty\tz" << endl;
        for (auto it = it1->points.begin(); it != it1->points.end(); it++) {
          fpoints << (unsigned int)it->c.r << "\t" << (unsigned int)it->c.g
            << "\t" << (unsigned int)it->c.b << "\t";
          fpoints << it->p.x << "\t" << it->p.y << "\t" << it->p.z << endl;
        }
        fpoints.close();
        cnt++;
        logger->log("Saved SurfacePatch to '%s%s'", path.c_str(), filename.str().c_str());
      }
    }
    catch (...) {
      logger->println("Failed to save SurfacePatch to '%spatchX.txt'", path.c_str());
    }
  }

  if (hasSnapFlag('h')) try {
    ofstream fhist;
    fhist.exceptions ( ofstream::eofbit | ofstream::failbit | ofstream::badbit );
    fhist.open(string(path + "h.txt").c_str(), ofstream::out);
    for (auto it = pobj->rasShapeDesc.angleHistogram.begin(); it != pobj->rasShapeDesc.angleHistogram.end(); it++) {
      fhist << (double)it[1] << " ";
    }
    fhist.close();
    logger->log("Saved shape descriptor histogram to '%sh.txt'", path.c_str());
  }
  catch (...) {
    logger->println("Failed to save shape descriptor histogram to '%sh.txt'", path.c_str());
  }

  if (hasSnapFlag('s'))
    saveImage("segmented image", string(path + "s.png"), pobj->image);

  if (hasSnapFlag('m'))
    saveMask("image mask", string(path + "m.png"), pobj->mask);

  if (hasSnapFlag('v')) {
    saveImage("left video image", string(path + "vl.png"), m_LeftImage);
    saveImage("right video image", string(path + "vr.png"), m_RightImage);
  }
}

} // namespace
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */
