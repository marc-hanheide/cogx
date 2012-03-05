#ifndef _VIDEOGRABBER_CORESTRUCTS_H_4F547079_
#define _VIDEOGRABBER_CORESTRUCTS_H_4F547079_

#include <string>
#include <vector>
#include <memory>

#include <ImageCache.h>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

namespace cogxgrabber
{

struct CRecordingInfo
{
  bool recording;
  std::string directory;
  long directoryStatus; // 0: don't; 1: check; 2: exists; -1: error
  std::string filenamePatt;
  std::vector<std::string> deviceNames;
  long counterDigits;
  long counterStart;
  long counterEnd;
  long counter;
  long missedTicks;
  IceUtil::Time tmStart;
  IceUtil::Time tmEnd;
  CRecordingInfo() {
    recording = false;
    counterStart = 0;
    counterEnd = 0;
    counter = 0;
    missedTicks = 0;
#if 1 // XXX testing
    directory = "/tmp/path/to/Model";
    filenamePatt = "image-%c-%d.png";
    deviceNames.push_back("L");
    deviceNames.push_back("R");
    counterDigits = 3;
    tmStart = IceUtil::Time::seconds(0);
    tmEnd = tmStart;
#endif
  }
};

// Some data is saved in a temporary file to leave more time and memory for grabbing.
// The extra saver opens this file and transforms the data to the final form;
class CExtraSaver
{
public:
  std::string tmpFilename;
  std::string finalFilename;
  virtual void save() = 0;
};
typedef std::shared_ptr<CExtraSaver> CExtraSaverPtr;

class CGrabbedItem
{
public:
  std::string mName;

public:
  std::string makeFilename(const CRecordingInfo& recinfo, int deviceId, const std::string& ext=".png");
  std::string makeTempFilename(const CRecordingInfo& recinfo, int deviceId, const std::string& ext=".tmp");
  // # of devices from which the contents was grabbed; usually 1
  virtual int numDevices()
  {
    return 1;
  }
  virtual ~CGrabbedItem()
  {
  }
  virtual CExtraSaverPtr save(const CRecordingInfo& recinfo, int deviceId) = 0;
};

class CPreview
{
  static Video::CIplImageCache gCache;
  int mWidth;
  int mHeight;
  std::string mName;
public:
  std::string deviceName;
  std::string deviceInfo;

public:
  CPreview(const std::string& name, int width, int height)
  {
    mName = name;
    mWidth = width;
    mHeight = height;
  }
  IplImage* getImage()
  {
    return gCache.getImage(mName, mWidth, mHeight, 8, 3);
  }
};

typedef std::shared_ptr<CGrabbedItem> CGrabbedItemPtr;

class CDataSource
{
  static int count;
protected:
  int mId;

public:
  std::string mName;
  CDataSource()
  {
    mId = count;
    ++count;
  }
  virtual ~CDataSource()
  {
    --count;
  }
  virtual void grab(std::vector<CGrabbedItemPtr>& items) = 0;
  virtual void getPreviews(std::vector<CPreview>& items, int width, int height, bool isGrabbing) = 0;
#ifdef FEAT_VISUALIZATION
  virtual void configExtraV11n(cogx::display::CDisplayClient& display)
  {
  }
  virtual void displayExtra(cogx::display::CDisplayClient& display)
  {
  }
#endif
};

} // namespace
#endif /* _VIDEOGRABBER_CORESTRUCTS_H_4F547079_ */
/* vim: set sw=2 ts=8 et: */
