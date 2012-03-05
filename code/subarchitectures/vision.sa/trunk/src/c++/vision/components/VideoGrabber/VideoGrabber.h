/**
 * @author Marko Mahnič
 * @created September 2010
 *
 */

#ifndef VIDEOGRABBER_V7JAPUYG
#define VIDEOGRABBER_V7JAPUYG

#include "CoreStructs.h"

#include <Video.hpp>
#include <VideoClient2.h>
#include <ImageCache.h>

#include <castutils/CastLoggerMixin.hpp>

namespace cogxgrabber
{

class CVideoGrabClient: public Video::CVideoClient2, public CDataSource,
public castutils::CCastLoggerMixin
{
public:
  CVideoGrabClient(cast::CASTComponent* pComponent);
  virtual void grab(std::vector<CGrabbedItemPtr>& items) /*override*/;
  virtual void getPreviews(std::vector<CPreview>& previews,
      int width, int height, bool isGrabbing) /*override*/;
};

class CGrabbedCachedImage: public CGrabbedItem
{
private:
  friend class CVideoGrabClient;
  Video::CCachedImagePtr mpImage;
public:
  CGrabbedCachedImage(Video::CCachedImagePtr& pimage)
  {
    mName = "Cached Image";
    mpImage = pimage;
  }
  virtual CExtraSaverPtr save(const CRecordingInfo& recinfo, int deviceId) /*override*/;
};

class CGrabbedImage: public CGrabbedItem
{
public:
  Video::Image mImage;
  CGrabbedImage()
  {
    mName = "Image";
  }
  virtual CExtraSaverPtr save(const CRecordingInfo& recinfo, int deviceId) /*override*/;
};


extern IplImage* cloneVideoImage(const Video::Image &img);
extern void releaseClonedImage(IplImage** pImagePtr);

} // namespace
#endif
/* vim: set sw=2 ts=8 et: */

