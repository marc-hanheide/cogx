/**
 * @author Marko Mahnič
 * @created September 2010
 */

#include "VideoGrabber.h"
#include "StringFmt.h"

#include <castutils/Timers.hpp>
#include <VideoUtils.h>

#include <string>

namespace cogxgrabber
{

IplImage* cloneVideoImage(const Video::Image &img)
{
   // HACK: Share data between IplImage and vector
   IplImage* pImg = cvCreateImageHeader(cvSize(img.width, img.height), IPL_DEPTH_8U, 3);
   pImg->imageData = (char*) &(img.data[0]);
   pImg->imageDataOrigin = pImg->imageData;
   pImg->widthStep = img.width * 3;
   pImg->imageSize = pImg->widthStep * img.height;
   return pImg;
}

void releaseClonedImage(IplImage** pImagePtr)
{
   if (!pImagePtr) return;
   if (*pImagePtr) {
      // HACK: Share data between IplImage and vector
      (*pImagePtr)->imageData = NULL;
      (*pImagePtr)->imageDataOrigin = NULL;
      cvReleaseImage(pImagePtr);
   }
}

std::string CGrabbedItem::makeFilename(const CRecordingInfo& frameInfo, int deviceId, const std::string& ext)
{
   std::string fname = frameInfo.filenamePatt;
   std::string sval = _str_(frameInfo.counter, frameInfo.counterDigits, '0');
   _s_::replace(fname, "%c", sval);

   std::string fullname = fname;
   std::string devname;
   if (deviceId < frameInfo.deviceNames.size())
      devname = frameInfo.deviceNames[deviceId];
   else devname = "d" + _str_(deviceId, 2, '0');
   _s_::replace(fullname, "%d", devname);

   fullname = frameInfo.directory + "/" + fullname + ext;

   return fullname;
}

std::string CGrabbedItem::makeTempFilename(const CRecordingInfo& frameInfo, int deviceId, const std::string& ext)
{
   std::string fname = "/tmp/grabber-"
      + _str_(frameInfo.counter, 8, '0') + "-" + _str_(deviceId, 4, '0')
      + ext;
   return fname;
}

CVideoGrabClient::CVideoGrabClient(cast::CASTComponent* pComponent)
   : CCastLoggerMixin(pComponent)
{
   mName = "Video Grabber " + _str_(mId);
}

void CVideoGrabClient::grab(std::vector<CGrabbedItemPtr>& items)
{
   std::vector<Video::CCachedImagePtr> timgs;
   getCachedImages(timgs);
   for(auto itt = timgs.begin(); itt != timgs.end(); itt++) {
      items.push_back(CGrabbedItemPtr(new CGrabbedCachedImage(*itt)));
      //pack.images.push_back(*itt);
   }
   //if (_getLockedCount() > 10) {
   //   println(" **** Client %ld - Locked images: %ld", mId, _getLockedCount());
   //}
}

void CVideoGrabClient::getPreviews(std::vector<CPreview>& previews,
      int width, int height, bool isGrabbing)
{
   std::vector<Video::CCachedImagePtr> timgs;
   getCachedImages(timgs);

   int i = 0;
   for (auto pCachedImg : timgs) {
      std::string cid = "pv-video-" + _str_(mId) + "." + _str_(i, 2, '0');
      previews.push_back(CPreview(cid, width, height));
      CPreview& pv = previews.back();

      pv.deviceName = "video." + _str_(mId) + "." + _str_(i);
      pv.deviceInfo = _str_(pCachedImg->width, "d") + "x" + _str_(pCachedImg->height, "d");

      IplImage *pPreview = pv.getImage();
      IplImage *pOrig = cloneVideoImage(*pCachedImg);
      cvResize(pOrig, pPreview);
      releaseClonedImage(&pOrig);
      i++;
   }
}

class CExtraImageSaver: public CExtraSaver
{
public:
   void save()
   {
      IplImage *pImage = cvLoadImage(tmpFilename.c_str());
      cvSaveImage(finalFilename.c_str(), pImage);
      cvReleaseImage(&pImage);
      remove(tmpFilename.c_str());
   }
};

CExtraSaverPtr CGrabbedCachedImage::save(const CRecordingInfo& frameInfo, int deviceId)
{
   // TODO: conversion to GS when saving;
   // TODO: compression parameters for jpeg and png
   std::string fullname = makeTempFilename(frameInfo, deviceId, ".bmp");
   // TODO: println("Saving image: %s", fullname.c_str());
   // TODO: if (!m_fakeRecording) {
   IplImage *iplImage = cloneVideoImage(*mpImage);
   cvSaveImage(fullname.c_str(), iplImage);
   releaseClonedImage(&iplImage);
   //}

   CExtraSaverPtr saver = CExtraSaverPtr(new CExtraImageSaver());
   saver->tmpFilename = fullname;
   saver->finalFilename = makeFilename(frameInfo, deviceId, ".png");
   return saver;
}

CExtraSaverPtr CGrabbedImage::save(const CRecordingInfo& frameInfo, int deviceId)
{
   // TODO: conversion to GS when saving;
   // TODO: compression parameters for jpeg and png
   std::string fullname = makeTempFilename(frameInfo, deviceId, ".bmp");
   IplImage *iplImage = cloneVideoImage(mImage);
   cvSaveImage(fullname.c_str(), iplImage);
   releaseClonedImage(&iplImage);

   CExtraSaverPtr saver = CExtraSaverPtr(new CExtraImageSaver());
   saver->tmpFilename = fullname;
   saver->finalFilename = makeFilename(frameInfo, deviceId, ".png");
   return saver;
}


} // namespace
/* vim: set sw=2 ts=8 et: */
