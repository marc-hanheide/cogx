/**
 * @author Marko Mahnič
 * @date September 2010
 */

#include "VideoGrabber.h"
#include "StringFmt.h"

#include <highgui.h>
#include <VideoUtils.h>
#include <IceUtil/Timer.h>

#define IDOBJ_GRABBER "Video.Grabber"
#define IDOBJ_SETTINGS "Video.Grabber.Settings"

#define IDPART_SETTINGS_FORM "001.settings"
#define IDCMD_GRAB "video.grab"
#define IDCTRL_STREAMING "video.streaming"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx::CVideoGrabber();
   }
}

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <pthread.h> // XXX: Testing code
#include <unistd.h>

double fclocks()
{
   return 1.0 * clock() / CLOCKS_PER_SEC;
}
// --------------------------


namespace cogx
{

using namespace std;
using namespace VisionData;
using namespace cast;
namespace cxd = cogx::display;

void CVideoGrabber::configure(const map<string,string> & _config)
{
   map<string,string>::const_iterator it;

   if((it = _config.find("--videoname")) != _config.end()) {
      m_videoServerName = it->second;
   }


   if((it = _config.find("--camids")) != _config.end()) {
      istringstream str(it->second);
      int id;
      while(str >> id)
         m_camIds.push_back(id);
   }

#ifdef FEAT_VISUALIZATION
   m_display.configureDisplayClient(_config);
#endif

   // sanity checks: Have all important things be configured? Is the
   // configuration consistent?
   if(m_videoServerName.empty())
      throw runtime_error(exceptionMessage(__HERE__, "no video server name given"));
}

void CVideoGrabber::start()
{
#ifdef FEAT_VISUALIZATION
   m_display.connectIceClient(*this);
   m_display.installEventReceiver();
   m_display.createForms();

   // TODO: Bug: only one (the first) checkbox control is shown ...
   m_display.addCheckBox(IDOBJ_GRABBER, IDCTRL_STREAMING, "&Streaming");
   m_display.addButton(IDOBJ_GRABBER, IDCMD_GRAB, "&Grab");
   m_display.addCheckBox(IDOBJ_SETTINGS, IDCTRL_STREAMING, "&Streaming");
#endif

   Video::CVideoClient2* pVideo;
   pVideo = new Video::CVideoClient2();
   pVideo->setServer(this, m_videoServerName, m_camIds);
   pVideo->setReceiver(new Video::CReceiverMethod<CVideoGrabber>(this, &CVideoGrabber::receiveImages));
   m_video.push_back(pVideo);

#if 1 // XXX: code for testing
   std::vector<int> camIds;
   camIds.push_back(0);
   pVideo = new Video::CVideoClient2();
   pVideo->setServer(this, "VideoServer2", camIds);
   pVideo->setReceiver(new Video::CReceiverMethod<CVideoGrabber>(this, &CVideoGrabber::receiveImages));
   m_video.push_back(pVideo);
#endif

   for(unsigned int i = 0; i < m_video.size(); i++) {
      pVideo = m_video[i];
      pVideo->setCaching(true);
      pVideo->connect();
      pVideo->setReceiving(true);
   }
}

#ifdef FEAT_VISUALIZATION
void CVideoGrabber::CVvDisplayClient::createForms()
{
   //std::string head =
   //   "<style>"
   //   ".info { font-size: 90%; color: #808080; }"
   //   "</style>"
   //   ;
   //setHtmlHead(IDOBJ_SETTINGS, IDPART_SETTINGS_FORM, head);
   std::ostringstream ss;
   string help1, help2;
   ss <<
      "<table>"
      "<tr><td>Model: </td><td>"
      "<input type='text' name='model' style='width:20em;' />"
      "</td></tr>"
      "</table>";
   m_frmSettings.add(new cxd::CFormValues::field("model"));

   help1 = "Use \"%m\" to place the model name in the Directory.";
   help2 = "\"Create directory\" can only create one (the last) level.";
   ss <<
      "<table>"
      "<tr><td>Directory: </td><td>"
      "<input type='text' name='directory' style='width:20em;' title='" << help1 << "'/>"
      "</td></tr><tr><td>&nbsp;</td><td>"
      "<input type='checkbox' name='createdir' value='on' title='" << help2 << "'/>Create directory"
      "</td></tr>"
      "</table>";
   m_frmSettings.add(new cxd::CFormValues::field("directory"));
   m_frmSettings.add(new cxd::CFormValues::set("createdir", cxd::CFormValues::valuelist() << "on"));

   help1 = 
      "Filename to save to.\n"
      "Use \"%c\" to place the counter.\n"
      "Use \"%d\" to place the device name.\n"
      "Use \"%m\" to place the model name.\n";
   ss <<
      "<table>"
      "<tr><td>Filename: </td><td>"
      "<input type='text' name='filename' style='width:20em;' title='" << help1 << "'/>"
      "</td></tr>"
      "</table>";
   m_frmSettings.add(new cxd::CFormValues::field("filename"));

   ss << "<table>";
   vector<string> cntValues;
   ss << "<tr><td>Counter digits: </td><td>"
      "<select name='countersize' >";
   for(int i = 1; i < 10; i++) {
      ss << "<option>" << i << "</option>";
      cntValues.push_back(_str_(i));
   }
   ss << "</select>";

   ss <<
      "</td><td>Next value: <td>"
      "<input type='text' name='countervalue' style='width:5em;' />"
      "</td></tr>";
   m_frmSettings.add(new cxd::CFormValues::field("countervalue"));
   m_frmSettings.add(new cxd::CFormValues::choice("countersize", cxd::CFormValues::valuelist(cntValues)));
   ss << "</table>";

   help1 = "Space delimited list of device names (for paramerer \"%d\").";
   ss <<
      "<table>"
      "<tr><td>Device names: </td><td>"
      "<input type='text' name='devicenames' style='width:10em;' title='" << help1 << "' />"
      "</td></tr>"
      "</table>";
   m_frmSettings.add(new cxd::CFormValues::field("devicenames"));

   ss << "<input type='submit' value='Apply' />";

   ss << 
      "<div class='v11ninfo'>"
      "If you change anything, don't forget to 'Apply'.<br>"
      "You can 'Save' current values for later use.<br>"
      "After you 'Load' the saved values, press 'Apply'.<br>"
      "More help is available in the tooltips."
      "</div>";

   // Set form defaults
   setDirectory("xdata/grab/%m");
   setImageFilenamePatt("image%c-%d.png");
   setDeviceNames("L R");
   setCounterDigits(3);
   setCounterValue(0);
   
   setHtmlForm(IDOBJ_SETTINGS, IDPART_SETTINGS_FORM, ss.str());
}

std::string CVideoGrabber::CVvDisplayClient::getDirectory()
{
   std::string dir = _s_::strip(m_frmSettings.get("directory"));
   std::string model = _s_::strip(m_frmSettings.get("model"));
   _s_::replace(dir, "%m", model);
   return _s_::rstrip(dir, "/ ");
}

std::string CVideoGrabber::CVvDisplayClient::getModelName()
{
   std::string model = _s_::strip(m_frmSettings.get("model"));
   return model;
}

void CVideoGrabber::CVvDisplayClient::setDirectory(const std::string& dirname)
{
   m_frmSettings.setValue("directory", dirname);
}

bool CVideoGrabber::CVvDisplayClient::getCreateDirectory()
{
   return _s_::strip(m_frmSettings.get("createdir")) == "on";
}

std::string CVideoGrabber::CVvDisplayClient::getImageFilenamePatt()
{
   return _s_::strip(m_frmSettings.get("filename"));
}

void CVideoGrabber::CVvDisplayClient::setImageFilenamePatt(const std::string& pattern)
{
   m_frmSettings.setValue("filename", pattern);
}

std::string CVideoGrabber::CVvDisplayClient::getDeviceNames()
{
   return _s_::strip(m_frmSettings.get("devicenames"));
}

void CVideoGrabber::CVvDisplayClient::setDeviceNames(const std::string& names)
{
   m_frmSettings.setValue("devicenames", names);
}

long CVideoGrabber::CVvDisplayClient::getCounterDigits()
{
   // TODO: countersize is a set so getInt doesn't work ... 
   return atoi(m_frmSettings.get("countersize").c_str());
}

void CVideoGrabber::CVvDisplayClient::setCounterDigits(long nDigits)
{
   m_frmSettings.setValue("countersize", _str_(nDigits));
}

long CVideoGrabber::CVvDisplayClient::getCounterValue()
{
   return m_frmSettings.getInt("countervalue");
}

void CVideoGrabber::CVvDisplayClient::setCounterValue(long nValue)
{
   m_frmSettings.setValue("countervalue", _str_(nValue));
}

void CVideoGrabber::CVvDisplayClient::handleEvent(const Visualization::TEvent &event)
{
   //debug(event.data + " (received by VideoViewer)");
   if (event.type == Visualization::evCheckBoxChange) {
      if (event.sourceId == IDCTRL_STREAMING) {
         //debug(std::string("Time: ") + _str_(fclocks()));
         bool newrcv = (event.data != "0");
         if (newrcv != pViewer->m_bReceiving) {
            if(pViewer->m_bReceiving) {
               //pViewer->m_video.setReceiving(false);
               pViewer->println("Stopped receiving images");
            }
            else {
               //pViewer->m_video.setReceiving(true);
               pViewer->println("Started receiving images");
            }
            pViewer->m_bReceiving = !pViewer->m_bReceiving;
         }
      }
   }
   else if (event.type == Visualization::evButtonClick) {
      if (event.sourceId == IDCMD_GRAB) {
         pViewer->m_frameGrabCount++; // XXX: not thread safe
         setCounterValue(getCounterValue() + 1);

         std::map<std::string, std::string> fields;
         m_frmSettings.get(fields);
         setHtmlFormData(IDOBJ_SETTINGS, IDPART_SETTINGS_FORM, fields);
      }
   }
}

std::string CVideoGrabber::CVvDisplayClient::getControlState(const std::string& ctrlId)
{
   if (ctrlId == IDCTRL_STREAMING) {
      return pViewer->m_bReceiving ? "1" : "0";
   }
   return "";
}

void CVideoGrabber::CVvDisplayClient::handleForm(const std::string& id,
      const std::string& partId, const std::map<std::string, std::string>& fields)
{
   std::cout << " *** handleForm " << partId << std::endl;
   if (partId == IDPART_SETTINGS_FORM) {
      m_frmSettings.apply(fields);
   }
}

bool CVideoGrabber::CVvDisplayClient::getFormData(const std::string& id,
      const std::string& partId, std::map<std::string, std::string>& fields)
{
   if (partId == IDPART_SETTINGS_FORM) {
      m_frmSettings.get(fields);
      return true;
   }
   return false;
}

void CVideoGrabber::releaseCanvas()
{
   if (m_pDisplayCanvas) {
      // HACK: Share data between IplImage and vector
      m_pDisplayCanvas->imageData = NULL;
      m_pDisplayCanvas->imageDataOrigin = NULL;
      cvReleaseImage(&m_pDisplayCanvas);
   }
}

void CVideoGrabber::prepareCanvas(int width, int height)
{
   if (m_pDisplayCanvas == NULL
         || m_pDisplayCanvas->width != width || m_pDisplayCanvas->height != height)
   {
      releaseCanvas();
      // HACK: Share data between IplImage and vector
      m_pDisplayCanvas = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
      m_DisplayBuffer.resize(width * height * 3);
      m_pDisplayCanvas->imageData = (char*) &(m_DisplayBuffer[0]);
      m_pDisplayCanvas->imageDataOrigin = m_pDisplayCanvas->imageData;
      m_pDisplayCanvas->widthStep = width * 3;
      m_pDisplayCanvas->imageSize = m_pDisplayCanvas->widthStep * height;
   }
}

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

#endif

void CVideoGrabber::destroy()
{
}

void CVideoGrabber::sendCachedImages()
{
#ifdef FEAT_VISUALIZATION
   std::vector<Video::CCachedImagePtr> images;
   for (unsigned int i = 0; i < m_video.size(); i++) {
      Video::CVideoClient2& v = *m_video[i];
      std::vector<Video::CCachedImagePtr> timgs;
      v.getCachedImages(timgs);
      std::vector<Video::CCachedImagePtr>::iterator itt;
      for(itt = timgs.begin(); itt != timgs.end(); itt++) {
         images.push_back(*itt);
      }
   }

   int w = 0, h = 0;
   double factor = 1.0;
   for (unsigned int i = 0; i < images.size(); i++) {
      if (images[i]->width > w) w = images[i]->width;
      h += images[i]->height;
   }
   if (w > 320) {
      factor = 320.0 / w;
      w = (int) (w * factor + 0.5);
      h = (int) (h * factor + 0.5);
   }
   // TODO: every image has its own scale factor

   CvFont fntSimplex, fntPlain;
   cvInitFont(&fntSimplex, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 2);
   cvInitFont(&fntPlain, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0, 1.5);
   std::vector<std::string> devnames = getDeviceNames();
   int vp = 0;
   prepareCanvas(w, h);
   IplImage *pDisp = m_pDisplayCanvas;
   for (unsigned int i = 0; i < images.size(); i++) {
      IplImage *iplImage = cloneVideoImage(*images[i]);
      int wi = (int) (images[i]->width * factor);
      int hi = (int) (images[i]->height * factor);
      cvSetImageROI(pDisp, cvRect(0, vp, wi, vp+hi));
      cvResize(iplImage, pDisp);
      releaseClonedImage(&iplImage);

      std::string sMsg = _str_(i);
      if (i < devnames.size()) sMsg += ":" + devnames[i];
      cvPutText (pDisp, sMsg.c_str(), cvPoint(10, 25), &fntSimplex, cvScalar(255,255,0));
      
      sMsg = _str_(images[i]->width) + "x" + _str_(images[i]->height);
      cvPutText (pDisp, sMsg.c_str(), cvPoint(10, hi-2), &fntPlain, cvScalar(255,255,0));

      cvResetImageROI(pDisp);
      vp += hi;
   }
   m_display.setImage(IDOBJ_GRABBER, w, h, 3, m_DisplayBuffer);
#endif
}

void CVideoGrabber::receiveImages(const std::string& serverName, const std::vector<Video::Image>& _images)
{
   //println("%s 0x%lx", serverName.c_str(), pthread_self()); // XXX: Testing code

   //if (m_frameGrabCount > 0) {
   //   saveImages(images);
   //   m_frameGrabCount--; // XXX: not thread safe
   //}

   if (serverName != m_videoServerName) return;

#ifdef FEAT_VISUALIZATION
   //sendCachedImages();
#else
   IplImage *iplImage = convertImageToIpl(images[0]);
   cvShowImage(getComponentID().c_str(), iplImage);
   cvReleaseImage(&iplImage);
#endif
}

std::vector<std::string> CVideoGrabber::getDeviceNames()
{
   std::vector<std::string> devnames = _s_::split(m_display.getDeviceNames(), " ", 0, false);
   while (devnames.size() < m_camIds.size()) {
      int i = m_camIds[devnames.size()];
      devnames.push_back(_str_(i));
   }
   return devnames;
}

void CVideoGrabber::saveImages(const std::vector<Video::Image>& images)
{
   std::string dir = m_display.getDirectory();
   if (m_display.getCreateDirectory()) {
      struct stat finfo;
      int rv = stat(dir.c_str(), &finfo);
      bool exists = rv == 0;
      if (! exists) {
         mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
      }
   }
   std::string fname = m_display.getImageFilenamePatt();
   std::vector<std::string> devnames = getDeviceNames();
   long digits = m_display.getCounterDigits();
   long val = m_display.getCounterValue();
   if (digits < 1) digits = 1;
   if (digits > 9) digits = 9;
   std::string sval = _str_(val, digits, '0');
   _s_::replace(fname, "%c", sval);

   std::string model = m_display.getModelName();
   _s_::replace(fname, "%m", model);
 
   // TODO: conversion to GS when saving;
   // TODO: compression parameters for jpeg and png
   for (unsigned int i = 0; i < images.size(); i++) {
      std::string fullname = fname;
      _s_::replace(fullname, "%d", devnames[i]);
      fullname = dir + "/" + fullname;
      println("Saving image: %s", fullname.c_str());
      IplImage *iplImage = convertImageToIpl(images[i]);
      cvSaveImage(fullname.c_str(), iplImage);
      cvReleaseImage(&iplImage);
   }
}

class CTickSyncedTask
{
protected:
   IceUtil::Monitor<IceUtil::Mutex> m_tickerMutex;
   int m_ticked;

public:
   CTickSyncedTask()
   {
      m_ticked = 0;
   }

   void tick()
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_tickerMutex);
      m_ticked++;
      m_tickerMutex.notify();
   }

   int waitForTick(int ms = 500)
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_tickerMutex);
      if (! m_ticked)
         m_tickerMutex.timedWait(IceUtil::Time::milliSeconds(ms));
      if (! m_ticked) return 0;
      int rv = m_ticked;
      m_ticked = 0;
      return rv;
   }
};

class CTickerTask: public IceUtil::TimerTask
{
   CTickSyncedTask* m_pWorker;
public:
   CTickerTask(CTickSyncedTask* pWorker)
   {
      assert(pWorker);
      m_pWorker = pWorker;
   }
   void runTimerTask()
   {
      // This should be as short as possible so that we get
      // approximately equal time periods.
      m_pWorker->tick();
   }
};


class CSaveQueThread: public IceUtil::Thread, public CTickSyncedTask
{
   CVideoGrabber *m_pGrabber;

public:
   struct CItem 
   {
      long frameId;
      std::vector<Video::CCachedImagePtr> images;
   };
   std::vector<CItem> items;

public:
   CSaveQueThread(CVideoGrabber *pGrabber)
   {
      m_pGrabber = pGrabber;
   }

   virtual void run()
   {  
      while (m_pGrabber && m_pGrabber->isRunning()) {
         int ticks = waitForTick(500);
         if (!ticks) continue;
         if (ticks > 1) 
            m_pGrabber->println(string(" *** missed ticks ") + _str_(ticks-1));

         IceUtil::Time tm = IceUtil::Time::now();
         m_pGrabber->println("%lld runTimerTask()", tm.toMilliSeconds());

         CItem pack;
         std::vector<Video::CCachedImagePtr> queue;
         for (unsigned int i = 0; i < m_pGrabber->m_video.size(); i++) {
            std::vector<Video::CCachedImagePtr> timgs;
            Video::CVideoClient2& v = *m_pGrabber->m_video[i];
            v.getCachedImages(pack.images);
            std::vector<Video::CCachedImagePtr>::iterator itt;
            for(itt = timgs.begin(); itt != timgs.end(); itt++) {
               pack.images.push_back(*itt);
            }
         }
         pack.frameId = m_pGrabber->m_display.getCounterValue();
         m_pGrabber->m_display.setCounterValue(pack.frameId + 1);
         items.push_back(pack);
         IceUtil::Time tm2 = IceUtil::Time::now() - tm;
         m_pGrabber->println("images copied in %lld", tm2.toMilliSeconds());
      }
   }
};

class CDrawingThread: public IceUtil::Thread, public CTickSyncedTask
{
   CVideoGrabber *m_pGrabber;

public:
   struct CItem 
   {
      long frameId;
      std::vector<Video::CCachedImagePtr> images;
   };
   std::vector<CItem> items;

public:
   CDrawingThread(CVideoGrabber *pGrabber)
   {
      m_pGrabber = pGrabber;
   }

   virtual void run()
   {  
      while (m_pGrabber && m_pGrabber->isRunning()) {
         int ticks = waitForTick(500);
         if (!ticks) continue;

         m_pGrabber->sendCachedImages();
      }
   }
};

void CVideoGrabber::runComponent()
{
   sleepComponent(1000);
   IceUtil::Timer timer;
   IceUtil::ThreadPtr pQueue = new CSaveQueThread(this);
   IceUtil::ThreadPtr pDrawer = new CDrawingThread(this);
   CTickerTask saveTick(dynamic_cast<CTickSyncedTask*>(pQueue.get()));
   CTickerTask drawTick(dynamic_cast<CTickSyncedTask*>(pDrawer.get()));

   // Unfortunately Timer isn't exact -- (arbitrary) time spent by runTimerTask
   // is added to the time of a period; because of this runTimerTask must be
   // extremely fast so instead of copying image data we implement a locking
   // image cache and copy only the pointers to locked images; this should take
   // under 1 microsecond per period. Synchronization/multithreading may make
   // this time much longer.
   // XXX With CTickSyncedTask the grabber lost 10ms in 10s. Find a better implementation of Timer.
   //timer.scheduleRepeated(&saveTick, IceUtil::Time::milliSeconds(200));
   timer.scheduleRepeated(&drawTick, IceUtil::Time::milliSeconds(200));

   IceUtil::ThreadControl tcQueue = pQueue->start();
   IceUtil::ThreadControl tcDrawer = pDrawer->start();

   while(isRunning()) {
      sleepComponent(300);
   }

   tcQueue.join();
   tcDrawer.join();

   // XXX: A crash, with both cancel() and destroy()
   //timer.cancel(&saveTick); timer.cancel(&drawTick);
   timer.destroy();
}

} // namespace
/* vim: set sw=3 ts=8 et: */
