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
#define IDPART_SETTINGS_VALUES "002.values"
#define IDCMD_GRAB   "video.grab"
#define IDCMD_RECORD "video.record"
#define IDCMD_STOP   "video.stop"
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
   map<string,string>::const_iterator it, iCam;

   string serverName;
   vector<int> camids;
   int id;
   Video::CVideoClient2* pVideo;

   for (it = _config.begin(); it != _config.end(); it++) {
      string sffx = "";

      if (it->first == "--videoname") iCam = _config.find("--camids");
      else if (_s_::startswith(it->first, "--videoname.")) {
         sffx = it->first.substr(11);
         iCam = _config.find("--camids" + sffx);
      }
      else continue;

      serverName = _s_::strip(it->second);
      if (serverName == "") continue;
      camids.clear();

      if (iCam != _config.end()) {
         istringstream str(iCam->second);
         while(str >> id) camids.push_back(id);
      }

      if (camids.size() < 1) {
         throw runtime_error(exceptionMessage(__HERE__, 
                  "Grabber id='%s': No --camids%s for VideoServer id='%s'.",
                  getComponentID().c_str(), sffx.c_str(), serverName.c_str()));
      }

      pVideo = new Video::CVideoClient2();
      pVideo->setServer(this, serverName, camids);
      pVideo->setReceiver(new Video::CReceiverMethod<CVideoGrabber>(this, &CVideoGrabber::receiveImages));
      m_video.push_back(pVideo);
   }

   if((it = _config.find("--fakegrabbing")) != _config.end()) {
      string s = _s_::lower(it->second);
      if (s == "1" || s == "yes" || s == "true" || s == "on")
         m_fakeRecording = true;
   }

   if((it = _config.find("--grab_ms")) != _config.end()) {
      istringstream str(it->second);
      str >> m_frameGrabMs;
   }
   if (m_frameGrabMs < 10) m_frameGrabMs = 10;

#ifdef FEAT_VISUALIZATION
   m_display.configureDisplayClient(_config);
#endif

   if(m_video.size() < 1)
      throw runtime_error(exceptionMessage(__HERE__, "No video server name given"));
}

void CVideoGrabber::start()
{
#ifdef FEAT_VISUALIZATION
   m_display.connectIceClient(*this);
   m_display.installEventReceiver();
   m_display.createForms();

   // TODO: Server Bug: only one (the first) checkbox control is shown (same id)...
   m_display.addCheckBox(IDOBJ_GRABBER, IDCTRL_STREAMING, "Streaming");
   m_display.addButton(IDOBJ_GRABBER, IDCMD_GRAB, "&Grab");
   m_display.addButton(IDOBJ_GRABBER, IDCMD_RECORD, "&Record");
   m_display.addButton(IDOBJ_GRABBER, IDCMD_STOP, "&Stop Recording");
   m_display.addCheckBox(IDOBJ_SETTINGS, IDCTRL_STREAMING, "Streaming");
#endif

   Video::CVideoClient2* pVideo;
#if 0 // XXX: code for testing
   pVideo = new Video::CVideoClient2();
   pVideo->setServer(this, m_videoServerName, m_camIds);
   pVideo->setReceiver(new Video::CReceiverMethod<CVideoGrabber>(this, &CVideoGrabber::receiveImages));
   m_video.push_back(pVideo);

   std::vector<int> camIds;
   camIds.push_back(0);
   pVideo = new Video::CVideoClient2();
   pVideo->setServer(this, "VideoServer2", camIds);
   pVideo->setReceiver(new Video::CReceiverMethod<CVideoGrabber>(this, &CVideoGrabber::receiveImages));
   m_video.push_back(pVideo);

   pVideo = new Video::CVideoClient2();
   pVideo->setServer(this, "VideoServer3", camIds);
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

   help1 = "Stop recording after limited time. Enter 0 or less for no limit.";
   ss <<
      "<table>"
      "<tr><td>Recording limit: </td><td>"
      "<input type='text' name='recordtimelimit' style='width:10em;' title='" << help1 << "' /> milliseconds"
      "</td></tr>"
      "</table>";
   m_frmSettings.add(new cxd::CFormValues::field("recordtimelimit"));

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
   setRecordTimeLimit(5000);
   
   setHtmlForm(IDOBJ_SETTINGS, IDPART_SETTINGS_FORM, ss.str());
   showCurrentSettings();
}

void CVideoGrabber::CVvDisplayClient::showCurrentSettings()
{
   std::ostringstream ss;
   ss <<
      "<hr>"
      "Current settings:"
      "<div>"
      "Saving to: " << getDirectory() << "/" << getImageFilenamePatt() << "<br>"
      "Next counter value: " << getCounterValue() << "<br>"
      "Time limit: " << getRecordTimeLimit() << "ms<br>"
      "Grabbing speed: " << pViewer->m_frameGrabMs << "ms, "
      << _str_(1000.0/pViewer->m_frameGrabMs, 0, 3, ' ') << "fps<br>"
      "</div>";
   setHtml(IDOBJ_SETTINGS, IDPART_SETTINGS_VALUES, ss.str());
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

long CVideoGrabber::CVvDisplayClient::getRecordTimeLimit()
{
   return m_frmSettings.getInt("recordtimelimit");
}

void CVideoGrabber::CVvDisplayClient::setRecordTimeLimit(long nValue)
{
   m_frmSettings.setValue("recordtimelimit", _str_(nValue));
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
         pViewer->startGrabbing(IDCMD_GRAB);
      }
      else if (event.sourceId == IDCMD_RECORD) {
         pViewer->startGrabbing(IDCMD_RECORD);
      }
      else if (event.sourceId == IDCMD_STOP) {
         pViewer->stopGrabbing();
      }
   }
}

void CVideoGrabber::CVvDisplayClient::updateDisplay()
{
   std::map<std::string, std::string> fields;
   m_frmSettings.get(fields);
   setHtmlFormData(IDOBJ_SETTINGS, IDPART_SETTINGS_FORM, fields);
   showCurrentSettings();
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
      showCurrentSettings();
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
#if 0
#ifdef FEAT_VISUALIZATION
   //sendCachedImages();
#else
   IplImage *iplImage = convertImageToIpl(images[0]);
   cvShowImage(getComponentID().c_str(), iplImage);
   cvReleaseImage(&iplImage);
#endif
#endif
}

std::vector<std::string> CVideoGrabber::getDeviceNames()
{
   std::vector<std::string> devnames = _s_::split(m_display.getDeviceNames(), " ", 0, false);
   // TODO: add missing device names, form m_video
   //while (devnames.size() < m_camIds.size()) {
   //   int i = m_camIds[devnames.size()];
   //   devnames.push_back(_str_(i));
   //}
   return devnames;
}

void CVideoGrabber::fillRecordingInfo(CRecordingInfo &info)
{
   info.directory = m_display.getDirectory();
   info.directoryStatus = 0;
   if (m_display.getCreateDirectory()) {
      info.directoryStatus = 1;
   }
   std::string fname = m_display.getImageFilenamePatt();
   std::string model = m_display.getModelName();
   _s_::replace(fname, "%m", model);
   info.filenamePatt = fname;

   long digits = m_display.getCounterDigits();
   if (digits < 1) digits = 1;
   if (digits > 9) digits = 9;
   info.counterDigits = digits;

   info.counterStart = m_display.getCounterValue();
   info.counterEnd = info.counterStart; // TODO: calculate from settings
   info.counter = info.counterStart;

   long reclimit = m_display.getRecordTimeLimit();
   info.tmStart = IceUtil::Time::now();
   info.tmEnd = info.tmStart + IceUtil::Time::milliSeconds(reclimit);

   info.deviceNames = getDeviceNames();
}

void CVideoGrabber::startGrabbing(const std::string& command)
{
   if (isGrabbing()) return;
   fillRecordingInfo(m_RecordingInfo);
   if (command == IDCMD_GRAB) {
      // On single-shot grabbing don't install the timer, just grab right now
      dynamic_cast<CSaveQueThread*>(m_pQueue.get())->grab();
      m_display.updateDisplay(); // send counter value
   }
   else if (command == IDCMD_RECORD) {
      m_RecordingInfo.recording = true;
      m_pTimer->scheduleRepeated(m_pQueueTick.get(), IceUtil::Time::milliSeconds(m_frameGrabMs));
   }
}

void CVideoGrabber::stopGrabbing()
{
   if (!isGrabbing()) return;
   m_pTimer->cancel(m_pQueueTick.get());
   m_RecordingInfo.recording = false;
   sleepComponent(100);
   m_display.updateDisplay();
}

void CVideoGrabber::checkStopGrabbing()
{
   CRecordingInfo& ri = m_RecordingInfo;
   if (ri.counterEnd > ri.counterStart && ri.counter >= ri.counterEnd) stopGrabbing();
   else if (ri.tmEnd > ri.tmStart && IceUtil::Time::now() >= ri.tmEnd) stopGrabbing();
}

// TODO: frameInfo should be const
void CVideoGrabber::saveQueuedImages(const std::vector<Video::CCachedImagePtr>& images,
      CRecordingInfo& frameInfo)
{
   if (frameInfo.directoryStatus == 0 || frameInfo.directoryStatus == 1) {
      struct stat finfo;
      string& dir = frameInfo.directory;
      bool exists = (0 == stat(dir.c_str(), &finfo));
      if (exists) frameInfo.directoryStatus = 2;
      else {
         if (frameInfo.directoryStatus == 0) 
            frameInfo.directoryStatus = -1;
         else {
            mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
            exists = (0 == stat(dir.c_str(), &finfo));
            frameInfo.directoryStatus = exists ? 2 : -1;
         }
      }
   }
   //if (frameInfo.directoryStatus < 0) return;

   std::string fname = frameInfo.filenamePatt;
   std::string sval = _str_(frameInfo.counter, frameInfo.counterDigits, '0');
   _s_::replace(fname, "%c", sval);

   // TODO: conversion to GS when saving;
   // TODO: compression parameters for jpeg and png
   for (unsigned int i = 0; i < images.size(); i++) {
      std::string fullname = fname;
      std::string devname;
      if (i < frameInfo.deviceNames.size()) devname = frameInfo.deviceNames[i];
      else devname = "d" + _str_(i, 2, '0');
      _s_::replace(fullname, "%d", devname);

      fullname = frameInfo.directory + "/" + fullname;
      println("Saving image: %s", fullname.c_str());
      if (!m_fakeRecording) {
         IplImage *iplImage = cloneVideoImage(*images[i]);
         cvSaveImage(fullname.c_str(), iplImage);
         releaseClonedImage(&iplImage);
      }
   }
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


CVideoGrabber::CSaveQueThread::CSaveQueThread(CVideoGrabber *pGrabber)
{
   m_pGrabber = pGrabber;
}

void CVideoGrabber::CSaveQueThread::getItems(
      std::vector<CVideoGrabber::CSaveQueThread::CItem>& items,
      unsigned int maxItems)
{
   IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_itemsLock);
   if (maxItems < 1 || maxItems >= m_items.size()) {
      items = m_items;
      m_items.clear();
      return;
   }
   items.clear();
   items.insert(items.end(), m_items.begin(), m_items.begin() + (maxItems-1));
   m_items.erase(m_items.begin(), m_items.begin() + (maxItems-1));
}

void CVideoGrabber::CSaveQueThread::grab()
{
   if (! m_pGrabber) return;
   //IceUtil::Time tm = IceUtil::Time::now();

   CItem pack;
   std::vector<Video::CCachedImagePtr> queue;
   std::vector<Video::CVideoClient2*> clients;
   m_pGrabber->getClients(clients);
   for (unsigned int i = 0; i < clients.size(); i++) {
      std::vector<Video::CCachedImagePtr> timgs;
      Video::CVideoClient2& v = *clients[i];
      v.getCachedImages(timgs);
      std::vector<Video::CCachedImagePtr>::iterator itt;
      for(itt = timgs.begin(); itt != timgs.end(); itt++) {
         pack.images.push_back(*itt);
      }
   }
   pack.frameInfo = m_pGrabber->m_RecordingInfo;

   m_pGrabber->m_RecordingInfo.counter++;
   m_pGrabber->m_display.setCounterValue(m_pGrabber->m_RecordingInfo.counter);

   IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_itemsLock);
   m_items.push_back(pack);

   //IceUtil::Time tm2 = IceUtil::Time::now() - tm;
   //m_pGrabber->debug("images copied in %lld micros", tm2.toMicroSeconds());
   // result: 2+1 images, 40us
}

void CVideoGrabber::CSaveQueThread::run()
{  
   while (m_pGrabber && m_pGrabber->isRunning()) {
      int ticks = waitForTick(500);
      if (!ticks) continue;
      if (!m_pGrabber->isGrabbing()) continue;
      if (ticks > 1) 
         m_pGrabber->println(string(" *** CSaveQueThread: missed ticks: ") + _str_(ticks-1));

      grab();
      m_pGrabber->checkStopGrabbing();
   }
}

CVideoGrabber::CDrawingThread::CDrawingThread(CVideoGrabber *pGrabber)
{
   m_pGrabber = pGrabber;
}

void CVideoGrabber::CDrawingThread::run()
{  
   while (m_pGrabber && m_pGrabber->isRunning()) {
      int ticks = waitForTick(500);
      if (!ticks) continue;

      m_pGrabber->sendCachedImages();
   }
}

void CVideoGrabber::runComponent()
{
   sleepComponent(1000);
   m_pQueue = new CSaveQueThread(this);
   m_pDrawer = new CDrawingThread(this);

   m_pQueueTick = new CTickerTask(dynamic_cast<CTickSyncedTask*>(m_pQueue.get()));
   m_pDrawTick  = new CTickerTask(dynamic_cast<CTickSyncedTask*>(m_pDrawer.get()));

   IceUtil::ThreadControl tcQueue = m_pQueue->start();
   IceUtil::ThreadControl tcDrawer = m_pDrawer->start();

   // Unfortunately Timer isn't exact -- (arbitrary) time spent by runTimerTask
   // is added to the time of a period; because of this runTimerTask must be
   // extremely fast so instead of copying image data we implement a locking
   // image cache and copy only the pointers to locked images; this should take
   // under 1 microsecond per period. Synchronization/multithreading may make
   // this time much longer.
   // XXX With CTickSyncedTask the grabber lost 10ms in 10s. Find a more precise implementation of Timer.
   m_pTimer = new IceUtil::Timer();
   m_pTimer->scheduleRepeated(m_pDrawTick.get(), IceUtil::Time::milliSeconds(200));

   while(isRunning()) {
      vector<CSaveQueThread::CItem> toSave;
      dynamic_cast<CSaveQueThread*>(m_pQueue.get())->getItems(toSave, 4);
      if (toSave.size() < 1) {
         sleepComponent(200);
         continue;
      }
      debug("Will save %d frames", toSave.size());
      vector<CSaveQueThread::CItem>::iterator it;
      for (it = toSave.begin(); it != toSave.end(); it++) {
         IceUtil::Time tm = IceUtil::Time::now();
         saveQueuedImages(it->images, it->frameInfo);
         IceUtil::Time tm2 = IceUtil::Time::now() - tm;
         debug("%d images saved in %lld ms", it->images.size(), tm2.toMilliSeconds());
      }
      toSave.clear();
   }

   debug("joining worker threads");
   tcQueue.join();
   tcDrawer.join();

   m_pTimer->destroy();
}

} // namespace
/* vim: set sw=3 ts=8 et: */