/**
 * @author Marko Mahnič
 * @created September 2010
 */
#include "Grabber.h"
#include "StringFmt.h"

#include <cstdio>
#include <sys/stat.h>
#include <sys/types.h>

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cogxgrabber::CGrabber();
  }
}

namespace cogxgrabber
{

namespace cxd = cogx::display;

#define IDOBJ_GRABBER "Video.Grabber"
#define IDOBJ_SETTINGS "Video.Grabber.Settings"

#define IDPART_SETTINGS_FORM "001.settings"
#define IDPART_SETTINGS_VALUES "002.values"
#define IDCMD_GRAB   "video.grab"
#define IDCMD_RECORD "video.record"
#define IDCMD_STOP   "video.stop"
#define IDCTRL_STREAMING "video.streaming"

CGrabber::CGrabber()
{
  m_frameGrabMs = 200;
  m_deviceNames = "L R";
#ifdef FEAT_VISUALIZATION
  m_pDisplayCanvas = NULL;
  m_display.setClientData(this);
#endif
}

CGrabber::~CGrabber()
{
  m_fakeRecording = false;
#ifdef FEAT_VISUALIZATION
  releaseCanvas();
#endif
  for(unsigned int i = 0; i < m_video.size(); i++) {
    Video::CVideoClient2* pVideo = m_video[i];
    pVideo->setReceiving(false);
    pVideo->disconnect();
    delete pVideo;
  }

#ifdef FEAT_VIDEOGRABBER_POINTCLOUD
  CPcGrabClient *pPc;
  for(unsigned int i = 0; i < m_pointcloud.size(); i++) {
    pPc = m_pointcloud[i];
    delete pPc;
  }
#endif
}

bool parseBool(const std::string& value, bool valEmpty=false, bool valInvalid=false)
{
  std::string s = _s_::lower(_s_::strip(value));
  if (s == "")
    return valEmpty;
  if (s == "1" || s == "yes" || s == "true" || s == "on")
    return true;
  if (s == "0" || s == "no" || s == "false" || s == "off")
    return false;
  return valInvalid;
}

void CGrabber::configure(const std::map<std::string,std::string> & _config)
{
  std::map<std::string,std::string>::const_iterator it, iCam, iopt;

  std::string serverName;
  std::vector<int> camids;
  int id;

  // Video servers
  for (it = _config.begin(); it != _config.end(); it++) {
    std::string sffx = "";

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
      std::istringstream str(iCam->second);
      while(str >> id) camids.push_back(id);
    }

    if (camids.size() < 1) {
      throw std::runtime_error(cast::exceptionMessage(__HERE__, 
            "Grabber id='%s': No --camids%s for VideoServer id='%s'.",
            getComponentID().c_str(), sffx.c_str(), serverName.c_str()));
    }

    CVideoGrabClient* pVideo = new CVideoGrabClient(this);
    pVideo->setServer(this, serverName, camids);
    pVideo->setReceiver(new Video::CReceiverMethod<CGrabber>(this, &CGrabber::receiveImages));
    m_video.push_back(pVideo);
  }

#ifdef FEAT_VIDEOGRABBER_POINTCLOUD
  CPcGrabClient *pPc;
  // Pointcloud servers
  for (it = _config.begin(); it != _config.end(); it++) {
    if (it->first != "--pcserver" && !_s_::startswith(it->first, "--pcserver."))
      continue;

    serverName = _s_::strip(it->second);

    if (serverName == "") continue;

    std::string sffx = "";
    if (_s_::startswith(it->first, "--pcserver.")){
      sffx = it->first.substr(10);
    }

    std::map<std::string,std::string> conf;
    conf["--pcserver"] = serverName;
    pPc = new CPcGrabClient();

    iopt = _config.find("--pcgrabpoints" + sffx);
    if (iopt != _config.end()) {
      pPc->mbGrabPoints = parseBool(iopt->second, /*empty=*/ true);
    }
    iopt = _config.find("--pcgrabdepth" + sffx);
    if (iopt != _config.end()) {
      pPc->mbGrabDepth = parseBool(iopt->second, /*empty=*/ true);
    }
    iopt = _config.find("--pcgrabrect" + sffx);
    if (iopt != _config.end()) {
      pPc->mbGrabRectImage = parseBool(iopt->second, /*empty=*/ true);
    }

    pPc->configurePcComm(conf);
    m_pointcloud.push_back(pPc);
  }
#endif

  if((it = _config.find("--fakegrabbing")) != _config.end()) {
    m_fakeRecording = parseBool(it->second, /*empty=*/ true);
  }

  if((it = _config.find("--grab_ms")) != _config.end()) {
    std::istringstream str(it->second);
    str >> m_frameGrabMs;
  }
  if (m_frameGrabMs < 10) m_frameGrabMs = 10;
  if((it = _config.find("--devicenames")) != _config.end()) {
    m_deviceNames = it->second;
  }

#ifdef FEAT_VISUALIZATION
  m_display.configureDisplayClient(_config);
#endif

  if(m_video.size() < 1)
    throw std::runtime_error(cast::exceptionMessage(__HERE__, "No video server name given"));
}

void CGrabber::start()
{
#ifdef FEAT_VISUALIZATION
  m_displayTimer.restart();
  m_display.connectIceClient(*this);
  m_display.installEventReceiver();
  m_display.createForms();

  // TODO: Server Bug: only one (the first) checkbox control is shown (same id)...
  m_display.addCheckBox(IDOBJ_GRABBER, IDCTRL_STREAMING, "Streaming");
  m_display.addButton(IDOBJ_GRABBER, IDCMD_GRAB, "&Grab");
  m_display.addButton(IDOBJ_GRABBER, IDCMD_RECORD, "&Record");
  m_display.addButton(IDOBJ_GRABBER, IDCMD_STOP, "&Stop Recording");
  m_display.addCheckBox(IDOBJ_SETTINGS, IDCTRL_STREAMING, "Streaming");

  m_display.updateDisplay();
#endif

  Video::CVideoClient2* pVideo;
#if 0 // XXX: code for testing
  pVideo = new Video::CVideoClient2();
  pVideo->setServer(this, m_videoServerName, m_camIds);
  pVideo->setReceiver(new Video::CReceiverMethod<CGrabber>(this, &CGrabber::receiveImages));
  m_video.push_back(pVideo);

  std::vector<int> camIds;
  camIds.push_back(0);
  pVideo = new Video::CVideoClient2();
  pVideo->setServer(this, "VideoServer2", camIds);
  pVideo->setReceiver(new Video::CReceiverMethod<CGrabber>(this, &CGrabber::receiveImages));
  m_video.push_back(pVideo);

  pVideo = new Video::CVideoClient2();
  pVideo->setServer(this, "VideoServer3", camIds);
  pVideo->setReceiver(new Video::CReceiverMethod<CGrabber>(this, &CGrabber::receiveImages));
  m_video.push_back(pVideo);
#endif

  for(unsigned int i = 0; i < m_video.size(); i++) {
    pVideo = m_video[i];
    pVideo->setCaching(true);
    pVideo->connect();
    pVideo->setReceiving(true);
  }

#ifdef FEAT_VIDEOGRABBER_POINTCLOUD
  CPcGrabClient *pPc;
  for(unsigned int i = 0; i < m_pointcloud.size(); i++) {
    pPc = m_pointcloud[i];
    pPc->startPcComm(*this);
#ifdef FEAT_VISUALIZATION
    pPc->configExtraV11n(m_display);
#endif
  }
#endif
}

#ifdef FEAT_VISUALIZATION
void CGrabber::CVvDisplayClient::createForms()
{
  std::string head =
    "<style>"
    "table.formrow { margin-top: 0px; margin-bottom: 0px; }\n"
    "table.formrow td { padding: 0px;}\n"
    "</style>"
    ;
  setHtmlHead(IDOBJ_SETTINGS, IDPART_SETTINGS_FORM, head);
  std::ostringstream ss;
  std::string help1, help2;

  ss <<
    "<table class='formrow'>"
    "<tr><td>Model: </td><td>"
    "<input type='text' name='model' style='width:20em;' />"
    "</td></tr>"
    "</table>";
  m_frmSettings.add(new cxd::CFormValues::field("model"));

  help1 = "Use \"%m\" to place the model name in the Directory.";
  help2 = "\"Create directory\" can only create one (the last) level.";
  ss <<
    "<table class='formrow'>"
    "<tr><td>Directory: </td><td>"
    "<input type='text' name='directory' style='width:20em;' title='" << help1 << "'/>"
    "</td></tr><tr><td>&nbsp;</td><td>"
    "<input type='checkbox' name='createdir' value='on' title='" << help2 << "'/> Create directory"
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
    "<table class='formrow'>"
    "<tr><td>Filename: </td><td>"
    "<input type='text' name='filename' style='width:20em;' title='" << help1 << "'/>"
    "</td></tr>"
    "</table>";
  m_frmSettings.add(new cxd::CFormValues::field("filename"));

  ss << "<table class='formrow'>";
  std::vector<std::string> cntValues;
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
    "<table class='formrow'>"
    "<tr><td>Device names: </td><td>"
    "<input type='text' name='devicenames' style='width:20em;' title='" << help1 << "' />"
    "</td></tr>"
    "</table>";
  m_frmSettings.add(new cxd::CFormValues::field("devicenames"));

  help1 = "Stop recording after limited time. Enter 0 or less for no limit.";
  help2 = "Grabbing speed in frames per second.";
  ss <<
    "<table class='formrow'>"
    "<tr><td>Recording limit: </td><td>"
    "<input type='text' name='recordtimelimit' style='width:5em;' title='" << help1 << "' /> ms"
    "</td><td>&nbsp;&nbsp;&nbsp; Speed: </td><td>"
    "<input type='text' name='recordspeed' style='width:5em;' title='" << help2 << "' /> fps"
    "</td></tr>"
    "</table>";
  m_frmSettings.add(new cxd::CFormValues::field("recordtimelimit"));
  m_frmSettings.add(new cxd::CFormValues::field("recordspeed"));

  ss << "<input type='submit' value='Apply' />";

  ss << 
    "<div class='v11ninfo'>"
    "If you change anything, don't forget to 'Apply'.<br>"
    "You can 'Save' current values for later use.<br>"
    "After you 'Load' the saved values, press 'Apply'.<br>"
    "More help is available in the tooltips."
    "</div>";

#if 0 // this makes DisplayServer crash because of mozilla plugins
  ss << R"(
    </td><td>
    <object type='application/cast-displayview' data='cogxdisp://view/Video.Grabber'
    width='200' height='400'>
    </object>
    </td></tr>
    )";
#endif

  // Set form defaults
  setDirectory("xdata/grab/%m");
  setImageFilenamePatt("frame-%c-%d");
  setDeviceNames(pViewer->m_deviceNames);
  setCounterDigits(5);
  setCounterValue(0);
  setRecordTimeLimit(5000);
  setGrabbingSpeed(pViewer->m_frameGrabMs >= 10 ? 1e+3 / pViewer->m_frameGrabMs : 5);

  setHtmlForm(IDOBJ_SETTINGS, IDPART_SETTINGS_FORM, ss.str());
  showCurrentSettings();
}

void CGrabber::CVvDisplayClient::showCurrentSettings()
{
  std::ostringstream ss;
  ss << "<hr>"
    << "<table><tr><td>";

  ss << "<table>"
    << "<tr><td>"
    << "<input type='button' value='Grab' @@ONCLICK@@('" << IDCMD_GRAB << "') />\n"
    << "</td></tr><tr><td>"
    << "<input type='button' value='Record' @@ONCLICK@@('" << IDCMD_RECORD << "') />\n"
    << "</td></tr><tr><td>"
    << "<input type='button' value='Stop Recording' @@ONCLICK@@('" << IDCMD_STOP << "') />\n"
    << "</td></tr>"
    << "</table>";

  ss << "</td><td>";

  std::string status;
  if (pViewer->isGrabbing()) {
    status = "<span style='color:red;'>&nbsp;Recording&nbsp;</span>";
  }
  else if (pViewer->isSaving()) {
    status = "<span style='color:red;'>&nbsp;Saving. "
      + _str_(pViewer->saveQueueSize())
      + " items to save</span>";
  }
  else {
    status = "Idle";
  }

  std::string missed = ( pViewer->m_RecordingInfo.missedTicks
      ? " <span style='color:red;'>Skipped: " + _str_(pViewer->m_RecordingInfo.missedTicks) + "</span>"
      : "");

  std::string maxframes;
  if (getRecordTimeLimit() > 0 && getGrabbingSpeed() > 0) {
    maxframes = "&nbsp;&nbsp;&nbsp; Approx. " 
      + _str_((int)(getRecordTimeLimit() * getGrabbingSpeed() / 1e3))
      + " frames";
  }


  ss <<
    "Current settings:"
    "<div>"
    "Saving to: " << getDirectory() << "/" << getImageFilenamePatt() << "<br>"
    "Next counter value: "
    << (pViewer->isGrabbing()
        ? "<span style='color:red;'>"
        : "<span>"
       )
    << getCounterValue() << "</span>"
    << missed
    << "<br>"
    "Time limit: " << getRecordTimeLimit() << "ms" << maxframes << "<br>"
    "Grabbing speed: " << (int)(1e3 / getGrabbingSpeed()) << "ms, "
    << _str_(getGrabbingSpeed(), 0, 3, ' ') << "fps<br>"
    << "Status: " << status
    << "</div>";

  ss << "</td></tr></table>";

  setActiveHtml(IDOBJ_SETTINGS, IDPART_SETTINGS_VALUES, ss.str());
}

std::string CGrabber::CVvDisplayClient::getDirectory()
{
  std::string dir = _s_::strip(m_frmSettings.get("directory"));
  std::string model = _s_::strip(m_frmSettings.get("model"));
  _s_::replace(dir, "%m", model);
  return _s_::rstrip(dir, "/ ");
}

std::string CGrabber::CVvDisplayClient::getModelName()
{
  std::string model = _s_::strip(m_frmSettings.get("model"));
  return model;
}

void CGrabber::CVvDisplayClient::setDirectory(const std::string& dirname)
{
  m_frmSettings.setValue("directory", dirname);
}

bool CGrabber::CVvDisplayClient::getCreateDirectory()
{
  return _s_::strip(m_frmSettings.get("createdir")) == "on";
}

std::string CGrabber::CVvDisplayClient::getImageFilenamePatt()
{
  return _s_::strip(m_frmSettings.get("filename"));
}

void CGrabber::CVvDisplayClient::setImageFilenamePatt(const std::string& pattern)
{
  m_frmSettings.setValue("filename", pattern);
}

std::string CGrabber::CVvDisplayClient::getDeviceNames()
{
  return _s_::strip(m_frmSettings.get("devicenames"));
}

void CGrabber::CVvDisplayClient::setDeviceNames(const std::string& names)
{
  m_frmSettings.setValue("devicenames", names);
}

long CGrabber::CVvDisplayClient::getCounterDigits()
{
  // TODO: countersize is a set so getInt doesn't work ... 
  return atoi(m_frmSettings.get("countersize").c_str());
}

void CGrabber::CVvDisplayClient::setCounterDigits(long nDigits)
{
  m_frmSettings.setValue("countersize", _str_(nDigits));
}

long CGrabber::CVvDisplayClient::getCounterValue()
{
  return m_frmSettings.getInt("countervalue");
}

void CGrabber::CVvDisplayClient::setCounterValue(long nValue)
{
  m_frmSettings.setValue("countervalue", _str_(nValue));
}

long CGrabber::CVvDisplayClient::getRecordTimeLimit()
{
  return m_frmSettings.getInt("recordtimelimit");
}

void CGrabber::CVvDisplayClient::setRecordTimeLimit(long nValue)
{
  m_frmSettings.setValue("recordtimelimit", _str_(nValue));
}

float CGrabber::CVvDisplayClient::getGrabbingSpeed()
{
  return m_frmSettings.getFloat("recordspeed");
}

void CGrabber::CVvDisplayClient::setGrabbingSpeed(float fValue)
{
  m_frmSettings.setValue("recordspeed", _str_(fValue, 2));
}

void CGrabber::CVvDisplayClient::handleEvent(const Visualization::TEvent &event)
{
  //debug(event.data + " (received by VideoViewer)");
  if (event.type == Visualization::evCheckBoxChange) {
    if (event.sourceId == IDCTRL_STREAMING) {
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
  else if (event.type == Visualization::evHtmlOnClick) {
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

void CGrabber::CVvDisplayClient::updateDisplay()
{
  std::map<std::string, std::string> fields;
  m_frmSettings.get(fields);
  setHtmlFormData(IDOBJ_SETTINGS, IDPART_SETTINGS_FORM, fields);
  showCurrentSettings();
}

std::string CGrabber::CVvDisplayClient::getControlState(const std::string& ctrlId)
{
  if (ctrlId == IDCTRL_STREAMING) {
    return pViewer->m_bReceiving ? "1" : "0";
  }
  return "";
}

void CGrabber::CVvDisplayClient::handleForm(const std::string& id,
    const std::string& partId, const std::map<std::string, std::string>& fields)
{
  std::cout << " *** handleForm " << partId << std::endl;
  if (partId == IDPART_SETTINGS_FORM) {
    m_frmSettings.apply(fields);
    double frameGrabMs = 1e3 / getGrabbingSpeed();
    if (frameGrabMs < 10) {
      setGrabbingSpeed(1e3 / 10);
      updateDisplay();
    }
    else
      showCurrentSettings();
  }
}

bool CGrabber::CVvDisplayClient::getFormData(const std::string& id,
    const std::string& partId, std::map<std::string, std::string>& fields)
{
  if (partId == IDPART_SETTINGS_FORM) {
    m_frmSettings.get(fields);
    return true;
  }
  return false;
}

void CGrabber::releaseCanvas()
{
  if (m_pDisplayCanvas) {
    // HACK: Share data between IplImage and vector
    m_pDisplayCanvas->imageData = NULL;
    m_pDisplayCanvas->imageDataOrigin = NULL;
    cvReleaseImage(&m_pDisplayCanvas);
  }
}

void CGrabber::prepareCanvas(int width, int height)
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

#endif

void CGrabber::destroy()
{
}

void CGrabber::sendCachedImages()
{
#ifdef FEAT_VISUALIZATION
  bool bGrabbing = isGrabbing();
  long frameMillis = bGrabbing ? 1780 : 312;
  if (m_displayTimer.elapsed() < frameMillis) {
    return;
  }
  m_displayTimer.restart();

  std::vector<CPreview> previews;
  std::vector<CDataSource*> clients;
  getClients(clients);
  for (auto pClient : clients){
    pClient->getPreviews(previews, 320, 240, bGrabbing);
  }
  int w = 0, h = 0;
  for (auto prv : previews) {
    IplImage* pImg = prv.getImage();
    if (w < pImg->width) {
      w = pImg->width;
    }
    h += pImg->height;
  }

  prepareCanvas(w, h);
  IplImage *pDisp = m_pDisplayCanvas;
  CvFont fntSimplex, fntPlain;
  cvInitFont(&fntSimplex, CV_FONT_HERSHEY_SIMPLEX, 0.7, 0.8, 0, 1.6);
  cvInitFont(&fntPlain, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0, 1.5);
  std::vector<std::string> devnames = getDeviceNames();

  int vp = 0;
  int i = 0;
  for (auto prv : previews) {
    int factor = 1;
    IplImage* pImage = prv.getImage();
    cvResetImageROI(pImage);
    int wi = (int) (pImage->width * factor);
    int hi = (int) (pImage->height * factor);
    cvSetImageROI(pDisp, cvRect(0, vp, wi, hi));
    cvResize(pImage, pDisp);

    std::string sMsg = _str_(i) + " " + prv.deviceName;
    if (i < devnames.size()) sMsg += ": " + devnames[i];
    cvPutText (pDisp, sMsg.c_str(), cvPoint(10, 25), &fntSimplex, cvScalar(255,255,0));

    sMsg = prv.deviceInfo;
    cvPutText (pDisp, sMsg.c_str(), cvPoint(10, hi-2), &fntPlain, cvScalar(255,255,0));

    cvResetImageROI(pDisp);
    vp += hi;
    ++i;
  }
  m_display.setImage(IDOBJ_GRABBER, w, h, 3, m_DisplayBuffer);

  for (auto pClient : clients){
    pClient->displayExtra(m_display);
  }
#endif

}

void CGrabber::receiveImages(const std::string& serverName, const std::vector<Video::Image>& _images)
{
}

std::vector<std::string> CGrabber::getDeviceNames()
{
  std::vector<std::string> devnames = _s_::split(m_display.getDeviceNames(), " ", 0, false);
  // TODO: add missing device names, form m_video
  //while (devnames.size() < m_camIds.size()) {
  //   int i = m_camIds[devnames.size()];
  //   devnames.push_back(_str_(i));
  //}
  return devnames;
}

void CGrabber::fillRecordingInfo(CRecordingInfo &info)
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
  info.missedTicks = 0;

  long reclimit = m_display.getRecordTimeLimit();
  info.tmStart = IceUtil::Time::now();
  info.tmEnd = info.tmStart + IceUtil::Time::milliSeconds(reclimit);

  info.deviceNames = getDeviceNames();
}

void CGrabber::startGrabbing(const std::string& command)
{
  if (isGrabbing()) return;
  fillRecordingInfo(m_RecordingInfo);
  if (command == IDCMD_GRAB) {
    // On single-shot grabbing don't install the timer, just grab right now
    dynamic_cast<CGrabQueThread*>(m_pQueue.get())->grab();
    m_display.updateDisplay(); // send counter value
  }
  else if (command == IDCMD_RECORD) {
    m_RecordingInfo.recording = true;
    m_pTimer->scheduleRepeated(m_pQueueTick.get(),
        IceUtil::Time::milliSeconds(1e3 / m_display.getGrabbingSpeed()));
  }
}

void CGrabber::stopGrabbing()
{
  if (!isGrabbing()) return;
  m_pTimer->cancel(m_pQueueTick.get());
  m_RecordingInfo.recording = false;
  sleepComponent(100);
  m_display.updateDisplay();
}

void CGrabber::checkStopGrabbing()
{
  CRecordingInfo& ri = m_RecordingInfo;
  if (ri.counterEnd > ri.counterStart && ri.counter >= ri.counterEnd) stopGrabbing();
  else if (ri.tmEnd > ri.tmStart && IceUtil::Time::now() >= ri.tmEnd) stopGrabbing();
}

void CGrabber::saveQueuedImages(const std::vector<CGrabbedItemPtr>& images,
    CRecordingInfo& frameInfo)
{
  if (frameInfo.directoryStatus == 0 || frameInfo.directoryStatus == 1) {
    struct stat finfo;
    std::string& dir = frameInfo.directory;
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

  int devid = 0;
  for (auto pitem : images) {
    castutils::CMilliTimer tm(true);
    CExtraSaverPtr pSaver = pitem->save(frameInfo, devid);
    //log(" *** %s ::save took %ldms", pitem->mName.c_str(), tm.elapsed());

    if (pSaver.get()) {
      dynamic_cast<CExtraSaveThread*>(m_pExtraSaver.get())->addSaver(pSaver);
    }
    devid += pitem->numDevices();
    if (isGrabbing()) {
      sleepComponent(5);
      //sleepComponent(500); // XXX: DEBUGGING, simulating a slow save
    }
  }
}

CGrabber::CGrabQueThread::CGrabQueThread(CGrabber *pGrabber)
{
  m_pGrabber = pGrabber;
}

void CGrabber::CGrabQueThread::getItems(
    std::vector<CGrabber::CGrabQueThread::CFramePack>& items,
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

void CGrabber::CGrabQueThread::grab()
{
  if (! m_pGrabber) return;

  CFramePack pack;
  std::vector<CDataSource*> clients;
  m_pGrabber->getClients(clients);
  for (auto psource : clients) {
    std::vector<CGrabbedItemPtr> timgs;

    castutils::CMilliTimer tm(true);
    psource->grab(timgs);
    //m_pGrabber->log(" *** %s ::grab took %ldms", psource->mName.c_str(), tm.elapsed());

    for(auto itt = timgs.begin(); itt != timgs.end(); itt++) {
      pack.images.push_back(*itt);
    }
  }
  pack.frameInfo = m_pGrabber->m_RecordingInfo;
  {
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_itemsLock);
    m_items.push_back(pack);
  }

  m_pGrabber->m_RecordingInfo.counter++;
  m_pGrabber->m_display.setCounterValue(m_pGrabber->m_RecordingInfo.counter);
}

void CGrabber::CGrabQueThread::run()
{  
  while (m_pGrabber && m_pGrabber->isRunning()) {
    int ticks = waitForTick(500);
    if (!ticks) continue;
    if (!m_pGrabber->isGrabbing()) continue;
    if (ticks > 1) {
      m_pGrabber->println(std::string(" *** CGrabQueThread: missed ticks: ") + _str_(ticks-1));
      m_pGrabber->m_RecordingInfo.missedTicks += (ticks - 1);
    }

    grab();
    m_pGrabber->checkStopGrabbing();
  }
}

CGrabber::CExtraSaveThread::CExtraSaveThread(CGrabber *pGrabber)
{
  m_pGrabber = pGrabber;
}

void CGrabber::CExtraSaveThread::run()
{
  while (m_pGrabber && m_pGrabber->isRunning()) {
    if (m_savers.empty()) {
      getThreadControl().sleep(IceUtil::Time::milliSeconds(20));
      continue;
    }
    CExtraSaverPtr saver = next();
    if (! saver.get()) {
      continue;
    }
    saver->save();
    //m_pGrabber->log(" *** Saved extra: %s", saver->finalFilename.c_str());
    if (m_pGrabber->isGrabbing()) {
      getThreadControl().sleep(IceUtil::Time::milliSeconds(10));
    }
  }
}

CGrabber::CDrawingThread::CDrawingThread(CGrabber *pGrabber)
{
  m_pGrabber = pGrabber;
}

void CGrabber::CDrawingThread::run()
{  
  castutils::CMilliTimer tm(true);
  bool wasSaving = false;
  while (m_pGrabber && m_pGrabber->isRunning()) {
    int ticks = waitForTick(500);
    if (!ticks) continue;

    m_pGrabber->sendCachedImages();
    if (tm.elapsed() > 1230 && (m_pGrabber->isGrabbing() || m_pGrabber->isSaving() || wasSaving)) {
      tm.restart();
      wasSaving = m_pGrabber->isSaving();
      m_pGrabber->m_display.showCurrentSettings();
    }
  }
}

void CGrabber::runComponent()
{
  sleepComponent(1000);
  m_pQueue = new CGrabQueThread(this);
  m_pDrawer = new CDrawingThread(this);
  m_pExtraSaver = new CExtraSaveThread(this);

  m_pQueueTick = new CTickerTask(dynamic_cast<CTickSyncedTask*>(m_pQueue.get()));
  m_pDrawTick  = new CTickerTask(dynamic_cast<CTickSyncedTask*>(m_pDrawer.get()));

  IceUtil::ThreadControl tcQueue = m_pQueue->start();
  IceUtil::ThreadControl tcDrawer = m_pDrawer->start();
  IceUtil::ThreadControl tcSaver = m_pExtraSaver->start();

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
    std::vector<CGrabQueThread::CFramePack> toSave;
    dynamic_cast<CGrabQueThread*>(m_pQueue.get())->getItems(toSave, 4);
    if (toSave.size() < 1) {
      sleepComponent(20);
      continue;
    }
    debug("Will save %d frames", toSave.size());
    for (auto it = toSave.begin(); it != toSave.end(); it++) {
      IceUtil::Time tm = IceUtil::Time::now();
      saveQueuedImages(it->images, it->frameInfo);
      IceUtil::Time tm2 = IceUtil::Time::now() - tm;
      debug("%d items saved in %lld ms", it->images.size(), tm2.toMilliSeconds());
    }
    toSave.clear();
  }

  debug("joining worker threads");
  tcQueue.join();
  tcDrawer.join();
  tcSaver.join();

  m_pTimer->destroy();
}

} // namespace
/* vim: set sw=2 ts=8 et: */
