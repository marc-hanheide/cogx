/**
 * @author Michael Zillich (original file: VideoViewer)
 * @date February 2009
 *
 * Adapted by Marko Mahnič.
 */

#include <highgui.h>
#include <VideoUtils.h>
#include "TestComponent.hpp"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cogx::test::VideoViewer();
  }
}

// Convenience
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <ctime>

std::string sfloat(double f, int precision=6)
{
   std::ostringstream out;
   out << std::fixed << std::setprecision(precision) << f;
   return out.str();
}

long long gethrtime(void)
{
  struct timespec sp;
  int ret;
  long long v;
#ifdef CLOCK_MONOTONIC_HR
  ret=clock_gettime(CLOCK_MONOTONIC_HR, &sp);
#else
  ret=clock_gettime(CLOCK_MONOTONIC, &sp);
#endif
  if(ret!=0) return 0;
  v=1000000000LL; /* seconds->nanonseconds */
  v*=sp.tv_sec;
  v+=sp.tv_nsec;
  return v;
}

double fclocks()
{
   // clock() is not working correctly
   // return ( (double) clock()) / CLOCKS_PER_SEC;
   return 1e-9 * gethrtime();
}
// --------------------------


namespace cogx { namespace test {

using namespace std;
using namespace VisionData;

void VideoViewer::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> camId;
  }

#ifdef FEAT_VISUALIZATION
  m_display.configureDisplayClient(_config);
#endif

  // sanity checks: Have all important things be configured? Is the
  // configuration consistent?
  if(videoServerName.empty())
    throw runtime_error(exceptionMessage(__HERE__, "no video server name given"));
}

void VideoViewer::start()
{
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

  vector<int> camIds;
  camIds.push_back(camId);

#ifdef FEAT_VISUALIZATION
  m_bSendIplImage = false;
  m_display.connectIceClient(*this);
  m_display.installEventReceiver();
  //m_display.setEventCallback(this, &VideoViewer::handleGuiEvent);
  //m_display.setStateQueryCallback(this, &VideoViewer::getControlState);

  //m_display.addButton("toggle.viewer.running", "Start");
  m_display.addCheckBox(getComponentID(), "toggle.viewer.running", "&Streaming");
  m_display.addCheckBox(getComponentID(), "toggle.viewer.sendipl", "Send &IplImage");
  m_display.addButton(getComponentID(), "viewer.do.nothing", "Show &time");
  //m_display.addButton(getComponentID(), "viewer.do.nothing2", "&Test 2");
  //m_display.addButton(getComponentID(), "viewer.do.nothing3", "&Test 3");
#else
  cvNamedWindow(getComponentID().c_str(), 1);
#endif

  // start receiving images pushed by the video server
  receiving = false;
  if (receiving)
    videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);
}

#ifdef FEAT_VISUALIZATION
void VideoViewer::CVvDisplayClient::handleEvent(const Visualization::TEvent &event)
{
  debug(event.data + " (received by VideoViewer)");
  if (event.type == Visualization::evCheckBoxChange) {
    if (event.sourceId == "toggle.viewer.running") {
      debug(std::string("Time: ") + sfloat (fclocks()));
      bool newrcv = (event.data != "0");
      if (newrcv != pViewer->receiving) {
        if(pViewer->receiving) {
          pViewer->videoServer->stopReceiveImages(pViewer->getComponentID());
          pViewer->println("Stopped receiving images");
        }
        else {
          vector<int> camIds;
          camIds.push_back(pViewer->camId);
          pViewer->videoServer->startReceiveImages(pViewer->getComponentID(), camIds, 0, 0);
          pViewer->println("Started receiving images");
        }
        pViewer->receiving = !pViewer->receiving;
      }
    }
    else if (event.sourceId == "toggle.viewer.sendipl") {
      debug(std::string("Time: ") + sfloat (fclocks()));
      bool newrcv = (event.data != "0");
      if (pViewer->m_bSendIplImage != newrcv) {
        pViewer->m_bSendIplImage = newrcv;
        if(pViewer->m_bSendIplImage) {
          pViewer->println("Sendind images of type IplImage.");
        }
        else {
          pViewer->println("Sending images of type Video::Image.");
        }
      }
    }
  }
  else if (event.type == Visualization::evButtonClick) {
    if (event.sourceId == "viewer.do.nothing") {
      debug(std::string("Time: ") + sfloat (fclocks()));
      pViewer->println("The button works.");
    }
  }
}

std::string VideoViewer::CVvDisplayClient::getControlState(const std::string& ctrlId)
{
  if (ctrlId == "toggle.viewer.running") {
    return pViewer->receiving ? "1" : "0";
  }
  else if (ctrlId == "toggle.viewer.sendipl") {
    return pViewer->m_bSendIplImage ? "1" : "0";
  }
  return "";
}

void VideoViewer::CVvDisplayClient::handleForm(const std::string& id, const std::string& partId,
      const std::map<std::string, std::string>& fields)
{
  pViewer->println("Handle Form: %s#%s", id.c_str(), partId.c_str());
  pViewer->m_HtmlForm.apply(fields);
  std::vector<std::string> dump;
  pViewer->m_HtmlForm.dump(dump);
  typeof(dump.begin()) it;
  for(it = dump.begin(); it != dump.end(); it++) {
    pViewer->println("%s", it->c_str());
  }
}

bool VideoViewer::CVvDisplayClient::getFormData(const std::string& id, const std::string& partId,
      std::map<std::string, std::string>& fields)
{
  pViewer->println("Get Form: %s#%s", id.c_str(), partId.c_str());
  pViewer->m_HtmlForm.get(fields);
  return true;
}
#endif

void VideoViewer::destroy()
{
#ifndef FEAT_VISUALIZATION
  cvDestroyWindow(getComponentID().c_str());
#endif
}

void VideoViewer::receiveImages(const std::vector<Video::Image>& images)
{
#ifdef FEAT_VISUALIZATION
  if (! m_bSendIplImage)
    m_display.setImage(getComponentID(), images[0]);
  else {
    IplImage *iplImage = convertImageToIpl(images[0]);
    m_display.setImage(getComponentID(), iplImage);
    cvReleaseImage(&iplImage);
  }
  if (1) {
    static double iangl = 0;
    iangl += 10;
    if (iangl > 360) iangl = 0;
    double mdata[9];
    double scl = 0.5;
    double angl = iangl * 3.14 / 180;
    mdata[0] = scl*cos(angl);  mdata[1] = scl*sin(angl); mdata[2] = 0;
    mdata[3] = -scl*sin(angl); mdata[4] = scl*cos(angl); mdata[5] = 0;
    mdata[6] = 100;            mdata[7] = 100;           mdata[8] = 1;
    CvMat mat = cvMat(3, 3, CV_64FC1, mdata);
    m_display.setObjectTransform2D("Visualization.test.SVG", "little-lion", &mat);
  }
  {
    std::stringstream str;
    str << "The current HR time is <span class='time'>" << sfloat (fclocks()) << "s</span></br>";
    m_display.setHtml("@info.TestComponent", "time", str.str());
    std::stringstream css;
    css << "<style type='text/css'>\n";
    css << ".time {color: ";
    switch( ((long long)fclocks()) % 3 ) {
      case 0: css << "red"; break;
      case 1: css << "green"; break;
      case 2: css << "blue"; break;
    }
    css << ";}\n</style>\n";
    m_display.setHtmlHead("@info.TestComponent", "time", css.str());
  }
#else
  IplImage *iplImage = convertImageToIpl(images[0]);
  cvShowImage(getComponentID().c_str(), iplImage);
  cvReleaseImage(&iplImage);
#endif
}

void VideoViewer::runComponent()
{
  sleepComponent(1000);

#ifdef FEAT_VISUALIZATION
  {
    std::ifstream infile;
    infile.open("subarchitectures/visualization.sa/config/test/images/lion.svg", std::ifstream::in);
    std::stringstream str;
    str << infile.rdbuf();
    infile.close();
    m_display.setObject("Visualization.test.SVG", "lion", str.str());
    m_display.setObject("Visualization.test.SVG", "little-lion", str.str());
  }
  {
    std::stringstream str;
    str << "function initlists()\n";
    str <<   "dirty = DispList:getDirty({'myobject'})\n";
    str <<   "if not dirty['myobject'] then return end\n";
    str <<   "DispList:newList('myobject')\n";
    str <<     "glBegin(GL_QUADS)\n";
    str <<     "glVertex(0.1, 0.1, 0.1)\n";
    str <<     "glVertex(0.3, 0.1, 0.2)\n";
    str <<     "glVertex(0.3, 0.4, 0.3)\n";
    str <<     "glVertex(0.0, 0.5, 0.4)\n";
    str <<     "glEnd()\n";
    str <<   "DispList:endList()\n";
    str << "end\n";

    str << "function render()\n";
    str << "initlists()\n";
    str << "N=36\n";
    str << "tabsin={0";
    for (int i = 0; i < 36; i++) str << "," << sin(10*i*3.14/180);
    str << "}\n";
    str << "tabcos={1.0";
    for (int i = 0; i < 36; i++) str << "," << cos(10*i*3.14/180);
    str << "}\n";
    str <<
         //"glPointSize(2)\n"
         "glBegin(GL_LINES)\n"
         "for i=1,10000 do\n"
            "glColor(i%23/23.0,i%17/17.0,i%11/11.0)\n"
            "glVertex(tabcos[i%N+1], tabsin[i%N+1], -i/100.0)\n"
         "end\n"
         "glEnd()\n"
         "glColor(0.5,0.1,0.1)\n"
         "DispList:draw('myobject')\n"
         "glPushMatrix()\n"
           "glColor(0.1,0.7,0.4)\n"
           "glTranslate(0.5,0.5,0.0)\n"
           "DispList:draw('myobject')\n"
         "glPopMatrix()\n"
      "end\n";
    m_display.setLuaGlObject("Visualization.sa.LuaGL", "Points", str.str());
  }
  {
    std::ifstream infile;
    infile.open("subarchitectures/visualization.sa/src/c++/core/object/gllua/test/pusher.luagl", std::ifstream::in);
    std::stringstream str;
    str << infile.rdbuf();
    infile.close();
    m_display.setLuaGlObject("Visualization.test.Pusher", "Pusher", str.str());
  }
  {
    std::stringstream strA;
    strA << "This is the TestComponent for the Display Server<br>";
    m_display.setHtml("@info.TestComponent", "text", strA.str());
  }
  if (1) {
    std::stringstream strB;
    clock_t a, b;
    a = clock();
    sleepComponent(1000);
    b = clock();
    strB << "Clock a=" << a << " b=" << b << " diff=" << b-a
        << " CLOCKS_PER_SEC=" << CLOCKS_PER_SEC << "<br>" << endl;
    double fa, fb;
    fa = fclocks();
    sleepComponent(1000);
    fb = fclocks();
    strB << "FClocks fa=" << fa << " fb=" << fb << " diff=" << fb-fa << "<br>" << endl;
    m_display.setHtml("@info.TestComponent", "zclock_test", strB.str());
  }
#ifdef V11N_OBJECT_HTML_PLUGINS
  {
    // XXX: This is not working because libflashplugin.so crashes.
    // Remove nspluginwrapper and it's better ... except no flash in firefox :(
    std::stringstream str;
    str << "This is the TestComponent for the Display Server<br>";
    str << "<hr>";
    str << "<object type='application/cast-displayview' data='cogxdisp://view/"
        << getComponentID() << "'></object>";
    str << "<hr>";
    str << "<object type='application/cast-displayview' data='cogxdisp://view/Visualization.test.SVG'></object>";
    str << "<hr>";
    str << "<object type='application/cast-displayview' data='cogxdisp://view/Visualization.test.Pusher'></object>";
    str << "<hr>";
    str << "<object type='application/cast-displayview' data='cogxdisp://view/Visualization.sa.LuaGL'></object>";
    str << "<hr>";
    // Recursive WebKit ...
    str << "<object type='application/cast-displayview' data='cogxdisp://view/@info.DisplayServer'></object>";
    str << "<hr>";
    str << "<object type='application/cast-displayview' data='cogxdisp://view/@info.TestComponent'></object>";
    str << "<hr>";
    m_display.setHtml("Visualization.test.HtmlPlugin", "text", str.str());
  }
#endif
  {
    using namespace cogx::display;
    // Forms
    std::stringstream str;
    str << "This is the TestComponent for the Display Server<br>";
    str << "<hr>";
    str << "<select name=\"single\"> <option>Single</option> <option>Single2</option> </select>";
    m_HtmlForm.add(new CFormValues::choice("single", CFormValues::valuelist() << "Single" << "Single2"));

    str << "<select name=\"multiple\" multiple=\"multiple\"><option selected=\"selected\">Multiple</option>";
    str << "<option>Multiple2</option> <option selected=\"selected\">Multiple3</option>";
    str << "</select><br/>";
    m_HtmlForm.add(new CFormValues::set("multiple",
        CFormValues::valuelist() << "Multiple" << "Multiple2" << "Multiple3"));

    str << "<input type=\"checkbox\" name=\"checkboxname\" value=\"check1\"/> CheckBox1";
    str << "<input type=\"checkbox\" name=\"checkboxname\" value=\"check2\"/> CheckBox2";
    m_HtmlForm.add(new CFormValues::set("checkboxname", CFormValues::valuelist() << "check1" << "check2"));

    str << "<input type=\"radio\"  name=\"rbname\" value=\"radio1\"/> RadioButton1";
    str << "<input type=\"radio\"  name=\"rbname\" value=\"radio2\"/> RadioButton2";
    m_HtmlForm.add(new CFormValues::choice("rbname", CFormValues::valuelist() << "radio1" << "radio2"));

    str << "<br/>";
    str << "<input type=\"text\" name=\"val\" value=\"Display Server Rules!\" />";
    m_HtmlForm.add(new CFormValues::field("val"));

    str << "<input type=\"submit\" name=\"submit\" value=\"Apply\"/>";
    m_display.setHtmlForm("Visualization.test.HtmlForm", "001_text", str.str());
  }
  if (0) {
    // Test if the form could be restored
    std::stringstream str;
    str << "<script>";
    str << "var vals = {'single': ['Single2'], 'val': 'This also works: Display Server Rules!', ";
    str << "   'multiple': ['Multiple2', 'Multiple3']};";
    str << "CogxJsFillFormV('#form_Visualization_test_HtmlForm_001_text', vals);";
    str << "</script>";
    m_display.setHtml("Visualization.test.HtmlForm", "999_test", str.str());
  }

  int count = 0;
  int boxrot = 0;
  srand ( time(NULL) );
#else
  println("press <s> to stop/start receving images");
#endif

  while(isRunning())
  {
    // needed to make the window appear
    // (an odd behaviour of OpenCV windows!)
#ifdef FEAT_VISUALIZATION
    sleepComponent(100);
    std::stringstream str;
    count++;
    if (count > 100) count = 0;
    if (count % 2 == 0) {
      int dir = rand() % 4;
      switch (dir) {
        case 0: str << "move(1,0)" << endl; break;
        case 1: str << "move(0,1)" << endl; break;
        case 2: str << "move(-1,0)" << endl; break;
        case 3: str << "move(0,-1)" << endl; break;
      }
    }
    boxrot = (boxrot++) % 36;
    str << "boxTurn=" << (boxrot * 10) << endl << "DispList:setDirty('pusher.box.rotation')" << endl;
    if (str.tellp() > 0) {
      m_display.setLuaGlObject("Visualization.test.Pusher", "Pusher", str.str());
    }
#else
    int key = cvWaitKey(100);
    switch(key)
    {
      case 's':  // start/stop getting images
        if(receiving)
        {
          videoServer->stopReceiveImages(getComponentID().c_str());
          println("stopped receving images");
        }
        else
        {
          vector<int> camIds;
          camIds.push_back(camId);
          videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);
          println("started receving images");
        }
        receiving = !receiving;
        break;
      default:
        break;
    }
#endif // FEAT_VISUALIZATION
  }
}

}} // namespace

