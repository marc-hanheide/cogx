/**
 * @date February 2009
 *
 * @author Marko Mahnič.
 * Adapted from VideoViewer component created by Michael Zillich
 */

#include <highgui.h>
#include <VideoUtils.h>
#include <convenience.hpp>
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

#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <ctime>
#ifdef HAS_LIBPLOT
//#include <plotter.h> // libplot-dev
#include <CSvgPlotter.hpp>
#endif

namespace cogx { namespace test {

using namespace std;

#include "res/test_calc.inc"

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

  m_display.setActiveHtml("TEST.parser", "active",
      "<table><tr><td>"
      "expanded </td><td> [@@ONCLICK@@('i.am.the.ctrl')]<br>\n"
      "</td></tr><tr><td>"
      "expanded </td><td> [@@ONCLICK@@    (  'i.am.the.ctrl'  )]<br>\n"
      "</td></tr><tr><td>"
      "id stripped </td><td> [@@ONCLICK@@(  ' \" i.am.the.ctrl   \" ' )]<br>\n"
      "</td></tr><tr><td>"
      "no id </td><td> [@@ONCLICK@@()]<br>\n"
      "</td></tr><tr><td>"
      "error </td><td> [@@ONCLICK@@('i.am.the.ctrl']<br>\n"
      "</td></tr><tr><td>"
      "recover after error </td><td> [@@ONCLICK@@('i.am.the.ctrl')]<br>\n"
      "</td></tr><tr><td>"
      "A real test</td><td> <input type='button' value='Click me!' @@ONCLICK@@('i.am.the.ctrl') />\n"
      "</td></tr></table>"
      );

  m_display.addDialog("Calculator 1", res_calculator_ui, res_calculator_js, "Calculator");
  m_display.addDialog("Calculator 2", res_calculator_ui, res_calculator_js, "Calculator calc");
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
    m_display.setObjectTransform2D("Visualization.test.SVGPlotter-online", "little-shapes", &mat);
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
    std::ifstream infile;
    infile.open("subarchitectures/visualization.sa/config/test/images/shapes.svg", std::ifstream::in);
    std::stringstream str;
    str << infile.rdbuf();
    infile.close();
    m_display.setObject("Visualization.test.SVG-Ani", "shapes", str.str());
  }
#if HAS_LIBPLOT
#define YY(y) -(y)
  {
    if (1) {
      std::ostringstream ssvg;
      cogx::display::CSvgStringPlotter p(ssvg);

      p.openpl();
      p.flinewidth (2.0);        // line thickness in user coordinates
      p.pencolorname ("red");    // path will be drawn in red
      int a = 160;
      int b = 240;
      p.line(a, YY(a), b, YY(b));
      p.line(0, YY(b), 4*a, YY(b));
      p.line(2*a, YY(0), 2*a, YY(2*b));
      p.line(a, YY(a), -a, YY(a));
      p.line(-a, YY(a), b, YY(b));
      p.fontsize(20);
      p.fontname("serif");
      p.framecolor("yellow");
      p.fframewidth(3);
      p.textangle(30);
      p.fframedtext(a, YY(a), "Coordinate test");
      p.closepl();

      string s = p.getScreenSvg();

      m_display.setObject("Visualization.test.SVGPlotter", "simple", s);
      m_display.setObject("Visualization.test.SVGPlotter-online", "little-shapes", s);
    }
    else {
      std::ostringstream svg;
      SVGPlotter p(std::cin, svg, std::cerr);
      p.openpl();
      // p.parampl("PAGESIZE", (char*)"a4"); // doesn't work for SVG
      if (0) {
        p.fspace (0.0, 0.0, 640.0, 480.0); // specify user coor system
        // flip the Y coordinate
        p.fscale (1.0, -1.0);
        p.ftranslate(0.0, -480.0);
      }
      else {
        // flip the Y coordinate (default viewport is 0,0 1,1)
        p.fscale (1.0, -1.0);
        p.ftranslate(0.0, -1.0);
      }

      p.bgcolorname("none");
      p.erase();
      p.flinewidth (1.0);        // line thickness in user coordinates
      p.pencolorname ("red");    // path will be drawn in red
      int a = 160;
      int b = 240;
      p.line(a, a, b, b);
      p.line(0, b, 2*a, b);
      p.line(2*a, 0, 2*a, b);
      p.line(a, a, -a, a);
      p.line(-a, a, b, b);
      p.closepl();

      // Replace line 3 (<svg tag>) to remove size and viewport information
      string str = svg.str();
      ostringstream svgfix;
      size_t pos, ppos = 0;
      for (int i = 0; i < 3; i++) {
        pos = str.find("\n", ppos+1);
        if (pos == str.npos) break; // error
        if (i == 1)
          svgfix << str.substr(0, pos+1);
        ppos = pos;
      }
      svgfix << "<svg version=\"1.1\" baseProfile=\"full\" id=\"body\" preserveAspectRatio=\"none\" "
        "xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" "
        "xmlns:ev=\"http://www.w3.org/2001/xml-events\">\n";
      svgfix << str.substr(ppos+1);

      m_display.setObject("Visualization.test.SVGPlotter", "simple", svgfix.str());
      m_display.setObject("Visualization.test.SVGPlotter-online", "little-shapes", svgfix.str());
      //println(svg.str());
    }
  }
#endif

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
    str << "setCamera('LuaGL.default', 0, 0, 5, 0, 0, -1, 0, 1, 0)\n";
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
  {
    std::stringstream strA;
    strA << "This is the HIDDEN TestComponent for the Display Server<br>";
    m_display.setHtml("@info.HiddenComponent", "text", strA.str());
    m_display.enableDefaultView("@info.HiddenComponent", false);
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
    strB << "<input type='button' value='onclick' onclick=\"CastQFormProxy.onClick('id.something')\" /><br>" << endl;
    m_display.setHtml("@info.TestComponent", "zclock_test", strB.str());
    m_display.setHtml("@info.HiddenComponent", "zclock_test", strB.str());
  }
  // Composed views
  {
    std::vector<std::string> views;
    views.push_back("@htmlhead.TestComponent");
    views.push_back("@info.TestComponent");
    views.push_back("@info.HiddenComponent");
    views.push_back("Visualization.test.SVG");
    views.push_back("Visualization.test.SVGPlotter");
    m_display.createView("Composite.HTML", Visualization::VtHtml, views);

    m_display.setHtmlHead("@htmlhead.TestComponent", "svgstyle",
        "<style>div.svgpart{background:yellow;margin-top:4px;height:120px;}</style>");
    m_display.enableDefaultView("@htmlhead.TestComponent", false);
  }
  {
    std::vector<std::string> views;
    views.push_back(getComponentID());
    views.push_back("Visualization.test.SVG");
    views.push_back("Visualization.test.SVGPlotter");
    views.push_back("Visualization.test.SVGPlotter-online");
    m_display.createView("Composite.Image+Svg", Visualization::VtGraphics, views);
  }
  {
    std::vector<std::string> views;
    views.push_back("Visualization.test.SVG");
    views.push_back(getComponentID());
    m_display.createView("Composite.Svg+Image", Visualization::VtGraphics, views);
  }
  {
    std::vector<std::string> views;
    views.push_back(getComponentID());
    views.push_back("Visualization.test.SVG-Ani");
    m_display.createView("Composite.Image+Svg-Ani", Visualization::VtGraphics, views);
  }
  {
    std::vector<std::string> views;
    views.push_back("Visualization.sa.LuaGL");
    views.push_back("Visualization.test.Pusher");
    m_display.createView("Composite.Spiral+Pusher", Visualization::VtOpenGl, views);
  }
  {
    // The default view won't be created if the object is already in another view. Create manually.
    std::vector<std::string> views;
    views.push_back(getComponentID());
    m_display.createView(getComponentID(), Visualization::VtGraphics, views);
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

    str << "<select name=\"multiple\" multiple=\"multiple\"><option>Multiple</option>";
    str << "<option>Multiple2</option> <option>Multiple3</option>";
    str << "</select><br/>";
    m_HtmlForm.add(new CFormValues::set("multiple",
        CFormValues::valuelist() << "Multiple" << "Multiple2" << "Multiple3"));

    str << "<br/>";
    str << "<input type=\"text\" name=\"mytext\" value=\"Display Server Rules!\" />";
    m_HtmlForm.add(new CFormValues::field("mytext"));

    str << "<br/>";
    str << "<input type=\"checkbox\" name=\"checkboxname\" value=\"check1\"/> CheckBox1";
    str << "<input type=\"checkbox\" name=\"checkboxname\" value=\"check2\"/> CheckBox2";
    m_HtmlForm.add(new CFormValues::set("checkboxname", CFormValues::valuelist() << "check1" << "check2"));

    str << "<br/>";
    str << "<input type=\"radio\"  name=\"rbname\" value=\"radio1\"/> RadioButton1";
    str << "<input type=\"radio\"  name=\"rbname\" value=\"radio2\"/> RadioButton2";
    m_HtmlForm.add(new CFormValues::choice("rbname", CFormValues::valuelist() << "radio1" << "radio2"));

    str << "<br><input type=\"submit\" name=\"submit\" value=\"Apply\"/>";
    str << "<hr>";
    str << "<div id=\"debugout\"></div>";

    // Set the initial form data - before setHtmlForm!
    m_HtmlForm.setValue("mytext", "Display Server Rules! (setValue)");
    m_HtmlForm.setValue("rbname/radio1", "true");
    m_HtmlForm.setValue("checkboxname/check1", "true");
    m_HtmlForm.setValue("checkboxname/check2", "true");
    m_HtmlForm.setValue("multiple", "Multiple\nMultiple2");

    m_display.setHtmlForm("Visualization.test.HtmlForm", "001_text", str.str());
  }
  if (0) {
    // Test if the form could be restored
    std::stringstream str;
    str << "<script>";
    str << "var vals = {'single': ['Single2'], 'mytext': 'This also works: Display Server Rules!', ";
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
    boxrot = (boxrot + 1) % 36;
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

