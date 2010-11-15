/**
 * @author Michael Zillich
 * @date October 2006
 *
 * Changes:
 * 2010-11 Marko Mahnič:
 *   - switch image sequence while running
 *   - parser for INI files that describe available image sequences
 *   - UI to select a sequence using the Display Server (requires Visualization SA)
 */

#include <highgui.h>
//#include <opencv/highgui.h> -- this is incorrect according to the pkg-config flags
#include <cast/core/CASTUtils.hpp>
#include <VideoUtils.h>
#include "OpenCvImgSeqServer.h"

#include <iostream>
#include <sstream>
#include <algorithm>
#include <iterator>

#ifdef FEAT_VISUALIZATION
#include <fstream>
#include <limits.h>
#include <stdlib.h>
#endif

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::OpenCvImgSeqServer();
  }
}

namespace cast
{

using namespace std;

CSequenceInfo::CSequenceInfo()
{
  clear();
}

void CSequenceInfo::clear()
{
  fileTemplates = "";
  start = 0;
  end = 0;
  step = 1;
  repeatFrame = 1;
  downsampleFactor = 1;
  loop = true;
}

void CSequenceInfo::checkLimits()
{
  if (start < 0) start = 0;
  if (end < 0) end = 0;
  if (step <= 0) step = 1;
  if (repeatFrame <= 0) repeatFrame = 1;
  if(downsampleFactor <= 0) downsampleFactor = 1;
}

void CSequenceInfo::parseConfig(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  clear();

  if((it = _config.find("--files")) != _config.end())
  {
    fileTemplates = it->second;
  }
  if((it = _config.find("--start")) != _config.end())
  {
    istringstream str(it->second);
    str >> start;
  }
  if((it = _config.find("--end")) != _config.end())
  {
    istringstream str(it->second);
    str >> end;
  }
  if((it = _config.find("--step")) != _config.end())
  {
    istringstream str(it->second);
    str >> step;
  }
  if((it = _config.find("--repeatframe")) != _config.end())
  {
    istringstream str(it->second);
    str >> repeatFrame;
  }
  if((it = _config.find("--noloop")) != _config.end())
  {
    loop = false;
  }
  if((it = _config.find("--downsample")) != _config.end())
  {
    istringstream str(it->second);
    str >> downsampleFactor;
  }
  checkLimits();
}

void CSequenceInfo::parseConfig(const std::string& _configStr)
{
  map<string,string> params;
  // TODO: split string, join quoted strings, add to map
  vector<string> tokens, parts;
  vector<string>::iterator it, itnx;
  istringstream iss(_configStr);
  copy(istream_iterator<string>(iss),
      istream_iterator<string>(),
      back_inserter<vector<string> >(tokens));

  // Fix quoted parameters (escape sequences are not handled)
  for(it = tokens.begin(); it != tokens.end(); it++) {
    if (it->substr(0, 1) == "\"") {
      string s = *it;
      while (s.substr(s.length()-1, 1) != "\"") {
        it++;
        if (it == tokens.end()) break;
        s = s + " " + *it;
      }
      parts.push_back(s.substr(1, s.length()-2));
    }
    else if (it->substr(0, 1) == "\'") {
      string s = *it;
      while (s.substr(s.length()-1, 1) != "\'") {
        it++;
        if (it == tokens.end()) break;
        s = s + " " + *it;
      }
      parts.push_back(s.substr(1, s.length()-2));
    }
    else parts.push_back(*it);
  }  

  // create a map from the list
  for(it = parts.begin(); it != parts.end(); it++) {
    string& s = *it;
    if (s.substr(0, 2) != "--") continue; // TODO: error - expecting parameter

    string ns;
    itnx = it+1;
    if (itnx == tokens.end()) ns = "";
    else {
      ns = *itnx;
      if (ns.substr(0, 2) == "--") ns = "";
    }
    params[s] = ns;
    if (itnx->substr(0, 2) != "--") it++;
  }

  parseConfig(params);
}

void CSequenceInfo::setInfo(Video::VideoSequenceInfo& info)
{
  clear();
  fileTemplates = info.fileTemplates;
  start = info.start;
  end = info.end;
  step = info.step;
  repeatFrame = info.repeatFrame;
  loop = info.loop;
  downsampleFactor = 1;
  checkLimits();
}

// default framerate if none specified is 1 s
static const int FRAMERATE_DEFAULT = 1000;

OpenCvImgSeqServer::OpenCvImgSeqServer()
{
  framerateMillis = FRAMERATE_DEFAULT;
  frameCnt = 0;
  downsampleFactor = 1;
  width = height = 0;
  frameRepeatCnt = 1;
  loopSequence = true;
#ifdef FEAT_VISUALIZATION
  sequenceIniFile = "";
#endif
}

OpenCvImgSeqServer::~OpenCvImgSeqServer()
{
  for(size_t i = 0; i < grabbedImages.size(); i++)
    cvReleaseImage(&grabbedImages[i]);
}

void OpenCvImgSeqServer::init(const vector<string> &fileTemplates,
    int first, int last, int inc) throw(runtime_error)
{
  if(fileTemplates.size() == 0)
    throw runtime_error(exceptionMessage(__HERE__, "no image lists given"));
  if(fileTemplates.size() != camIds.size())
    throw runtime_error(exceptionMessage(__HERE__,
          "number of file templates %d does not match number of camera IDs %d",
          (int)fileTemplates.size(), (int)camIds.size()));
  grabTimes.resize(getNumCameras());
  grabbedImages.resize(getNumCameras());
  for(size_t i = 0; i < grabbedImages.size(); i++)
    grabbedImages[i] = 0;
  constructFilenames(fileTemplates, first, last, inc);
  obtainImageSize();
}

/**
 * Read the first image of the sequence and store its size. This assumes of
 * course that all images in the sequence have the same size.
 * Furthermore this requires the file list to be initialised already
 */
void OpenCvImgSeqServer::obtainImageSize() throw(runtime_error)
{
  // if width and height are not set yet
  if(width == 0)
  {
    IplImage *img = 0;

    if(filenames.size() == 0)
      throw runtime_error(exceptionMessage(__HERE__, "video not initialised"));
    // load the first image and have a look at its size
    // note: we are not simply using GrabFrames() here, as that would increment
    // the counter, set times etc.
    img = cvLoadImage(filenames[0].c_str(), CV_LOAD_IMAGE_COLOR);
    if(img == 0)
      throw runtime_error(exceptionMessage(__HERE__,
          "failed to load image '%s'", filenames[0].c_str()));
    width = img->width;
    height = img->height;
    cvReleaseImage(&img);
  }
  else
    throw runtime_error(exceptionMessage(__HERE__,
        "obtainImageSize() must only be called once"));
}

/**
 * Construct filenames from file templates.
 * Each camera has a file template, e.g. img_left_%03d.jpg img_right_%03d.jpg
 */
void OpenCvImgSeqServer::constructFilenames(const vector<string> &fileTemplates,
    int first, int last, int inc)
{
  char filename[1024];
  // normal order: first < last, positive increment
  // (note first = last is a special case: just one image)
  if(first <= last)
  {
    // if no increment given, assume default
    if(inc == 0)
      inc = 1;
    // just in case we were given a stupid increment
    else if(inc < 0)
      inc = -inc;
    for(int i = first; i <= last; i += inc)
      for(size_t c = 0; c < fileTemplates.size(); c++)
      {
        snprintf(filename, 1024, fileTemplates[c].c_str(), i);
        filenames.push_back(filename);
      }
  }
  // reverse order (to run a movie backwards): first > last, negative increment
  else
  {
    if(inc == 0)
      inc = -1;
    else if(inc > 0)
      inc = -inc;
    for(int i = first; i >= last; i += inc)
      for(size_t c = 0; c < fileTemplates.size(); c++)
      {
        snprintf(filename, 1024, fileTemplates[c].c_str(), i);
        filenames.push_back(filename);
      }
  }
}

void OpenCvImgSeqServer::grabFramesInternal() throw(runtime_error)
{
  if(filenames.size() == 0)
    throw runtime_error(exceptionMessage(__HERE__, "video not initialised"));
  if(frameCnt < numFrames() || loopSequence)
  {
    // number of current frame, note that we loop
    int fn = frameCnt % numFrames();
    for(size_t i = 0; i < grabbedImages.size(); i++)
    {
      cvReleaseImage(&grabbedImages[i]);
      grabbedImages[i] = cvLoadImage(filenames[fn*getNumCameras() + i].c_str(),
          CV_LOAD_IMAGE_COLOR);
      if(grabbedImages[i] == 0)
        throw runtime_error(exceptionMessage(__HERE__,
              "failed to load image '%s'",
              filenames[fn*getNumCameras() + i].c_str()));
      if(grabbedImages[i]->width != width || grabbedImages[i]->height != height)
        throw runtime_error(exceptionMessage(__HERE__,
              "size of loaded image '%s': %dx%d does not match video size %dx%d",
              filenames[fn*getNumCameras() + i].c_str(),
              grabbedImages[i]->width, grabbedImages[i]->height,
              width, height));
    }
  }
  else
  {
    // return empty images
    for(size_t i = 0; i < grabbedImages.size(); i++)
    {
      cvReleaseImage(&grabbedImages[i]);
      grabbedImages[i] = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
      cvSet(grabbedImages[i], cvScalar(0));
    }
  }
  cdl::CASTTime time = getCASTTime();
  for(size_t i = 0; i < grabTimes.size(); i++)
    grabTimes[i] = time;

  if (frameRepeatPos > frameRepeatCnt) frameRepeatPos = frameRepeatCnt;
  frameRepeatPos--;
  if (frameRepeatPos <= 0)
  {
    frameRepeatPos = frameRepeatCnt;
    frameCnt++;
  }
}

void OpenCvImgSeqServer::grabFrames()
{
  // TODO: if (VideoSequenceInfo present) switchSequence(seqInfo);

  // note that by just calling sleep(framrate) we actually do not really
  // get a fixed framerate, this would require setitimer() and pause()
  sleepComponent(framerateMillis);
  grabFramesInternal();
}

void OpenCvImgSeqServer::retrieveFrameInternal(int camIdx, int width, int height,
    Video::Image &frame)
{
  // To handle the case where retrieve is called before any Grab
  if(!haveFrames())
    grabFramesInternal();

  frame.time = grabTimes[camIdx];
  frame.camId = camIds[camIdx];
  frame.camPars = camPars[camIdx];

  // no size given, use native size
  if((width == 0 || height == 0) || (width == this->width && height == this->height))
  {
    convertImageFromIpl(grabbedImages[camIdx], frame);
    // adjust to native size
    // (note that calibration image size need not be the same as currently set
    // native capture size)
    changeImageSize(frame.camPars, this->width, this->height);
  }
  else
  {
    IplImage *tmp = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    cvResize(grabbedImages[camIdx], tmp);
    convertImageFromIpl(tmp, frame);
    // TODO: avoid allocate/deallocating all the time
    cvReleaseImage(&tmp);
    // adjust to scaled image size
    changeImageSize(frame.camPars, width, height);
  }
}

void OpenCvImgSeqServer::retrieveFrames(const std::vector<int> &camIds,
    int width, int height, std::vector<Video::Image> &frames)
{
  frames.resize(camIds.size());
  for(size_t j = 0; j < camIds.size(); j++)
  {
    size_t i = getCamIndex(camIds[j]);
    retrieveFrameInternal(i, width, height, frames[j]);
  }
}

void OpenCvImgSeqServer::retrieveFrames(int width, int height,
    vector<Video::Image> &frames)
{
  frames.resize(getNumCameras());
  for(size_t i = 0; i < (size_t)getNumCameras(); i++)
    retrieveFrameInternal(i, width, height, frames[i]);
}

void OpenCvImgSeqServer::retrieveFrame(int camId, int width, int height,
    Video::Image &frame)
{
  size_t i = getCamIndex(camIds[camId]);
  retrieveFrameInternal(i, width, height, frame);
}

void OpenCvImgSeqServer::retrieveHRFrames(std::vector<Video::Image> &frames)
{
  printf("OpenCvImgSeqServer::retrieveHRFrames: not yet implemented.\n");
}

/**
 * Returns whether any frames have been grabbed yet.
 */
bool OpenCvImgSeqServer::haveFrames()
{
  for(int i = 0; i < getNumCameras(); i++)
    if(grabbedImages[i] == 0)
      return false;
  return true;
}

void OpenCvImgSeqServer::getImageSize(int &width, int &height)
{
  width = this->width;
  height = this->height;
}

int OpenCvImgSeqServer::getFramerateMilliSeconds()
{
  return framerateMillis;
}

void OpenCvImgSeqServer::configure(const map<string,string> & _config)
  throw(runtime_error)
{
  //int start = 0, end = 0, step = 1;
  //vector<string> fileTemplates;
  map<string,string>::const_iterator it;

  // first let the base class configure itself
  VideoServer::configure(_config);

  // The framerate can be configured at startup, but not when switching sequences
  if((it = _config.find("--framerate_ms")) != _config.end())
  {
    istringstream str(it->second);
    str >> framerateMillis;
    if(framerateMillis <= 0)
      framerateMillis = FRAMERATE_DEFAULT;
  }

  CSequenceInfo sequenceInfo;
  sequenceInfo.parseConfig(_config);

#ifdef FEAT_VISUALIZATION
  if((it = _config.find("--sequences")) != _config.end())
  {
    sequenceIniFile = it->second;
    if (! parseSequenceIniFile(sequenceIniFile)) {
      sequenceIniFile = "";
    }
  }

  if((it = _config.find("--show_sequence")) != _config.end())
  {
    if (sequenceMap.find(it->second) == sequenceMap.end()) {
        throw runtime_error(exceptionMessage(__HERE__,
              "The sequence '%s' specified in --show_sequence does not exist in --sequences",
              it->second.c_str()));
    }
    else {
      sequenceInfo = sequenceMap[it->second];
    }
  }
#endif

  if (sequenceInfo.fileTemplates.length() < 1) {
    throw runtime_error(exceptionMessage(__HERE__,
          "No video sequence was configured. Use --files or --sequences/--show_sequence."));
  }

  switchSequence(sequenceInfo);

#ifdef FEAT_VISUALIZATION
  m_display.configureDisplayClient(_config);
#endif
}

#ifdef FEAT_VISUALIZATION
// Read definitions of image sequences from an INI file.
//
// INI example:
//    [:global:]
//    imagepath=%(INI_DIR)/images
//
//    [MyBlueMug]
//    files=%(imagepath)/image%03-L.png %(imagepath)/image%03d-R.png
//    in=--files "%(files)" --start 0 --end 10
//    sequence=--files "%(files)" --start 11 --end 20
//    out=--files "%(files)" --start 21 --end 30
//
// Sequences are defined with parameters accepted by CSequenceInfo::parseConfig.
// Variable names INIPATH, in, out and sequence have a special meaning.
//
bool OpenCvImgSeqServer::parseSequenceIniFile(const std::string& fname)
{
  ifstream f(fname.c_str(), ifstream::in);
  if (f.fail()) return false;

  string section;
  struct {
    map<string, string> globalVar, localVar;
    string expandVars(const string& str)
    {
      string::size_type pos = str.find("%(");
      if (pos == string::npos) return str;

      // Result is built incrementally
      ostringstream oss;
      oss << str.substr(0, pos);

      // Position after the last position from which output was generated
      string::size_type nextpos = pos;

      while (pos != string::npos && pos < str.size()) {
        string::size_type epos = str.find(")", pos);
        if (epos == string::npos) break;

        string var = str.substr(pos + 2, epos-(pos+2));

        // Replace the %(variable) with value.
        // If the variable is not defined, keep original string.
        if (localVar.find(var) != localVar.end()) oss << localVar[var];
        else if (globalVar.find(var) != globalVar.end()) oss << globalVar[var];
        else {
          oss << str.substr(pos, epos-pos+1);
          // TODO: warning, variable not defined
        }

        nextpos = epos+1;
        pos = str.find("%(", nextpos);
        if (pos != string::npos) {
          oss << str.substr(nextpos, pos-nextpos);
          nextpos = pos;
        }
      }

      if (nextpos != string::npos && nextpos < str.size()) {
        oss << str.substr(nextpos);
      }
      return oss.str();
    }
  } state;

  {
    char cpath[PATH_MAX];
    realpath(fname.c_str(), cpath);
    string path(cpath);
    string::size_type const pos = path.find_last_of("/");
    if (pos == string::npos) state.globalVar["INI_DIR"] = "/";
    else state.globalVar["INI_DIR"] = path.substr(0, pos);
  }

  while(f.good()) {
    string line;
    getline(f, line);

    // strip ws from line
    string::size_type pos = line.find_first_not_of(" \t\n");
    line = ( pos == string::npos ) ? string() : line.substr(pos, line.find_last_not_of(" \t\n")-pos+1);

    // skip empty lines and comments
    if (line == "") continue;
    if (line[0] == '#') continue;

    if (line[0] == '[') {
      // new image sequence - extract name
      state.localVar.clear();
      pos = line.find_first_not_of(" \t\n[");
      section = ( pos == string::npos ) ? string() : line.substr(pos, line.find_last_not_of(" \t\n]")-pos+1);
      if (section == "") section = "[null]"; // TODO: notify error
      continue;
    }
    else {
      if (section == "[null]") continue;
      if (section == "") section = ":global:";
      string::size_type const posEq = line.find("=");
      if (posEq == string::npos) continue; // TODO: notify error

      string var, value;
      pos = line.find_last_not_of(" \t\n", posEq-1);
      if (pos == string::npos) continue; // TODO: notify error
      else var = line.substr(0, pos+1);

      pos = line.find_first_not_of(" \t\n", posEq+1);
      if (pos == string::npos) value = "";
      else value = line.substr(pos);

      if (section == ":global:") state.globalVar[var] = state.expandVars(value);
      else {
        if (var == "sequence") {
          CSequenceInfo info;
          info.parseConfig(state.expandVars(value));
          sequenceMap[section] = info;
        }
        // transitions - may be supported some day
        //else if (var == "in" || var == "out") {
        //  CSequenceInfo info;
        //  info.parseConfig(state.expandVars(value));
        //  transitionMap[section + "-" + var] = info;
        //}
        else if (var == "INI_DIR") {
          continue; // TODO: notify error: some names shouldn't be used
        }
        else {
          state.localVar[var] = state.expandVars(value);
        }
      }
    }
  }
  f.close();
  return true;
}

#define ID_V11N_OBJECT "ImageServer.Sequence"
#define IDCHUNK_SEQUENCES "SequenceButtons"

void OpenCvImgSeqServer::start()
{
  // Enable Display only if configured with multiple sequences
  if (sequenceIniFile != "" && sequenceMap.size() > 1) {
    m_display.connectIceClient(*this);
    m_display.installEventReceiver();
    m_display.setClientData(this);

    map<string,CSequenceInfo>::iterator it;
    ostringstream ss;
    for (it = sequenceMap.begin(); it != sequenceMap.end(); it++) {
      ss << "<input type='button' value='Load sequence " << it->first << "' "
         << "@@ONCLICK@@('" << it->first << "');\" /><br>";
    }
    m_display.setActiveHtml(ID_V11N_OBJECT, IDCHUNK_SEQUENCES, ss.str());
  }

  VideoServer::start();
}

void OpenCvImgSeqServer::CDisplayClient::handleEvent(const Visualization::TEvent& event)
{
  if (event.type == Visualization::evHtmlOnClick) {
    log("evHtmlOnClick on " + event.sourceId + " (" + event.objectId + ":" + event.partId + ")");
    string seqname = event.sourceId;
    if (pComponent->sequenceMap.find(seqname) != pComponent->sequenceMap.end()) {
      log("going to load sequence " + seqname);
      pComponent->switchSequence(pComponent->sequenceMap[seqname]);
    }
  }

}
#endif

void OpenCvImgSeqServer::switchSequence(CSequenceInfo& seq)
{
  seq.checkLimits();

  // XXX: filenames are not cleared by constructFilenames().
  // XXX: width & height are reset so that obtainImageSize() will work.
  // TODO: frame size should remain unchanged when switching sequences (use downsampleFactor?)
  filenames.clear();
  width = 0;
  height = 0;

  vector<string> fileTemplates;
  {
    istringstream str(seq.fileTemplates);
    string file;
    while(str >> file)
      fileTemplates.push_back(file);
  }
  // do some initialisation based on configured items
  init(fileTemplates, seq.start, seq.end, seq.step);

  frameRepeatCnt = seq.repeatFrame;
  if(frameRepeatCnt <= 0) frameRepeatCnt = 1;
  loopSequence = seq.loop;

  // Restart the sequence
  frameRepeatPos = frameRepeatCnt;
  frameCnt = 0;
}

/**
 * @brief This function is only for the PointGrey server available: experimental mode.
 * @param width Image width
 * @param height Image height
 * @param offsetX Offset in x- direction for Format7 mode.
 * @param offsetY Offset in y- direction for Format7 mode.
 * @param mode Image grabbing mode for the Format7 mode.
 * @param fps Requested framerate [1/s]
 */
void OpenCvImgSeqServer::changeFormat7Properties(int width, int height, int offsetX, int offsetY, int mode, int paketSize)
{
  log("only for the PointGrey server available: abort.");
}

/**
 * @brief Camera is not in Format7 mode.
 * @return Returns false, when cameras not in Format7 mode.
 */
bool OpenCvImgSeqServer::inFormat7Mode()
{
  return false;
}

/**
 * @brief Get the server name
 * @param name Server name (PointGreyServer / OpenCvImgSeqServer / OpenCvLiveServer)
 */
const std::string OpenCvImgSeqServer::getServerName()
{
  const std::string str("OpenCvImgSeqServer");
  return str;
}

}
