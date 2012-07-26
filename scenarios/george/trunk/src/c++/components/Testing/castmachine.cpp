/**
 * @author Marko Mahnič
 * @created April 2012
 */
#include "castmachine.hpp"
#include "tester.hpp"

#include <dialogue.hpp>
#include <dialogue_utils.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

#include <sstream>
#include <fstream>
#include <cstdlib>

namespace dlgice = de::dfki::lt::tr::dialogue::slice;

namespace testing {

#define MSSG(streamexpr)  ((std::ostringstream&)(std::ostringstream() << std::flush << streamexpr))

std::string CTeachTestEntry::getLessonText(int numLesson)
{
  if (numLesson == 1 && mColor != "")
    return "the object is " + mColor;
  if (numLesson == 2 && mShape != "")
    return "the object is " + mShape;

  return "";
}

// TODO 0 NO answer, 1 YES, 2 NO
long CTeachTestEntry::classifyResponse(int numLesson, const std::string& response)
{
  if (response.size() > 0 && response.substr(0, 2) == "ok")
    return OK;
  return NOTOK;
}

#define TIME_VIEW "Evaluator.tm"

CCastMachine::CCastMachine(CTester* pOwner)
#ifdef FEAT_VISUALIZATION
  : mDisplay(pOwner->mDisplay)
#endif
{
  setCastComponent(pOwner);
  setLoggingComponent(pOwner);
  mPlayerHost = "localhost";
  mPlayerPort = 6665;
  mCurrentTest = 0;
  mOptions["dialogue.sa"] = "dialogue"; // hardcoded default SA name; change with --dialogue-sa
  mOptions["vision.sa"] = pOwner->getSubarchitectureID(); // change with --vision-sa

}

void CCastMachine::configure(const std::map<std::string,std::string> & _config)
{
  std::map<std::string,std::string>::const_iterator it;

  if((it = _config.find("--player-host")) != _config.end()) {
    mPlayerHost = it->second;
  }
  if((it = _config.find("--player-port")) != _config.end()) {
    std::istringstream str(it->second);
    str >> mPlayerPort;
  }
  if((it = _config.find("--objects")) != _config.end()) {
    // This defines what objects and places are available. Shared with GazeboJuggler.
    loadObjectsAndPlaces(it->second);
  }
  if((it = _config.find("--learn-attrs")) != _config.end()) {
    // This loads attributes for the objects in --objects.
    loadLearningAttributes(it->second);
  }

  // CONFIG: --dialogue-sa
  // TYPE: string (subarchitecture-id)
  // DEFAULT: "dialogue"
  // The subarchitecture id in which ASR is running
  if((it = _config.find("--dialogue-sa")) != _config.end())
  {
    if (it->second == "") mOptions["dialogue.sa"] = castComponent()->getSubarchitectureID();
    else mOptions["dialogue.sa"] = it->second;
  }

  // CONFIG: --vision-sa
  // TYPE: string (subarchitecture-id)
  // DEFAULT: ""
  // The subarchitecture id in which SOIFilter, ObjectAnalyzer and OpenCvImgSeqServer
  // video server are running. The default is empty which means that the components
  // are running in the same SA as this component.
  if((it = _config.find("--vision-sa")) != _config.end())
  {
    if (it->second == "") mOptions["vision.sa"] = castComponent()->getSubarchitectureID();
    else mOptions["vision.sa"] = it->second;
  }

  // CONFIG: --start-number
  // TYPE: integer
  // The number of the test to start with. The number of the first test is 0.
  //if((it = _config.find("--start-number")) != _config.end())
  //{
  //  istringstream ss(it->second);
  //  ss >> m_currentTest;
  //  if (m_currentTest < 0) m_currentTest = 0;
  //}

  mpRobot = std::unique_ptr<PlayerCc::PlayerClient>(new PlayerCc::PlayerClient(mPlayerHost, mPlayerPort));
  mpSim = std::unique_ptr<PlayerCc::SimulationProxy>(new PlayerCc::SimulationProxy(&*mpRobot, 0));
  log("Connected to Player %x", &*mpRobot);

  prepareObjects();
  msObjectOnScene = "";
}

void CCastMachine::loadObjectsAndPlaces(const std::string& fname)
{
  mObjects.clear();
  mLocations.clear();
  std::ifstream f;
  f.open(fname.c_str());
  if (f.fail()) {
    error("File not found: '" + fname + "'");
  }
  else {
    int mode = 0; // 1 - objects, 2 - places
    while (f.good() && !f.eof()) {
      std::string line, tok;
      std::getline(f, line);
      //println(" GJ **** " + line);
      std::istringstream itok(line);

      int newmode = mode;
      itok >> tok;
      if (tok == "[objects]") newmode = 1;
      else if (tok == "[places]") newmode = 2;
      if (mode != newmode) {
        mode = newmode;
        continue;
      }

      if (mode == 1) {
        if (tok != "")
          // In Gazebo 0.9 top objects didn't have a prefix.
          // In Gazebo 0.10 the prefix is "noname::"
          mObjects.push_back(GObject(tok, "noname::" + tok));
      }
      else if (mode == 2) {
        cogx::Math::Vector3 v;
        int n;
        n = sscanf(line.c_str(), "%lf %lf %lf", &v.x, &v.y, &v.z);
        if (n > 1) {
          if (n != 3) v.z = 1.0;
          else v.z += 0.5;
          mLocations.push_back(v);
        }
      }
    }
    f.close();
  }
}

void CCastMachine::loadLearningAttributes(const std::string& fname)
{
#if 0 // debugging, with george-01.world
  for (int i = 0; i < 10; i++) { 
    if (i >= mObjects.size()) break; 
    auto pe = new CTeachTestEntry(mObjects[i].label, "orange", "elongated"); 
    mTestEntries.push_back(CTestEntryPtr(pe)); 
  } 
  return;
#endif

  mTestEntries.clear();
  std::ifstream f;
  f.open(fname.c_str());
  if (f.fail()) {
    error("File not found: '" + fname + "'");
  }
  else {
    while (f.good() && !f.eof()) {
      std::string line, name, color, shape;
      std::getline(f, line);
      std::istringstream itok(line);

      itok >> name >> color >> shape;

      auto iobj = std::find_if(mObjects.begin(), mObjects.end(), [&name](GObject& obj) {
            return obj.label == name;
          });
      if (iobj == mObjects.end()) {
        castComponent()->log("Object '%s' is in --learn-attrs but not in --objects", name.c_str());
        continue;
      }
      auto pe = new CTeachTestEntry(name, color, shape);
      mTestEntries.push_back(CTestEntryPtr(pe));
    }
    f.close();
  }
}

const double OFF = 121231e99; 
void CCastMachine::prepareObjects()
{
  double time;
  // Discover and keep valid objects
  std::vector<GObject> objs = mObjects;
  std::vector<GObject> notthere;
  mObjects.clear();
  for (unsigned int i = 0; i < objs.size(); i++) {
    GObject& o = objs[i];
    o.loc.x = OFF;
    mpSim->GetPose3d((char*)o.gazeboName.c_str(), o.loc.x, o.loc.y, o.loc.z, o.pose.x, o.pose.y, o.pose.z, time);
    if (o.loc.x == OFF) {
      println(" *** Object '%s' is not in the scene.", o.label.c_str());
      notthere.push_back(o);
      continue;
    }
    o.loc.x = 100 + (i / 10) * 3;
    o.loc.y = 100 + (i % 10) * 3;
    mpSim->SetPose3d((char*)o.gazeboName.c_str(), o.loc.x, o.loc.y, o.loc.z, o.pose.x, o.pose.y, o.pose.z);
    mObjects.push_back(o);
  }
  println("%ld objects managed in the scene.", mObjects.size());
}

long CCastMachine::getCount(const std::string& counter)
  // Enter: Locked mCount
{
  auto p = mCount.find(counter);
  if (p == mCount.end()) return 0;
  return p->second;
}

void CCastMachine::report(std::ostringstream& what)
{
  report(what.str());
}

void CCastMachine::report(const std::string& message)
{
  log("%s", message.c_str());
}

void CCastMachine::reportTimeout(const std::string& reason)
{
  report("Timeout (" + reason + ")");
}

void CCastMachine::reportRunningTime(const std::string& id, double seconds)
{
  log("%s finished in %.3g", id.c_str(), seconds);

#ifdef FEAT_VISUALIZATION
  auto it = mRunningTimes.find(id);
  if (it == mRunningTimes.end()) {
    mRunningTimes[id] = std::vector<double>();
    it = mRunningTimes.find(id);
  }
  auto& times = it->second;
  times.push_back(seconds);

  std::string idhtm = "110." + id;
  std::ostringstream ss;
  ss << "<tr><td>" << id << "</td>";

  ss << "<td>Now(" << times.size() << "):</td>";
  int pos = times.size() - 1;
  int i = 5;
  while (i > 0 && pos >= 0) {
    ss << "<td>" << times[pos] << "</td>";
    --pos;
    --i;
  }
  while (--i >= 0) ss << "<td>&nbsp;</td>";

  i = 3;
  if (pos <= 16) {
    ss << "<td>Mid:</td>";
  }
  else {
    pos /= 2;
    ss << "<td>Mid(" << (pos - times.size()) << "):</td>";
    while (i > 0 && pos >= 0) {
      ss << "<td>" << times[pos] << "</td>";
      --pos;
      --i;
    }
  }
  while (--i >= 0) ss << "<td>&nbsp;</td>";

  if (pos >= 3) {
    ss << "<td>Start:</td>";
    pos = 2;
    while (pos >= 0) {
      ss << "<td>" << times[pos] << "</td>";
      --pos;
    }
  }
  ss << "</tr>";
  mDisplay.setHtml(TIME_VIEW, idhtm, ss.str());
#endif
}

bool CCastMachine::verifyCount(const std::string& counter, long min, long max)
{ 
  long count = getCount(counter);
  if (min < 0) min = 0;
  if (max < min) max = min;
  if (min <= count && count <= max)
    return true;

  static std::string lastReportId;
  std::string reportId = MSSG("rep::" << mCurrentTest << "::" << counter << "::" << count).str();

  if (lastReportId != reportId) {
    lastReportId = reportId;
    if (min >= max) {
      report(MSSG("Count of " << counter << " is " << count << " while it should be " << min << "."));
    }
    else {
      report(MSSG("Count of " << counter << " is " << count << " while it should be between "
            << min << " and " << max << "."));
    }
  }
  return false;
#if 0
  static int lastReport = 0;
  int curReport = mCurrentTest * 100 + mObjectCount;
  if (count == 0) {
    if (mObjectCount != 0) {
      if (lastReport != curReport)
        report(MSSG("The scene should be empty, but I see " << mObjectCount << " objects."));
      lastReport = curReport;
    }
  }
  else if (count == 1) {
    if (mObjectCount != 1) {
      if (lastReport != curReport)
        report(MSSG("There should be one object on the scene, but I see " << mObjectCount << "."));
      lastReport = curReport;
    }
  }
  else {
    if (mObjectCount == 0) {
      if (lastReport != curReport)
        report(MSSG("There should some objects on the scene, but I see " << mObjectCount << "."));
      lastReport = curReport;
    }
  }
#endif
}

void CCastMachine::addRobotResponse(const std::string &response)
  // Enter: Locked mRobotResponses
{
  mRobotResponses.push_back(response);
}

void CCastMachine::clearRobotResponses()
  // Enter: Locked mRobotResponses
{
  mRobotResponses.clear();
}


CTestEntryPtr CCastMachine::getCurrentTest()
{
  if (mCurrentTest <= 0 || mCurrentTest > mTestEntries.size())
    return CTestEntryPtr();
  return mTestEntries[mCurrentTest-1];
}

bool CCastMachine::nextScene()
{
  ++mCurrentTest;
  mTeachingStep = 0;
  return getCurrentTest().get() != nullptr;
}

bool CCastMachine::loadObjectModel(const std::string& label, const std::string& modelFilename)
{
  static long modelId = 0;

  auto ito = std::find_if(mObjects.begin(), mObjects.end(), [&label](GObject& obj) {
      return obj.label == label;
      });
  if (ito != mObjects.end()) {
    deleteObjectModel(label);
  }

  ++modelId;
  //GObject go(label, "noname::" + label);
  GObject go(label, label);
  go.loc.x =  1 + (modelId / 10) * 3;
  go.loc.y = -1 - (modelId % 10) * 3;
  go.loc.z = 1.5;

  std::ostringstream cmd;
  cmd << "gzfactory spawn -f " << modelFilename << " -m " << label;
  cmd << " -x " << go.loc.x;
  cmd << " -y " << go.loc.y;
  cmd << " -z " << go.loc.z;
  log("Executing w. system: %s", cmd.str().c_str());
  int rv = system(cmd.str().c_str());

  if (rv == 0) {
    mObjects.push_back(go);
  }

  return rv == 0;
}

bool CCastMachine::deleteObjectModel(const std::string& label)
{
  auto ito = std::find_if(mObjects.begin(), mObjects.end(), [&label](GObject& obj) {
      return obj.label == label;
      });
  if (ito == mObjects.end()) {
    return false;
  }
  mObjects.erase(ito);

  std::string cmd("gzfactory delete -m " + label);
  log("Executing w. system: %s", cmd.c_str());
  int rv = system(cmd.c_str());

  return rv == 0;
}

bool CCastMachine::moveObject(const std::string& label, int placeIndex)
{
  for(unsigned int i = 0; i < mObjects.size(); i++) {
    GObject& o = mObjects[i];
    if (o.label != label)
      continue;
    bool physon = false;
    if (placeIndex < 0 || placeIndex >= mLocations.size()) {
      o.loc.x = 100 + (i / 10) * 3;
      o.loc.y = 100 + (i % 10) * 3;
      physon = false;
    }
    else {
      o.loc = mLocations[placeIndex];
      physon = true;
    }
    auto pose_z = o.pose.z;
    // XXX: causes problems (unstable stereo): pose_z = (std::rand() % 30 - 15) * 3.16 / 180;
    mpSim->SetPose3d((char*)o.gazeboName.c_str(), o.loc.x, o.loc.y, o.loc.z, o.pose.x, o.pose.y, pose_z);
    return true;
  }
  return false;
}

bool CCastMachine::loadScene()
{
  CTestEntryPtr pt(getCurrentTest());
  if (! pt) return false;

  CTeachTestEntry* pe = dynamic_cast<CTeachTestEntry*>(pt.get());
  if (pe) {
    bool rv;
    std::string model = "instantiations/xdata/gazebo/models/gen-" + pe->mLabel + ".model"; // XXX HARDCODED :(
    rv = loadObjectModel(pe->mLabel, model);
    rv = moveObject(pe->mLabel, 0);
    msObjectOnScene = pe->mLabel;
    castComponent()->log("Placing '%s' on the scene.", pe->mLabel.c_str());
    return rv;
  }

  return false;

#if 0
   CTestEntry *pinfo = getCurrentTest();
   if (! pinfo) return;
   report(MSSG("SCENE " << m_currentTest << ": " << pinfo->name));

   Video::VideoSequenceInfoPtr pseq = new Video::VideoSequenceInfo();

   pseq->fileTemplates = m_imageDir + "/" + pinfo->videoLeft + " " + m_imageDir + "/" + pinfo->videoRight;
   pseq->start = 0;
   pseq->end = 0;
   pseq->step = 0;
   pseq->loop = true;
   pseq->repeatFrame = 5;

   cdl::WorkingMemoryAddress addr;
   addr.subarchitecture = options["vision.sa"];
   addr.id = m_pOwner->newDataID();

   m_pOwner->addToWorkingMemory(addr, pseq);
#endif
}

void CCastMachine::clearScene()
{
  if (msObjectOnScene != "") {
    moveObject(msObjectOnScene, -1);
    deleteObjectModel(msObjectOnScene);
    castComponent()->log("Removing '%s' from the scene.", msObjectOnScene.c_str());
  }

#if 0
   Video::VideoSequenceInfoPtr pseq = new Video::VideoSequenceInfo();

   pseq->fileTemplates = m_imageDir + "/empty-l.png" + " " + m_imageDir + "/empty-r.png";
   pseq->start = 0;
   pseq->end = 0;
   pseq->step = 0;
   pseq->loop = true;
   pseq->repeatFrame = 5;

   cdl::WorkingMemoryAddress addr;
   addr.subarchitecture = options["vision.sa"];
   addr.id = m_pOwner->newDataID();

   m_pOwner->addToWorkingMemory(addr, pseq);
   report("SCENE: empty");
#endif
}


// If hasMoreLessons then nextLesson will select a valid lesson
bool CCastMachine::hasMoreLessons()
{
  CTestEntryPtr pt(getCurrentTest());
  if (mTeachingStep < 0) mTeachingStep = 0;
  return pt.get() != nullptr && mTeachingStep < pt->getLessonCount();
}

bool CCastMachine::nextLesson()
{
  ++mTeachingStep;
  CTestEntryPtr pt(getCurrentTest());
  return pt.get() != nullptr && mTeachingStep >= 0 && mTeachingStep <= pt->getLessonCount();
}

bool CCastMachine::sayLesson()
{
  CTestEntryPtr pinfo = getCurrentTest();

  std::string text;
  
  if (mTeachingStep < 1 || !pinfo) {
    return false;
  }
  if (mTeachingStep > pinfo->getLessonCount()) {
    return false;
  }

  text = pinfo->getLessonText(mTeachingStep);
  if (text == "") {
    return false;
  }

  cast::cdl::WorkingMemoryAddress addr;
  addr.subarchitecture = mOptions["dialogue.sa"];
  addr.id = castComponent()->newDataID();

  //dlgice::asr::PhonStringPtr sayWhat = new dlgice::asr::PhonString();
  dlgice::asr::PhonStringPtr sayWhat = dlgice::asr::newPhonString(text);
  sayWhat->id = addr.id;
  //sayWhat->wordSequence = text;
  //sayWhat->confidenceValue = 1.0;

  long now = castComponent()->getCASTTime().s * 1000;
  sayWhat->ival->begin->msec = now;
  sayWhat->ival->end->msec = now + 500;
  report(MSSG("Tutor: " << text));
  castComponent()->log("Saying: '%s'.", text.c_str());

  { 
    std::lock_guard<std::mutex> lock(mWmCopyMutex);
    clearRobotResponses();
  }

  castComponent()->addToWorkingMemory(addr, sayWhat);
  return true;

#if 0 // OLD
  //m_bLearningExpected = false;
  if (! pinfo)
    sayWhat->wordSequence = "hi";
  else {
    // describe the object, or say hello if the attribute is not present.
    switch(stepId) {
      case 1: 
        if (pinfo->shape == "") sayWhat->wordSequence = "hello";
        else {
          sayWhat->wordSequence = "the object is " + pinfo->shape;
          report(MSSG("Tutor: the object is " << pinfo->shape));
          m_bLearningExpected = true;
        }
        break;
      case 2: 
        if (pinfo->color == "") sayWhat->wordSequence = "hello";
        else {
          sayWhat->wordSequence = "the object is " + pinfo->color;
          report(MSSG("Tutor: the object is " << pinfo->color));
          m_bLearningExpected = true;
        }
        break;
      default:
        sayWhat->wordSequence = "hello";
        break;
    }
  }
#endif
}

long CCastMachine::getRobotAnswerClass()
{
  CTestEntryPtr pinfo = getCurrentTest();

  if (mTeachingStep < 1 || !pinfo) {
    return 0;
  }
  if (mTeachingStep > pinfo->getLessonCount()) {
    return 0;
  }

  {
    std::lock_guard<std::mutex> lock(mWmCopyMutex);
    for (auto r : mRobotResponses) {
      long cls = pinfo->classifyResponse(mTeachingStep, r); 
      if (cls != 0) {
        return cls;
      }
    }
  }
  return 0;
}

void CCastMachine::writeMachineDescription(std::ostringstream& ss)
{
  {
    std::lock_guard<std::mutex> lock(mWmCopyMutex);
    for (auto v : mCount) {
      ss << v.first << ": " << v.second << "<br>\n";
    }
  }
}

void CCastMachine::start()
{
   castComponent()->addChangeFilter(
      cast::createLocalTypeFilter<VisionData::VisualObject>(cast::cdl::ADD),
      new cast::MemberFunctionChangeReceiver<CCastMachine>(
         this, &CCastMachine::onAdd_VisualObject)
      );
   castComponent()->addChangeFilter(
      cast::createLocalTypeFilter<VisionData::VisualObject>(cast::cdl::DELETE),
      new cast::MemberFunctionChangeReceiver<CCastMachine>(
         this, &CCastMachine::onDel_VisualObject)
      );
   castComponent()->addChangeFilter(
      cast::createLocalTypeFilter<VisionData::VisualObject>(cast::cdl::OVERWRITE),
      new cast::MemberFunctionChangeReceiver<CCastMachine>(
         this, &CCastMachine::onChange_VisualObject)
      );

   castComponent()->addChangeFilter(
      cast::createLocalTypeFilter<VisionData::ProtoObject>(cast::cdl::ADD),
      new cast::MemberFunctionChangeReceiver<CCastMachine>(
         this, &CCastMachine::onAdd_ProtoObject)
      );
   castComponent()->addChangeFilter(
      cast::createLocalTypeFilter<VisionData::ProtoObject>(cast::cdl::DELETE),
      new cast::MemberFunctionChangeReceiver<CCastMachine>(
         this, &CCastMachine::onDel_ProtoObject)
      );

   castComponent()->addChangeFilter(
      cast::createGlobalTypeFilter<dlgice::synthesize::SpokenOutputItem>(cast::cdl::ADD),
      new cast::MemberFunctionChangeReceiver<CCastMachine>(
         this, &CCastMachine::onAdd_SpokenItem)
      );

   castComponent()->addChangeFilter(
      cast::createLocalTypeFilter<VisionData::VisualLearningTask>(cast::cdl::ADD),
      new cast::MemberFunctionChangeReceiver<CCastMachine>(
         this, &CCastMachine::onAdd_LearningTask)
      );
   castComponent()->addChangeFilter(
      cast::createLocalTypeFilter<VisionData::VisualLearningTask>(cast::cdl::OVERWRITE),
      new cast::MemberFunctionChangeReceiver<CCastMachine>(
         this, &CCastMachine::onChange_LearningTask)
      );

   castComponent()->addChangeFilter(
      cast::createLocalTypeFilter<VisionData::VisualLearnerRecognitionTask>(cast::cdl::ADD),
      new cast::MemberFunctionChangeReceiver<CCastMachine>(
         this, &CCastMachine::onAdd_LearnerRecognitionTask)
      );
   castComponent()->addChangeFilter(
      cast::createLocalTypeFilter<VisionData::VisualLearnerRecognitionTask>(cast::cdl::OVERWRITE),
      new cast::MemberFunctionChangeReceiver<CCastMachine>(
         this, &CCastMachine::onChange_LearnerRecognitionTask)
      );

   mCount["VisualObject"] = 0;
   mCount["ProtoObject"] = 0;
   mCount["LearningTask-add"] = 0;
   mCount["LearningTask-done"] = 0;
   mCount["LearnerRecognitionTask-add"] = 0;
   mCount["LearnerRecognitionTask-done"] = 0;

#if 0
   loadEmptyScene();
   switchState(stStart);
#endif
#ifdef FEAT_VISUALIZATION
  mDisplay.setHtml(TIME_VIEW, "100", "<table border='1' style='border-collapse:collapse;'>");
  mDisplay.setHtml(TIME_VIEW, "199", "</table>");
#endif
}

long CCastMachine::getVisibleVisualObjectCount()
  // Enter: Locked mVisualObjects
{
   long count = 0;
   for(auto v : mVisualObjects) {
     if (! v.second.get()) continue;
     if (v.second->presence == VisionData::VopVISIBLE)
       ++count;
   }
   return count;
}

void CCastMachine::onAdd_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   auto pvo = castComponent()->getMemoryEntry<VisionData::VisualObject>(_wmc.address);
   {
     std::lock_guard<std::mutex> lock(mWmCopyMutex);
     mVisualObjects[_wmc.address] = pvo;
     mCount["VisualObject"] = getVisibleVisualObjectCount();
     checkReceivedEvent("::VisionData::VisualObject");
   }
}

void CCastMachine::onDel_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   {
     std::lock_guard<std::mutex> lock(mWmCopyMutex);
     mVisualObjects[_wmc.address] = VisionData::VisualObjectPtr();
     mCount["VisualObject"] = getVisibleVisualObjectCount();
     checkReceivedEvent("::VisionData::VisualObject");
   }
}

void CCastMachine::onChange_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   auto pvo = castComponent()->getMemoryEntry<VisionData::VisualObject>(_wmc.address);
   {
     std::lock_guard<std::mutex> lock(mWmCopyMutex);
     mVisualObjects[_wmc.address] = pvo;
     mCount["VisualObject"] = getVisibleVisualObjectCount();
     checkReceivedEvent("::VisionData::VisualObject");
   }
}

void CCastMachine::onAdd_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mCount["ProtoObject"] += 1;
    checkReceivedEvent("::VisionData::ProtoObject");
  }
}

void CCastMachine::onDel_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mCount["ProtoObject"] -= 1;
    checkReceivedEvent("::VisionData::ProtoObject");
  }
}

void CCastMachine::onAdd_SpokenItem(const cast::cdl::WorkingMemoryChange & _wmc)
{
  dlgice::synthesize::SpokenOutputItemPtr psaid =
    castComponent()->getMemoryEntry<dlgice::synthesize::SpokenOutputItem>(_wmc.address);
  {
    std::lock_guard<std::mutex> lock(mWmCopyMutex);
    addRobotResponse(psaid->phonString);
    checkReceivedEvent("::synthesize::SpokenOutputItem");
  }
}

void CCastMachine::onAdd_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mCount["LearningTask-add"] += 1;
    checkReceivedEvent("::VisionData::VisualLearningTask");
  }
}

void CCastMachine::onChange_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mCount["LearningTask-done"] += 1;
    checkReceivedEvent("::VisionData::VisualLearningTask");
  }
}

void CCastMachine::onAdd_LearnerRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mCount["LearnerRecognitionTask-add"] += 1;
    checkReceivedEvent("::VisionData::VisualLearnerRecognitionTask");
  }
}

void CCastMachine::onChange_LearnerRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mCount["LearnerRecognitionTask-done"] += 1;
    checkReceivedEvent("::VisionData::VisualLearnerRecognitionTask");
  }
}

}// namespace
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim
