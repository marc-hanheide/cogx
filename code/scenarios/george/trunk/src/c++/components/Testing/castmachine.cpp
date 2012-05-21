/**
 * @author Marko Mahnič
 * @created April 2012
 */
#include "castmachine.hpp"
#include "tester.hpp"

#include <dialogue.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

#include <sstream>
#include <fstream>

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
  // TODO parse the answer
  return 1; // for now accept anything; TODO constant YES
}

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
    loadObjectsAndPlaces(it->second);
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

  for (int i = 0; i < 3; i++) {
    auto pe = new CTeachTestEntry(mObjects[i].label, "orange", "elongated");
    mTestEntries.push_back(CTestEntryPtr(pe));
  }
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

      // TODO: objects have additional fields: color, shape, type

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

const double OFF = 121231e99; 
void CCastMachine::prepareObjects()
{
  double time;
  // Discover and keep valid objects
  std::vector<GObject> objs = mObjects;
  std::vector<GObject> notthere;
  mObjects.clear();
  for (int i = 0; i < objs.size(); i++) {
    GObject& o = objs[i];
    o.loc.x = OFF;
    mpSim->GetPose3d((char*)o.gazeboName.c_str(), o.loc.x, o.loc.y, o.loc.z, o.pose.x, o.pose.y, o.pose.z, time);
    if (o.loc.x == OFF) {
      println(" *** Object '%s' is not in the scene.", o.label.c_str());
      notthere.push_back(o);
      continue;
    }
    o.loc.x = 100 + i * 10;
    o.loc.y = 100 + i * 10;
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
  printf("%s", message.c_str());
}

void CCastMachine::reportTimeout(const std::string& reason)
{
  report("Timeout (" + reason + ")");
}

bool CCastMachine::verifyCount(const std::string& counter, long min, long max)
{ 
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

bool CCastMachine::moveObject(const std::string& label, int placeIndex)
{
  for(int i = 0; i < mObjects.size(); i++) {
    GObject& o = mObjects[i];
    if (o.label != label)
      continue;
    if (placeIndex < 0 || placeIndex >= mLocations.size()) {
      o.loc.x = 100 + i * 10;
      o.loc.y = 100 + i * 10;
    }
    else
      o.loc = mLocations[placeIndex];
    mpSim->SetPose3d((char*)o.gazeboName.c_str(), o.loc.x, o.loc.y, o.loc.z, o.pose.x, o.pose.y, o.pose.z);
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
    bool rv = moveObject(pe->mLabel, 0);
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

  dlgice::asr::PhonStringPtr sayWhat = new dlgice::asr::PhonString();
  sayWhat->id = addr.id;
  sayWhat->wordSequence = text;
  report(MSSG("Tutor: " << text));
  castComponent()->log("Saying: '%s'.", text.c_str());

  { 
    std::lock_guard(mWmCopyMutex);
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
    std::lock_guard(mWmCopyMutex);
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
    std::lock_guard(mWmCopyMutex);
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

   mCount["VisualObject"] = 0;
   mCount["ProtoObject"] = 0;
   mCount["LearnigTask"] = 0;

#if 0
   loadEmptyScene();
   switchState(stStart);
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
}

void CCastMachine::onAdd_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   auto pvo = castComponent()->getMemoryEntry<VisionData::VisualObject>(_wmc.address);
   {
     std::lock_guard(mWmCopyMutex);
     mVisualObjects[_wmc.address] = pvo;
     mCount["VisualObject"] = getVisibleVisualObjectCount();
     checkReceivedEvent("::VisionData::VisualObject");
   }
}

void CCastMachine::onDel_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   {
     std::lock_guard(mWmCopyMutex);
     mVisualObjects[_wmc.address] = VisionData::VisualObjectPtr();
     mCount["VisualObject"] = getVisibleVisualObjectCount();
     checkReceivedEvent("::VisionData::VisualObject");
   }
}

void CCastMachine::onChange_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   auto pvo = castComponent()->getMemoryEntry<VisionData::VisualObject>(_wmc.address);
   {
     std::lock_guard(mWmCopyMutex);
     mVisualObjects[_wmc.address] = pvo;
     mCount["VisualObject"] = getVisibleVisualObjectCount();
     checkReceivedEvent("::VisionData::VisualObject");
   }
}

void CCastMachine::onAdd_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    std::lock_guard(mWmCopyMutex);
    mCount["ProtoObject"] += 1;
    checkReceivedEvent("::VisionData::ProtoObject");
  }
}

void CCastMachine::onDel_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    std::lock_guard(mWmCopyMutex);
    mCount["ProtoObject"] -= 1;
    checkReceivedEvent("::VisionData::ProtoObject");
  }
}

void CCastMachine::onAdd_SpokenItem(const cast::cdl::WorkingMemoryChange & _wmc)
{
  dlgice::synthesize::SpokenOutputItemPtr psaid =
    castComponent()->getMemoryEntry<dlgice::synthesize::SpokenOutputItem>(_wmc.address);
  {
    std::lock_guard(mWmCopyMutex);
    addRobotResponse(psaid->phonString);
    checkReceivedEvent("::synthesize::SpokenOutputItem");
  }
}

void CCastMachine::onAdd_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    std::lock_guard(mWmCopyMutex);
    mCount["LearnigTask-add"] += 1;
  }
}

void CCastMachine::onChange_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    std::lock_guard(mWmCopyMutex);
    mCount["LearnigTask"] += 1;
  }
}

}// namespace
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim
