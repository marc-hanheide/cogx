/**
 * @author Marko Mahnič
 * @created April 2012
 */
#include "castmachine.hpp"
#include "tester.hpp"

#include <cast/architecture/ChangeFilterFactory.hpp>

#include <sstream>
#include <fstream>

namespace testing {

#define MSSG(streamexpr)  ((std::ostringstream&)(std::ostringstream() << std::flush << streamexpr))

CCastMachine::CCastMachine(CTester* pOwner)
#ifdef FEAT_VISUALIZATION
  : mDisplay(pOwner->mDisplay)
#endif
{
  setCastComponent(pOwner);
  setLoggingComponent(pOwner);
  mPlayerHost = "localhost";
  mPlayerPort = 6665;
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

  mpRobot = std::unique_ptr<PlayerCc::PlayerClient>(new PlayerCc::PlayerClient(mPlayerHost, mPlayerPort));
  mpSim = std::unique_ptr<PlayerCc::SimulationProxy>(new PlayerCc::SimulationProxy(&*mpRobot, 0));
  log("Connected to Player %x", &*mpRobot);

  prepareObjects();
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
bool CCastMachine::getCurrentTest()
{
  return false;
#if 0
  if (mCurrentTest < 0 || mCurrentTest >= mTestEntries.size())
    return 0;
  return mTestEntries[mCurrentTest];
#endif
}

bool CCastMachine::nextScene()
{
  return true;
}

bool CCastMachine::loadScene()
{
  return true;
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

bool CCastMachine::sayLesson(long stepId)
{
  return stepId < 2;
}

void CCastMachine::clearScene()
{
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

void CCastMachine::writeMachineDescription(std::ostringstream& ss)
{
  for (auto v : mCount) {
    ss << v.first << ": " << v.second << "<br>\n";
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

   mCount["VisualObject"] = 0;
   mCount["ProtoObject"] = 0;

#if 0
   m_pOwner->addChangeFilter(
      createLocalTypeFilter<VisualLearningTask>(cdl::ADD),
      new MemberFunctionChangeReceiver<CCastMachine>(
         this, &CCastMachine::onAdd_LearningTask)
      );
   m_pOwner->addChangeFilter(
      createLocalTypeFilter<VisualLearningTask>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CCastMachine>(
         this, &CCastMachine::onChange_LearningTask)
      );
   m_pOwner->addChangeFilter(
      createGlobalTypeFilter<synthesize::SpokenOutputItem>(cdl::ADD),
      new MemberFunctionChangeReceiver<CCastMachine>(
         this, &CCastMachine::onAdd_SpokenItem)
      );

   loadEmptyScene();
   switchState(stStart);
#endif
}

void CCastMachine::onAdd_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
#if 0
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_ObjectCount++;
   }
   m_EventMonitor.notify();
#endif
   checkReceivedEvent("::VisionData::VisualObject");

   auto pvo = castComponent()->getMemoryEntry<VisionData::VisualObject>(_wmc.address);
   mVisualObjects[_wmc.address] = pvo;

   long count = 0;
   for(auto v : mVisualObjects) {
     if (! v.second.get()) continue;
     if (v.second->presence == VisionData::VopVISIBLE)
       ++count;
   }
   mCount["VisualObject"] = count;
}

void CCastMachine::onDel_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
#if 0
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_ObjectCount--;
   }
   m_EventMonitor.notify();
#endif
   checkReceivedEvent("::VisionData::VisualObject");

   mVisualObjects[_wmc.address] = VisionData::VisualObjectPtr();

   long count = 0;
   for(auto v : mVisualObjects) {
     if (! v.second.get()) continue;
     if (v.second->presence == VisionData::VopVISIBLE)
       ++count;
   }
   mCount["VisualObject"] = count;
}

void CCastMachine::onChange_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
#if 0
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_ObjectCount--;
   }
   m_EventMonitor.notify();
#endif
   checkReceivedEvent("::VisionData::VisualObject");

   auto pvo = castComponent()->getMemoryEntry<VisionData::VisualObject>(_wmc.address);
   mVisualObjects[_wmc.address] = pvo;

   long count = 0;
   for(auto v : mVisualObjects) {
     if (! v.second.get()) continue;
     if (v.second->presence == VisionData::VopVISIBLE)
       ++count;
   }
   mCount["VisualObject"] = count;
}

void CCastMachine::onAdd_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
#if 0
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_ObjectCount++;
   }
   m_EventMonitor.notify();
#endif
   mCount["ProtoObject"] += 1;
   checkReceivedEvent("::VisionData::ProtoObject");
}

void CCastMachine::onDel_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
#if 0
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_ObjectCount--;
   }
   m_EventMonitor.notify();
#endif
   mCount["ProtoObject"] -= 1;
   checkReceivedEvent("::VisionData::ProtoObject");
}

#if 0
void CCastMachine::onAdd_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   report(MSSG("Learnig task added: " << _wmc.address));
   //{
   //   IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
   //   m_LearnTaskCount++;
   //}
   //m_EventMonitor.notify();
}

void CCastMachine::onChange_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   report(MSSG("Learnig task changed: " << _wmc.address));
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_LearnTaskCount++;
   }
   m_EventMonitor.notify();
}

void CCastMachine::onAdd_SpokenItem(const cast::cdl::WorkingMemoryChange & _wmc)
{
   synthesize::SpokenOutputItemPtr psaid = m_pOwner->getMemoryEntry<synthesize::SpokenOutputItem>(_wmc.address);
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_RobotResponse = psaid->phonString;
   }
   m_EventMonitor.notify();
}
#endif

}// namespace
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim
