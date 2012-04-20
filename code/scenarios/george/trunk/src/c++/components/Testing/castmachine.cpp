/**
 * @author Marko Mahnič
 * @created April 2012
 */
#include "castmachine.hpp"

#include <sstream>

namespace testing {

#define MSSG(streamexpr)  ((std::ostringstream&)(std::ostringstream() << std::flush << streamexpr))

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

# if 0
void CCastMachine::onStart()
{
   m_pOwner->addChangeFilter(
      createLocalTypeFilter<VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<CCastMachine>(
         this, &CCastMachine::onAdd_VisualObject)
      );
   m_pOwner->addChangeFilter(
      createLocalTypeFilter<VisualObject>(cdl::DELETE),
      new MemberFunctionChangeReceiver<CCastMachine>(
         this, &CCastMachine::onDel_VisualObject)
      );
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
}

void CCastMachine::onAdd_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_ObjectCount++;
   }
   m_EventMonitor.notify();
}

void CCastMachine::onDel_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_ObjectCount--;
   }
   m_EventMonitor.notify();
}

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
