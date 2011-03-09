#include "gy2article.h"

#include "StringFmt.h"
#include <VisionData.hpp>
#include <dialogue.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

#include <fstream>
#include <string>
#include <sstream>
#include <sys/time.h>
#include <unistd.h>

using namespace std;
using namespace cast;
using namespace VisionData;
using namespace de::dfki::lt::tr::dialogue::slice;

#define MSSG(streamexpr)  (std::ostringstream&)(std::ostringstream() << flush << streamexpr)

// Return time in milliseconds
static double now()
{
   struct timeval tm;
   gettimeofday(&tm, NULL);
   double now = tm.tv_sec + tm.tv_usec * 1e-6;

   static double start = 0;
   if (start == 0) start = now;

   return now - start;
}

#define STATE(name) m_stateNames[name] = #name;

CGeorgeY2Article::CGeorgeY2Article(string name, CTestRecognizer *pOwner)
   : CTestCase(name, pOwner)
{
   m_State = stTableEmpty;
   m_ObjectCount = 0;
   m_LearnTaskCount = 0;
   m_RobotResponse = "";
   m_imageDir = "";
   m_currentTest = 0;
   options["dialogue.sa"] = "dialogue"; // hardcoded default SA name; change with --dialogue-sa
   options["video.sa"] = pOwner->getSubarchitectureID(); // change with --video-sa
   m_timeout = now();
   m_waitOnEnter = now();

   STATE(stStart);
   STATE(stTableEmpty);
   STATE(stWaitToAppear);
   STATE(stObjectOn);
   STATE(stTeaching);
   STATE(stWaitForResponse);
   STATE(stWaitForLearningTask);
   STATE(stEndOfTeaching);
   STATE(stWaitToDisappear);
   STATE(stFinished);
   STATE(stTimedOut);
   STATE(stUnloadScene);
}

CGeorgeY2Article::~CGeorgeY2Article()
{
   m_testEntries.delete_all();
}

void CGeorgeY2Article::report(std::ostringstream& what)
{
   m_pOwner->println("%.2f %s", now(), what.str().c_str());
}

void CGeorgeY2Article::report(const std::string& what)
{
   m_pOwner->println("%.2f %s", now(), what.c_str());
}

void CGeorgeY2Article::error(std::ostringstream& what)
{
   m_pOwner->error("%.2f %s", now(), what.str().c_str());
}

void CGeorgeY2Article::error(const std::string& what)
{
   m_pOwner->error("%.2f %s", now(), what.c_str());
}

static int _index(const vector<string>& vec, const string& val)
{
   for(size_t i = 0; i < vec.size(); i++) {
      if (vec[i] == val) return i;
   }
   return -1;
}

void CGeorgeY2Article::configure(const std::map<std::string,std::string> & _config)
{
   map<string,string>::const_iterator it;

   // CONFIG: --index
   // TYPE: filename, required
   // Index file with attributes and filenames for the left and right video image.
   // The first line of the index file contains the header with filed names.
   // Fields are separated by a single tab character.
   // The following fields will be used: left, right, shape1, color.
   // The values for shape1 and color must be supported in Matlab code of the VisualLearner.
   if((it = _config.find("--index")) != _config.end())
   {
      m_testEntries.delete_all();
      ifstream f(it->second.c_str(), ifstream::in);
      if (f.fail()) {
         m_pOwner->error("Failed to open index file '%s'", it->second.c_str());
         return;
      }
      string line;
      getline(f, line);
      vector<string> header = _s_::split(line, "\t");
      int fiLeft = _index(header, "left");
      int fiRight = _index(header, "right");
      int fiShape = _index(header, "shape1");
      int fiColor = _index(header, "color");
      while(f.good()) {
         getline(f, line);
         vector<string> exmpl = _s_::split(line, "\t");
         int nf = exmpl.size();
         CTestEntry *pdata = new CTestEntry();
         if (fiLeft >= 0 && fiLeft < nf) pdata->videoLeft = exmpl[fiLeft];
         if (fiRight >= 0 && fiRight < nf) pdata->videoRight = exmpl[fiRight];
         if (fiShape >= 0 && fiShape < nf) pdata->shape = exmpl[fiShape];
         if (fiColor >= 0 && fiColor < nf) pdata->color = exmpl[fiColor];
         if (pdata->isValid()) {
            // name = filename to first dot
            vector<string> parts = _s_::split(pdata->videoLeft, "/");
            parts = _s_::split(parts.back(), ".");
            pdata->name = parts.front();
            m_testEntries.push_back(pdata);
         }
         else delete pdata;
      }
      f.close();
   }

   if (m_testEntries.size() < 1) {
      throw runtime_error(string("No valid tests were provided with --index"));
   }

   // CONFIG: --imagedir
   // TYPE: directory
   // DEFAULT: ""
   // Directory with image files listed in --index.
   if((it = _config.find("--imagedir")) != _config.end())
   {
      m_imageDir = it->second;
   }

   // CONFIG: --dialogue-sa
   // TYPE: string (subarchitecture-id)
   // DEFAULT: "dialogue"
   // The subarchitecture id in which ASR is running
   if((it = _config.find("--dialogue-sa")) != _config.end())
   {
      if (it->second == "") options["dialogue.sa"] = m_pOwner->getSubarchitectureID();
      else options["dialogue.sa"] = it->second;
   }

   // CONFIG: --video-sa
   // TYPE: string (subarchitecture-id)
   // DEFAULT: ""
   // The subarchitecture id in which the OpenCvImgSeqServer video server is running.
   // The default is empty which means that the video server is running in the same
   // SA as this component.
   if((it = _config.find("--video-sa")) != _config.end())
   {
      if (it->second == "") options["video.sa"] = m_pOwner->getSubarchitectureID();
      else options["video.sa"] = it->second;
   }
}

void CGeorgeY2Article::onStart()
{
   m_pOwner->addChangeFilter(
      createLocalTypeFilter<VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<CGeorgeY2Article>(
         this, &CGeorgeY2Article::onAdd_VisualObject)
      );
   m_pOwner->addChangeFilter(
      createLocalTypeFilter<VisualObject>(cdl::DELETE),
      new MemberFunctionChangeReceiver<CGeorgeY2Article>(
         this, &CGeorgeY2Article::onDel_VisualObject)
      );
   m_pOwner->addChangeFilter(
      createLocalTypeFilter<VisualLearningTask>(cdl::ADD),
      new MemberFunctionChangeReceiver<CGeorgeY2Article>(
         this, &CGeorgeY2Article::onAdd_LearningTask)
      );
   m_pOwner->addChangeFilter(
      createLocalTypeFilter<VisualLearningTask>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CGeorgeY2Article>(
         this, &CGeorgeY2Article::onChange_LearningTask)
      );
   m_pOwner->addChangeFilter(
      createGlobalTypeFilter<synthesize::SpokenOutputItem>(cdl::ADD),
      new MemberFunctionChangeReceiver<CGeorgeY2Article>(
         this, &CGeorgeY2Article::onAdd_SpokenItem)
      );

   switchState(stStart);
}

void CGeorgeY2Article::onAdd_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_ObjectCount++;
   }
   m_EventMonitor.notify();
}

void CGeorgeY2Article::onAdd_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   report(MSSG("Learnig task added: " << _wmc.address));
   //{
   //   IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
   //   m_LearnTaskCount++;
   //}
   //m_EventMonitor.notify();
}

void CGeorgeY2Article::onChange_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   report(MSSG("Learnig task changed: " << _wmc.address));
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_LearnTaskCount++;
   }
   m_EventMonitor.notify();
}

void CGeorgeY2Article::onDel_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_ObjectCount--;
   }
   m_EventMonitor.notify();
}

void CGeorgeY2Article::onAdd_SpokenItem(const cast::cdl::WorkingMemoryChange & _wmc)
{
   synthesize::SpokenOutputItemPtr psaid = m_pOwner->getMemoryEntry<synthesize::SpokenOutputItem>(_wmc.address);
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_RobotResponse = psaid->phonString;
   }
   m_EventMonitor.notify();
}

void CGeorgeY2Article::switchState(int newState)
{
   if (newState == m_State) return;

   if (newState == stTeaching) {
      if (m_State == stObjectOn) {
         m_TeachingStep = 0;
      }
   }

   // from stTeaching
   if (newState == stWaitForResponse) {
      m_RobotResponse = "";
      // if all is well, next will be stWaitForLearningTask
      m_LearnTaskCount = 0;
   }

   if (newState == stTimedOut) {
      error(MSSG("STATE " << m_stateNames[m_State] << " TIMED OUT"));
   }

   m_State = newState;

   double timeout = 0;
   double enterWait = 0;
   switch(m_State) {
      default:
         timeout = 15;
         break;
      case stStart:            // Matlab starting
         timeout = 30;
         break;
      case stWaitToAppear:     // PPO
      case stWaitToDisappear:  // PPO
         timeout = 30;
         break;
      case stUnloadScene:
         if (m_ObjectCount) enterWait = 3; // Something may still be using the VO
         break;
      case stTableEmpty:
      case stObjectOn:
         enterWait = 3;
         break;
      case stWaitForLearningTask: // Motivation/Planner/Execution
         timeout = 30;
         break;
   }
   m_waitOnEnter = now() + enterWait;
   m_timeout = m_waitOnEnter + timeout;

   report("**********");
   report(MSSG("NEW STATE " << m_stateNames[m_State] << " (to=" << timeout << "s)"));
}

void CGeorgeY2Article::verifyCount(int count)
{
   static int lastReport = 0;
   int curReport = m_currentTest * 100 + m_ObjectCount;
   if (count == 0) {
      if (m_ObjectCount != 0) {
         if (lastReport != curReport)
            report(MSSG("The scene should be empty, but I see " << m_ObjectCount << " objects."));
         lastReport = curReport;
      }
   }
   else if (count == 1) {
      if (m_ObjectCount != 1) {
         if (lastReport != curReport)
            report(MSSG("There should be one object on the scene, but I see " << m_ObjectCount << "."));
         lastReport = curReport;
      }
   }
}

CTestEntry* CGeorgeY2Article::getCurrentTest()
{
   if (m_currentTest < 0 || m_currentTest >= m_testEntries.size())
      return 0;
   return m_testEntries[m_currentTest];
}

bool CGeorgeY2Article::nextTest()
{
   m_currentTest++;
   return (m_currentTest >= 0 && m_currentTest < m_testEntries.size());
}

bool CGeorgeY2Article::isTimedOut()
{
   return now() > m_timeout;
}

void CGeorgeY2Article::loadScene()
{
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
   addr.subarchitecture = options["video.sa"];
   addr.id = m_pOwner->newDataID();

   m_pOwner->addToWorkingMemory(addr, pseq);
}

void CGeorgeY2Article::loadEmptyScene()
{
   Video::VideoSequenceInfoPtr pseq = new Video::VideoSequenceInfo();

   pseq->fileTemplates = m_imageDir + "/empty-l.png" + " " + m_imageDir + "/empty-r.png";
   pseq->start = 0;
   pseq->end = 0;
   pseq->step = 0;
   pseq->loop = true;
   pseq->repeatFrame = 5;

   cdl::WorkingMemoryAddress addr;
   addr.subarchitecture = options["video.sa"];
   addr.id = m_pOwner->newDataID();

   m_pOwner->addToWorkingMemory(addr, pseq);
   report("SCENE: empty");
}

bool CGeorgeY2Article::performNextTeachingStep()
{
   m_TeachingStep++;
   if (m_TeachingStep > 2) return false;

   CTestEntry *pinfo = getCurrentTest();

   cdl::WorkingMemoryAddress addr;
   addr.subarchitecture = options["dialogue.sa"];
   addr.id = m_pOwner->newDataID();

   asr::PhonStringPtr sayWhat = new asr::PhonString();
   sayWhat->id = addr.id;
   m_bLearningExpected = false;

   if (! pinfo)
      sayWhat->wordSequence = "hi";
   else {
      // describe the object, or say hello if the attribute is not present.
      switch(m_TeachingStep) {
         case 1: 
            if (pinfo->shape == "") sayWhat->wordSequence = "hello";
            else {
               sayWhat->wordSequence = "this is " + pinfo->shape;
               report(MSSG("Tutor: this is " << pinfo->shape));
               m_bLearningExpected = true;
            }
            break;
         case 2: 
            if (pinfo->color == "") sayWhat->wordSequence = "hello";
            else {
               sayWhat->wordSequence = "this is " + pinfo->color;
               report(MSSG("Tutor: this is " << pinfo->color));
               m_bLearningExpected = true;
            }
            break;
         default:
            sayWhat->wordSequence = "hello";
            break;
      }
   }

   m_pOwner->addToWorkingMemory(addr, sayWhat);

   return true;
}

void CGeorgeY2Article::runOneStep()
{
   // SYNC: Lock the monitor
   IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
   bool mustWait = false;
   mustWait = mustWait
      || (m_State == stStart)
      || (m_State == stTableEmpty && m_ObjectCount > 0) // ghosts
      || (m_State == stWaitToAppear && m_ObjectCount < 1)
      || (m_State == stTeaching && m_ObjectCount < 1)   // object unstable
      || (m_State == stWaitToDisappear && m_ObjectCount > 0)
      || (m_State == stWaitForResponse && m_RobotResponse.length() < 1);

   // SYNC: Unlock the monitor and wait for notify() or timeout
   if (mustWait)
      m_EventMonitor.timedWait(IceUtil::Time::seconds(2));
   else 
      m_EventMonitor.timedWait(IceUtil::Time::milliSeconds(2)); // give a chance to other thread, anyway
   // SYNC: Continue with a locked monitor

   if (now() < m_waitOnEnter)
      return; // return to unlock the monitor

   if (mustWait) {
      static int reported = -1;
      static int count = 0;
      count++;
      if (reported != m_State || count >= 10) {
         report(MSSG("STATE " << m_stateNames[m_State]));
         count = 0;
         reported = m_State;
      }
   }

   switch (m_State) {
      case stStart:
         if (isTimedOut()) {
            m_currentTest = 110; // XXX DEBUGGING
            switchState(stTableEmpty);
         }
         break;

      case stTableEmpty:
         verifyCount(0);
         if (m_ObjectCount < 1) {
            if (! getCurrentTest()) {
               switchState(stFinished);
            }
            else {
               loadScene();
               switchState(stWaitToAppear);
            }
         }
         break;

      case stWaitToAppear:
         if (m_ObjectCount > 0) {
            switchState(stObjectOn);
         }
         else if (isTimedOut()) {
            switchState(stTimedOut);
         }
         break;

      case stObjectOn:
         verifyCount(1);
         switchState(stTeaching);
         break;

      // While there are teaching steps to perform, change between stTeaching and stWaitForResponse.
      // After last step, (remove the object video sequence and) change to stEndOfTeaching
      case stTeaching:
         verifyCount(1);
         if (performNextTeachingStep())
            switchState(stWaitForResponse);
         else
            switchState(stEndOfTeaching);
         break;

      case stWaitForResponse:
         verifyCount(1);
         if (m_RobotResponse.length() > 0) {
            report(MSSG("Robot says: " << m_RobotResponse));
            if (m_bLearningExpected)
               switchState(stWaitForLearningTask);
            else
               switchState(stTeaching);
         }
         else if (isTimedOut()) {
            switchState(stTimedOut);
         }
         break;

      case stWaitForLearningTask:
         verifyCount(1);
         if (m_LearnTaskCount > 0) {
            report("A Learning Task was detected.");
            switchState(stTeaching);
         }
         else if (isTimedOut()) {
            switchState(stTimedOut);
         }
         break;

      case stEndOfTeaching:
         switchState(stUnloadScene);
         break;

      case stUnloadScene:
         loadEmptyScene();
         switchState(stWaitToDisappear);
         break;

      case stWaitToDisappear:
         if (m_ObjectCount <= 0) {
            nextTest();
            switchState(stTableEmpty);
         }
         else if (isTimedOut()) {
            switchState(stTimedOut);
         }
         break;

      case stFinished:
         m_pOwner->sleepComponent(100);
         break;

      case stTimedOut:
         switchState(stUnloadScene);
         break;
   }
}
// vim: set sw=3 ts=8 et :vim
