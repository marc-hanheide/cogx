/**
 * @author Marko Mahnič
 * @created April 2012
 */
#ifndef _TESTING_CASTMACHINE_HPP_4F7C58AD_
#define _TESTING_CASTMACHINE_HPP_4F7C58AD_

#include "statemachine.hpp"

#include <VisionData.hpp>
#include <castutils/CastLoggerMixin.hpp>
#include <cast/architecture/ManagedComponent.hpp>
#include <cogxmath.h>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

#include <libplayerc++/playerc++.h>

#include <map>
#include <string>
#include <sstream>
#include <memory>
//#include <atomic>

namespace testing
{

class GObject // Copied from GazeboJuggler
{
public:
  std::string label;
  std::string gazeboName;
  std::string color;
  std::string shape;
  std::string type;
  cogx::Math::Vector3 loc;
  cogx::Math::Vector3 pose; // x=roll, y=pitch, z=yaw
  GObject(const std::string& label, const std::string& gazeboname)
  {
    this->label = label;
    this->gazeboName = gazeboname;
  }
};

class CTestEntry
{
public:
  virtual long getLessonCount()
  {
    return 3;
  }
  virtual std::string getLessonText(int numLesson)
  {
    if (numLesson <= getLessonCount()) {
      switch (numLesson % 3) {
        case 1: return "hello";
        case 2: return "hi";
        default: return "";
      }
    }
    return "";
  }
  virtual long classifyResponse(int numLesson, const std::string& response)
  {
    return 0;
  }
};
typedef std::shared_ptr<CTestEntry> CTestEntryPtr;

class CTeachTestEntry: public CTestEntry
{
public:
  std::string mLabel;
  std::string mColor;
  std::string mShape;
  CTeachTestEntry(std::string label, std::string color, std::string shape)
  {
    mLabel = label;
    mColor = color;
    mShape = shape;
  }

  virtual long getLessonCount()
  {
    return 2;
  }
  virtual std::string getLessonText(int numLesson);
  virtual long classifyResponse(int numLesson, const std::string& response);
};

class CCastComponentMixin
{
  cast::ManagedComponent* mpOwner;
protected:
  cast::ManagedComponent* castComponent()
  {
    return mpOwner;
  }

public:
  void setCastComponent(cast::ManagedComponent* pOwner)
  {
    mpOwner = pOwner;
  }

  virtual void configure(const std::map<std::string,std::string> & _config)
  {
  }

  virtual void start()
  {
  }
};

class CTester;
class CCastMachine: public CMachine, public CCastComponentMixin, public castutils::CCastLoggerMixin
{
private:
  std::string mPlayerHost;
  long mPlayerPort;
  std::unique_ptr<PlayerCc::PlayerClient> mpRobot;
  std::unique_ptr<PlayerCc::SimulationProxy> mpSim;
  void prepareObjects();
  std::vector<GObject> mObjects;
  std::string msObjectOnScene;
  std::vector<cogx::Math::Vector3> mLocations;
  std::map<std::string, std::string> mOptions;
  std::vector<CTestEntryPtr> mTestEntries;

private:
  // A mutex for protecting data that is transferred from wm filters to the main thread.
  std::mutex mWmCopyMutex;
  std::map<std::string, long> mCount;
  std::map<cast::cdl::WorkingMemoryAddress, VisionData::VisualObjectPtr> mVisualObjects;
  std::vector<std::string> mRobotResponses; // what the robot said

private:
  // Management of WM copies.
  long getVisibleVisualObjectCount();
  void addRobotResponse(const std::string &response);
  void clearRobotResponses();

private:
  void loadObjectsAndPlaces(const std::string& fname);
  void loadLearningAttributes(const std::string& fname);
  bool moveObject(const std::string& label, int placeIndex);

public:
#ifdef FEAT_VISUALIZATION
  cogx::display::CDisplayClient& mDisplay;
#endif
  long mSceneId;
  long mTeachingStep; // 0-start; 1-first lesson, ...
  long mStepsTaught;
  long mCurrentTest;

public:
  CCastMachine(CTester* pOwner);

  void configure(const std::map<std::string,std::string> & _config);
  void start(); /*override*/
  long getCount(const std::string& counter);
  bool verifyCount(const std::string& counter, long min, long max=-1);
  void report(const std::string& message);
  void report(std::ostringstream& what);
  void reportTimeout(const std::string& reason);
  CTestEntryPtr getCurrentTest();
  bool nextScene();
  bool loadScene();
  void clearScene();
  bool hasMoreLessons();
  bool nextLesson();
  bool sayLesson();
  long getRobotAnswerClass();

  void writeMachineDescription(std::ostringstream& ss) /*override*/;

private:
  void onAdd_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc);
  void onDel_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc);
  void onChange_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc);
  void onAdd_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc);
  void onDel_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc);
  void onAdd_SpokenItem(const cast::cdl::WorkingMemoryChange & _wmc);
  void onAdd_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc);
  void onChange_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc);
  void onAdd_LearnerRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
  void onChange_LearnerRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
};

}// namespace
#endif /* _TESTING_CASTMACHINE_HPP_4F7C58AD_ */
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim
