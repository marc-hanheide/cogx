/**
 * @author Marko Mahnič
 * @created April 2012
 */
#ifndef _TESTING_CASTMACHINE_HPP_4F7C58AD_
#define _TESTING_CASTMACHINE_HPP_4F7C58AD_

#include "statemachine.hpp"

#include <castutils/CastLoggerMixin.hpp>
#include <cast/architecture/ManagedComponent.hpp>
#include <cogxmath.h>

#include <libplayerc++/playerc++.h>

#include <map>
#include <string>
#include <sstream>
#include <memory>

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

class CCastComponentMixin
{
  cast::ManagedComponent* mpOwner;
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

class CCastMachine: public CMachine, public CCastComponentMixin, public castutils::CCastLoggerMixin
{
  std::string mPlayerHost;
  long mPlayerPort;
  std::unique_ptr<PlayerCc::PlayerClient> mpRobot;
  std::unique_ptr<PlayerCc::SimulationProxy> mpSim;
  std::map<std::string, long> mCount;
  void prepareObjects();
  std::vector<GObject> mObjects;
  std::vector<cogx::Math::Vector3> mLocations;

private:
  void loadObjectsAndPlaces(const std::string& fname);

public:
  CCastMachine(cast::ManagedComponent* pOwner)
  {
    setCastComponent(pOwner);
    setLoggingComponent(pOwner);
    mPlayerHost = "localhost";
    mPlayerPort = 6665;
  }
  long mSceneId;
  long mTeachingStep;
  long mStepsTaught;
  long mCurrentTest;
  void configure(const std::map<std::string,std::string> & _config);
  long getCount(const std::string& counter);
  bool verifyCount(const std::string& counter, long min, long max=-1);
  void report(const std::string& message);
  void report(std::ostringstream& what);
  void reportTimeout(const std::string& reason);
  bool getCurrentTest();
  bool nextScene();
  bool loadScene();
  void clearScene();
  bool sayLesson(long stepId);
};

}// namespace
#endif /* _TESTING_CASTMACHINE_HPP_4F7C58AD_ */
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim
