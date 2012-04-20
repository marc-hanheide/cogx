/**
 * @author Marko Mahnič
 * @created April 2012
 */
#ifndef _TESTING_CASTMACHINE_HPP_4F7C58AD_
#define _TESTING_CASTMACHINE_HPP_4F7C58AD_

#include "statemachine.hpp"

#include <map>
#include <string>
#include <sstream>

namespace testing
{

class CCastMachine: public CMachine
{
  std::map<std::string, long> mCount;
public:
  long mSceneId;
  long mTeachingStep;
  long mStepsTaught;
  long mCurrentTest;
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
