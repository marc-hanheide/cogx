#ifndef __TEST_DRIVERS_HPP__
#define __TEST_DRIVERS_HPP__

//#include <vector>
//#include <string>

//#include <cast/architecture/ManagedComponent.hpp>
//#include <VisionData.hpp>

#include "TestLearner.h"

class CTestCase_Standalone: public CTestCase
{
   int m_nCallsLeft;
   void issueRequest();
   void onChange_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
public:
   CTestCase_Standalone(std::string name, CTestRecognizer *pOwner);
   void onStart();
   void runOneStep();
};

class CTestCase_FakeProto: public CTestCase
{
   int m_nCallsLeft;
   std::vector<std::string> m_protoIDs;
   void issueRequest();
public:
  CTestCase_FakeProto(std::string name, CTestRecognizer *pOwner);
  void onStart();
  void runOneStep();
};

class CTestCase_Learning: public CTestCase
{
   int m_stepsComplete;
   int m_issued;
   std::vector<std::string> m_protoIDs;
   std::vector<std::string> labels;
   void performLearningStep(int issued, std::string protoId);
   void onChange_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
   void onChange_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc);
public:
  CTestCase_Learning(std::string name, CTestRecognizer *pOwner);
  void onStart();
  void runOneStep();
};

class CTestCase_SavedExamples: public CTestCase
{
   std::vector<std::string> m_protoIDs;
   void issueRequest();
public:
  CTestCase_SavedExamples(std::string name, CTestRecognizer *pOwner);
  void onStart();
  void runOneStep();
};
#endif
