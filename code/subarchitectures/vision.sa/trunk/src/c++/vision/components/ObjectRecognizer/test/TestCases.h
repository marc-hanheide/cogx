#ifndef __TEST_DRIVERS_HPP__
#define __TEST_DRIVERS_HPP__

#include "TestRecognizer.h"

class CTestCase_Server: public CTestCase
{
public:
   CTestCase_Server(std::string name, CTestRecognizer *pOwner);
   void onStart();
   void runOneStep();
};

class CTestCase_WmResponder: public CTestCase
{
   // TODO: Events moved from CTestRecognizer to CTestCase_WmResponder
   // Capture Recognition Task events
   //void onAdd_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
   //void onDelete_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
   //void onChange_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);

   // Trigger recognition on some events
   //void onAdd_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc);

   // Some helpers for testing
   //void _test_addRecognitionTask();
public:
   CTestCase_WmResponder(std::string name, CTestRecognizer *pOwner);
   void onStart();
   void runOneStep();
};

#endif
