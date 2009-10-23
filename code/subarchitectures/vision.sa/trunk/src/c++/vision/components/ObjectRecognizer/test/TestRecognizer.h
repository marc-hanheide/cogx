#ifndef RECOGNIZER_HPP_
#define RECOGNIZER_HPP_

#include <vector>
#include <string>
#include <time.h>

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>

class CTestRecognizer: public cast::ManagedComponent, public cast::VideoClient
{
private:
   int testmode;
   int nTasks;

public:
   CTestRecognizer();

protected:
   virtual void start();
   virtual void runComponent();
   void configure(const std::map<std::string,std::string> & _config);

   // Capture Recognition Task events
   void onAdd_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
   void onDelete_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
   void onChange_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);

   // Trigger recognition on some events
   void onAdd_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc);

   // Some helpers for testing
   void _test_addRecognitionTask();
};

//class CommandListener: public cast::WorkingMemoryChangeReceiver 
//{
//public:
//   CommandListener(CRecognizer & _component) : m_component(_component) {}

//   void workingMemoryChanged(const cast::cdl::WorkingMemoryChange & _wmc)
//   {
//      if(m_component.commandOverwritten(_wmc)) {
//         //remove this filter
//         m_component.removeChangeFilter(this, cast::cdl::DELETERECEIVER);	    
//      }
//   }

//private:
//   CRecognizer & m_component;  
//};

#endif /* end of include guard: RECOGNIZER_HPP_ */
// vim:set fileencoding=utf-8 sw=3 ts=8 et:
