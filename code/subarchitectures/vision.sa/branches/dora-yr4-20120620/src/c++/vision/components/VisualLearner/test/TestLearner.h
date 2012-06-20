#ifndef __TEST_VISUAL_LEARNER_H__
#define __TEST_VISUAL_LEARNER_H__

#include <vector>
#include <string>
#include <time.h>

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>

class CTestRecognizer; // forward
class CTestCase
{
public:
   std::string m_name;
   CTestRecognizer *m_pOwner;
   CTestCase(std::string name, CTestRecognizer *pOwner) {
      m_pOwner = pOwner;
   }
   virtual void runOneStep() { }
   virtual void onStart() { }
   virtual void onExitComponent() { }
};

class CTestRecognizer: public cast::ManagedComponent
{
private:
   CTestCase *m_pTestCase;
   std::string testmode;
   int learningStepsComplete;
   std::vector<int> labels;
   
   std::vector<std::string> m_protoIDs;

public:
   // TODO Couters, should be RO for public!
   int m_ProtoObjects;
   int m_RecogTasks;
   int m_LearnTasks;

   bool dumpChanged_RecogTask;
   bool dumpNew_LearningTask;

private:

   // Proof of concept
   class CMyVideoClient: public cast::VideoClient {
      public:
      void receiveImages(const std::vector<Video::Image>& images)
      {
         printf("Got some images\n");
      }
      virtual ~CMyVideoClient() {}
   } Video;

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

   // Capture Learning Task events
   void onAdd_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc);
   void onDelete_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc);
   void onChange_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc);

   // not really used, just for logging
   void onAdd_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc);
   void onDelete_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc);

   // Some helpers for testing
   void _test_addRecognitionTask();
   void _test_performLearningStep(int issued, std::string protoId);
public:
   VisionData::ProtoObjectPtr loadFakeProtoObject();
   using cast::ManagedComponent::sleepComponent;
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

#endif /* end of include guard: __TEST_VISUAL_LEARNER_H__ */
// vim:set fileencoding=utf-8 sw=3 ts=8 et:
