#ifndef __TEST_VISUAL_LEARNER_H__
#define __TEST_VISUAL_LEARNER_H__

#include <vector>
#include <string>
#include <time.h>

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>

class CTestRecognizer: public cast::ManagedComponent
{
private:
   std::string testmode;
   int nRequests;

   // Proof of concept
   class CMyVideoClient: public cast::VideoClient {
      void receiveImages(const std::vector<Video::Image>& images)
      {
         printf("Got some images\n");
      }
   } Video;

public:
   CTestRecognizer();

protected:
   virtual void start();
   virtual void runComponent();
   void configure(const std::map<std::string,std::string> & _config);

   // Capture Recognition Task events
   void onAddRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
   void onDeleteRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
   void onChangeRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);

   // not really used, just for logging
   void onAddProtoObject(const cast::cdl::WorkingMemoryChange & _wmc);
   void onAddAttrObject(const cast::cdl::WorkingMemoryChange & _wmc);

   // Some helpers for testing
   void _test_addRecognitionTask();
   VisionData::ProtoObjectPtr loadFakeProtoObject();
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
