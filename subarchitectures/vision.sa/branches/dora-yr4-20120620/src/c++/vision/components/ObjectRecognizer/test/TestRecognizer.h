#ifndef RECOGNIZER_HPP_
#define RECOGNIZER_HPP_

#include <vector>
#include <string>
#include <time.h>

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>

class CTestRecognizer;
class CTestCase;

class CTestRecognizer: public cast::ManagedComponent, public cast::VideoClient
{
private:
   CTestCase *m_pTestCase;
   std::vector<CTestCase*> m_KnownTests;
   Video::VideoInterfacePrx m_videoServer;
   std::string m_videoServerName;

public:
   int m_camId;
   CTestRecognizer();
   ~CTestRecognizer();
   bool getOneImage(Video::Image &image);

protected:
   virtual void start();
   virtual void runComponent();
   void configure(const std::map<std::string,std::string> & _config);

public:
   using cast::ManagedComponent::sleepComponent;
};

class CTestCase
{
public:
   std::string m_name;
   CTestRecognizer *m_pOwner;
   CTestCase(std::string name, CTestRecognizer *pOwner) {
      m_name = name;
      m_pOwner = pOwner;
   }
   virtual void configure(const std::map<std::string,std::string> & _config) {}
   virtual void onStart() {}
   virtual void onRunComponent() {}
   virtual void runOneStep() { m_pOwner->log("."); }
   virtual void onExitComponent() {}
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
