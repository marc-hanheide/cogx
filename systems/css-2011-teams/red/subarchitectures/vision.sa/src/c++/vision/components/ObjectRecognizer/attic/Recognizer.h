#ifndef RECOGNIZER_HPP_
#define RECOGNIZER_HPP_

#include <vector>
#include <string>
#include <time.h>
#include <Python.h>

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>

class CRecognizer: public cast::ManagedComponent, public cast::VideoClient
{
private:
  std::string videoServerName;
  std::string videoHostName;
  /**
   * our ICE proxy to the video server
   */
  Video::VideoInterfacePrx videoServer;
  /**
   * wether we are currently receiving images from the server
   */
   int testmode;

public:
   CRecognizer();

protected:
   virtual void start();
   virtual void runComponent();
   void configure(const std::map<std::string,std::string> & _config);

   /* Capture addition/change of a Recognition Task
    */
   void onAddRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
   //void onDeleteRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
   //void onChangeRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);

   // Workers
   void doRecognize(const cast::cdl::WorkingMemoryChange & _wmc);
   void abortRecognition(const cast::cdl::WorkingMemoryChange & _wmc, VisionData::ObjectRecognitionTaskPtr cmd);

   // Some helpers for testing
   // void _test_addRecognitionTask();
private:
   PyObject* pyProcessImage(Video::Image &image, const int *region);
   void pyParseMatches(PyObject *pMatches, VisionData::ObjectRecognitionMatchPtr &imatch);
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
