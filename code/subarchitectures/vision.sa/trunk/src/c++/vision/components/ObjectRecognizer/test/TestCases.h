// vim:set fileencoding=utf-8 sw=3 ts=8 et:vim
/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */

#ifndef __TEST_DRIVERS_HPP__
#define __TEST_DRIVERS_HPP__

#include "TestRecognizer.h"
#include "../RecognizerClient.h"

#include <string>
#include <map>

/*
 * Test the connection between ObjectRecognizer Client and Server.
 *
 * This is also an example of the usage of CObjectRecognizerClient in
 * a CASTComponent:
 *    - add a member variable for the client (m_OrClient); deriving the
 *      CASTComponent from CObjectRecognizerClient might work, but it is
 *      a bad design choice;
 *    - call configureRecognizer() from CASTComponent::configure(); see configure()
 *    - call connectIceClient() from CASTComponent::start(); see onStart().
 *   
 * Configure the client and the server in a cast configuration file:
 * 
 *    SUBARCHITECTURE vision.sa
 *    CPP MG vis.recognizer.srv ObjectRecognizerSrv --matcher-cuda --log true
 *        --models "SwGreenTea ShelcoreCube TwLemonTea"
 *        --modeldir "subarchitectures/vision.sa/config/test-objrecognizer/model"
 *    CPP MG vis.recognizer.test TestRecognizer
 *        --testmode server
 *        --recognizerhost localhost --recognizerid vis.recognizer.srv
 *
 *    (each component should be defined on one line) 
 */
class CTestCase_Server: public CTestCase
{
   cogx::vision::CObjectRecognizerClient m_OrClient;
public:
   // CTestRecognizer is a CASTComponent
   CTestCase_Server(std::string name, CTestRecognizer *pOwner);

   // called from CTestRecognizer::configure()
   void configure(const std::map<std::string,std::string> & _config);

   // called from CTestRecognizer::start()
   void onStart();

   // called repeatedly from CTestRecognizer::runComponent()
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
