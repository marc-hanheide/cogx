// vim:set fileencoding=utf-8 sw=3 ts=8 et:vim
/*
 * @author:  Marko Mahnič
 * @created: jun 2009 
 */
   
// cast
#include "TestRecognizer.h"

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <VideoUtils.h>
#include <Video.hpp>

#include "../../VisionUtils.h"
#include "TestCases.h"

using namespace std;
using namespace cast;
using namespace VisionData;

extern "C" {
   cast::CASTComponentPtr newComponent()
   {
      return new CTestRecognizer();
   }
}

CTestRecognizer::CTestRecognizer()
{
   m_pTestCase = NULL;
}

CTestRecognizer::~CTestRecognizer()
{
   if (m_pTestCase != NULL) {
      delete m_pTestCase;
      m_pTestCase = NULL;
   }
}

void CTestRecognizer::start()
{
   log("Recognizer TEST starting");
}

void CTestRecognizer::configure(const std::map<std::string,std::string> & _config)
{
   map<string,string>::const_iterator it;

   if (m_pTestCase) {
      log("configure: Deleting an existing Test Case.");
      m_pTestCase->onExitComponent();
      delete m_pTestCase;
      m_pTestCase = NULL;
   }

   if((it = _config.find("--testmode")) != _config.end())
   {
      string mode;
      istringstream istr(it->second);
      istr >> mode;
      if (mode == "server") m_pTestCase = new CTestCase_Server(mode, this);
      else if (mode == "wmcall") m_pTestCase = new CTestCase_WmResponder(mode, this);
      log("TEST MODE: %s", mode.c_str());
   }

   if (m_pTestCase != NULL) {
      m_pTestCase->onStart();
   }
}

void CTestRecognizer::runComponent()
{
   sleepComponent(2000);

   if (m_pTestCase != NULL) {
      while(isRunning()) {
         m_pTestCase->runOneStep();
         sleepComponent(10);
      }
   }
   if (m_pTestCase) {
      log("runComponent: Test Case Cleanup.");
      m_pTestCase->onExitComponent();
      delete m_pTestCase;
      m_pTestCase = NULL;
   }
   log("runComponent Done.");   
}

