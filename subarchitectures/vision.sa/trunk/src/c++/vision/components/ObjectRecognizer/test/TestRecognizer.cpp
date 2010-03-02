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
   m_KnownTests.push_back(new CTestCase_Server("server", this));
   m_KnownTests.push_back(new CTestCase_WmResponder("wmcall", this));
}

CTestRecognizer::~CTestRecognizer()
{
   if (m_pTestCase != NULL) {
     m_pTestCase = NULL;
   }
   vector<CTestCase*>::iterator itest;
   for (itest = m_KnownTests.begin(); itest != m_KnownTests.end(); itest++) {
      CTestCase* ptest = *itest;
      *itest = NULL;
      delete ptest;
   }
   m_KnownTests.clear();
}

void CTestRecognizer::start()
{
   log("TEST starting");
   if (m_pTestCase != NULL) {
      m_pTestCase->onStart();
   }
}

void CTestRecognizer::configure(const std::map<std::string,std::string> & _config)
{
   log("TEST configuring");
   map<string,string>::const_iterator it;
   vector<CTestCase*>::iterator itest;
   if (1) {
      ostringstream msg;
      for (itest = m_KnownTests.begin(); itest != m_KnownTests.end(); itest++) {
         msg << " '" << (*itest)->m_name << "' ";
      }
      println("Known tests: %s", msg.str().c_str());
   }

   if (m_pTestCase) {
      debug("configure: Terminating an existing Test Case.");
      m_pTestCase->onExitComponent();
      m_pTestCase = NULL;
   }

   string mode;
   if((it = _config.find("--testmode")) != _config.end())
   {
      istringstream istr(it->second);
      istr >> mode;
      for (itest = m_KnownTests.begin(); itest != m_KnownTests.end(); itest++) {
         if (mode == (*itest)->m_name) m_pTestCase = *itest;
         break;
      }
   }

   if (m_pTestCase != NULL) {
      m_pTestCase->configure(_config);
      log("TEST MODE: '%s'", m_pTestCase->m_name.c_str());
   }
   else {
      log("WARNING: UNKNOWN TEST MODE '%s'", mode);
   }
}

void CTestRecognizer::runComponent()
{
   sleepComponent(2000);
   log("TEST running.");

   if (m_pTestCase != NULL) {
      m_pTestCase->onRunComponent();

      while(isRunning()) {
         // debug("runOneStep %s", m_pTestCase->m_name.c_str());
         m_pTestCase->runOneStep();
         sleepComponent(10);
      }

      log("runComponent: Test Case Cleanup.");
      m_pTestCase->onExitComponent();
      m_pTestCase = NULL;
   }
   else {
      log("No test case.");
   }
 
   log("TEST runComponent Done.");   
}

