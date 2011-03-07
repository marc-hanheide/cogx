#ifndef GY2ARTICLE_BVLYY5Z8
#define GY2ARTICLE_BVLYY5Z8

#include "TestLearner.h"
#include "ptrvector.hpp"

#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <string>
#include <sstream>

struct CTestEntry
{
   std::string name;
   std::string videoLeft;
   std::string videoRight;
   std::string shape;
   std::string color;

   CTestEntry()
   {
      name = "";
      videoLeft = "";
      videoRight = "";
      color = "";
      shape = "";
   }

   bool isValid()
   {
      if (videoLeft == "" || videoRight == "") return false;
      if (color == "" && shape == "") return false;
      return true;
   }
};

// Test for Y2 article about George.
// We are testing how the system performs while it is learning inside CAST.
// The system starts with an empty knowledge. Then we show the system a series
// of objects and tell it about these objects.
class CGeorgeY2Article: public CTestCase
{
   void onAdd_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc);
   void onDel_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc);
   void onAdd_SpokenItem(const cast::cdl::WorkingMemoryChange & _wmc);

   int m_ObjectCount;
   int m_TeachingStep;
   std::string m_RobotResponse;
   CPtrVector<CTestEntry> m_testEntries;
   int m_currentTest;
   std::string m_imageDir;
   std::map<std::string, std::string> options;
   double m_timeout; // absolute time of next timeout (in seconds)

   enum _state_ {
      stStart, stTableEmpty, stWaitToAppear, stObjectOn, stTeaching, stWaitForResponse,
      stEndOfTeaching, stWaitToDisappear, stFinished, stTimedOut
   };
   std::map<int, std::string> m_stateNames;
   int m_State;
   IceUtil::Monitor<IceUtil::Mutex> m_EventMonitor; // WM Events can trigger state changes
   void switchState(int newState);
   void verifyCount(int count);
   CTestEntry* getCurrentTest();
   bool nextTest();
   void loadScene();
   void loadEmptyScene();
   bool isTimedOut();
   bool performNextTeachingStep();
   void report(std::ostringstream& what);
   void report(const std::string& what);

public:
   CGeorgeY2Article(std::string name, CTestRecognizer *pOwner);
   ~CGeorgeY2Article();
   void configure(const std::map<std::string,std::string> & _config);
   void onStart();
   void runOneStep();
};

#endif /* end of include guard: GY2ARTICLE_BVLYY5Z8 */
