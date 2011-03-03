#ifndef GY2ARTICLE_BVLYY5Z8
#define GY2ARTICLE_BVLYY5Z8

#include "TestLearner.h"

#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <string>

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

   enum _state_ {
      stTableEmpty, stWaitToAppear, stObjectOn, stTeaching, stWaitForResponse, stEndOfTeaching, stWaitToDisappear
   };
   int m_State;
   IceUtil::Monitor<IceUtil::Mutex> m_EventMonitor; // WM Events can trigger state changes
   void switchState(int newState);
   void verifyCount(int count);
   void loadNextObject();
   void loadEmptyScene();
   bool performNextTeachingStep();

public:
   CGeorgeY2Article(std::string name, CTestRecognizer *pOwner);
   void onStart();
   void runOneStep();
};

#endif /* end of include guard: GY2ARTICLE_BVLYY5Z8 */
