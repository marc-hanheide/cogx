#ifndef __CAST_VISUAL_LEARNER_H__
#define __CAST_VISUAL_LEARNER_H__

#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include <MatlabData.hpp>

#include <vector>
#include <map>

namespace cogx
{
//default useful namespaces, fix to reflect your own code
using namespace std;
using namespace VisionData;
using namespace cast;
using namespace boost;
using namespace Matlab; // Matrix

// typedef map<string, shared_ptr<const CASTData<VisionData::SOI> > > SOIMap;
// typedef vector<shared_ptr<const cast::CASTData<VisionData::SOI> > > SOIVector;
typedef vector<cdl::WorkingMemoryAddress> WmAddressVector;
class VisualLearner : public ManagedComponent {
   public:
      VisualLearner();
      virtual ~VisualLearner();

      virtual void runComponent();
      virtual void configure(map<string,string> & _config);
      virtual void start();
      virtual void stop();

   private:
      WmAddressVector m_RequestIdQueue;
      void onNewRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
      void recogniseAttributes(VisionData::VisualLearnerRecognitionTaskPtr _pTask);

      //string allowedSoiSource;
      //void quip(CASTData<ComedyEssentials::Joke> *_pData);
      //string generatePunchline(const string &_setup);

      // Hashtable used to record the tasks we want to carry out
      //SOIMap * m_pProposedProcessing;
      //SOIVector * m_pSOIs;
      //float m_PerspectiveTrafo[9];
      //bool m_bTrafoEnabled;
}; // class VisualLearner
}

#endif //  __CAST_VISUAL_LEARNER_H__

