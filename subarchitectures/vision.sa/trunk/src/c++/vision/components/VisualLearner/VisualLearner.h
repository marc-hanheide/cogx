#ifndef __CAST_VISUAL_LEARNER_H__
#define __CAST_VISUAL_LEARNER_H__

#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>

#include <vector>
#include <map>

//default useful namespaces, fix to reflect your own code
using namespace std;
using namespace VisionData; // ROI
using namespace cast;
using namespace boost;
using namespace Matlab; // Matrix

// typedef map<string, shared_ptr<const CASTData<VisionData::SOI> > > SOIMap;
typedef vector<shared_ptr<const CASTData<VisionData::SOI> > > SOIVector;

namespace cast
{

class VisualLearner : public ManagedComponent {
   public:
      VisualLearner();
      virtual ~VisualLearner();

      virtual void runComponent();
      virtual void configure(map<string,string> & _config);
      virtual void start();
      virtual void stop();

   protected:
      virtual void taskAdopted(const string &_taskID);
      virtual void taskRejected(const string &_taskID);

      void recogniseAttributes(ProtoObjectPtr _pData); //shared_ptr<const CASTData<SOI> > _pData);

   private:

      void onNewProtoObject(const cdl::WorkingMemoryChange & _wmc);
//      string allowedSoiSource;
//      void quip(CASTData<ComedyEssentials::Joke> *_pData);
//      string generatePunchline(const string &_setup);

      // Hashtable used to record the tasks we want to carry out
//      SOIMap * m_pProposedProcessing;
      SOIVector * m_pSOIs;
//      float m_PerspectiveTrafo[9];
//      bool m_bTrafoEnabled;
}; // class VisualLearner

}

#endif //  __CAST_VISUAL_LEARNER_H__

