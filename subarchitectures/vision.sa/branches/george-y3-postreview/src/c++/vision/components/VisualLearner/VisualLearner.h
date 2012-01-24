#ifndef __CAST_VISUAL_LEARNER_H__
#define __CAST_VISUAL_LEARNER_H__

#include <VisionData.hpp>
#include <MatlabData.hpp>
#include <CDisplayClient.hpp>

#include <cast/architecture/ManagedComponent.hpp>

#include <vector>
#include <map>

namespace cogx
{

typedef std::vector<cast::cdl::WorkingMemoryAddress> TWmAddressVector;
class VisualLearner : public cast::ManagedComponent
{
public:
   VisualLearner();
   virtual ~VisualLearner();

   virtual void runComponent();
   virtual void configure(const std::map<std::string,std::string> & _config);
   virtual void start();

#ifdef FEAT_VISUALIZATION
private:
   class CMyDisplayClient: public cogx::display::CDisplayClient
   {
      VisualLearner* pComponent;
   public:
      CMyDisplayClient() { pComponent = NULL; }
      void setClientData(VisualLearner* pVisualLearnter) { pComponent = pVisualLearnter; }
      void handleEvent(const Visualization::TEvent &event); /*override*/
      //std::string getControlState(const std::string& ctrlId); [>override<]
      //void handleForm(const std::string& id, const std::string& partId,
      //      const std::map<std::string, std::string>& fields);
      //bool getFormData(const std::string& id, const std::string& partId,
      //      std::map<std::string, std::string>& fields);

      void createViews();
   };
   CMyDisplayClient m_display;
#endif

private:
   // Two request queues with a single monitor
   TWmAddressVector m_RecogTaskId_Queue;
   TWmAddressVector m_LearnTaskId_Queue;
   TWmAddressVector m_AffordanceTaskId_Queue; // Affordance Recognition
   IceUtil::Monitor<IceUtil::Mutex> m_RrqMonitor;
   cast::cdl::WorkingMemoryAddress m_addrColorStatus;
   cast::cdl::WorkingMemoryAddress m_addrShapeStatus;

   // CLFStart configuration file (evaluated by MCR)
   std::string m_ClfConfigFile;

   void onAdd_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
   void onAdd_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc);
   void onAdd_AffordanceTask(const cast::cdl::WorkingMemoryChange & _wmc);
   bool recogniseAttributes(VisionData::VisualLearnerRecognitionTaskPtr _pTask);
   bool recogniseAffordance(VisionData::AffordanceRecognitionTaskPtr _pTask);
   bool updateModel(VisionData::VisualLearningTaskPtr _pTask);
   void updateWmModelStatus();

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

