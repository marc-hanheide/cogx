#ifndef DTP_CONTROL
#define DTP_CONTROL


#include <tr1/memory>
//#include <cast/architecture.hpp>
#include <Ice/Ice.h>

#include <cast/core/CASTComponent.hpp>

#include "../Planner.hpp"


using namespace autogen::Planner;


class DTPCONTROL : 
  public autogen::Planner::DTPServer,
  public cast::CASTComponent {
public:
  void deliverObservation(const Ice::Current&, int id, ObservationSeq observationSeq);
  void newTask(const Ice::Current&, int id, string probleFile, string domainFile);  
  void cancelTask(const Ice::Current&, int id);

protected:
  void configure(const cast::cdl::StringMap& _config, const Ice::Current& _current){}
   void start(){}
   void runComponent(){}
   void stop(){};
  

//   /*slice (see planner.sa/src/slice/###)*/
//   class Internal_DTP_Server : public DTPServer  
//   {
//   public:
//     Internal_DTP_Server(DTP_CONTROL*);
//     void deliverObservation();
//     void deliverProblem();
//     void deliverDomain();
//   private:
//     DTP_CONTROL* parent;
//   };

//   Internal_DTP_Server internal_DTP_Server;
};


#endif
