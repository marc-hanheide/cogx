#ifndef DTP_CONTROL
#define DTP_CONTROL


#include <tr1/memory>
//#include <cast/architecture.hpp>
#include <Ice/Ice.h>

#include <cast/core/CASTComponent.hpp>

#include "../Planner.hpp"

#include <map>


using namespace autogen::Planner;


class DTPCONTROL : 
  public autogen::Planner::DTPServer,
  public cast::CASTComponent 
{
public:
  DTPCONTROL(){};
  ~DTPCONTROL(){};

  //~DTPCONTROL();
  void deliverObservation(Ice::Int, const autogen::Planner::ObservationSeq& observationSeq, const Ice::Current&);
  void newTask(Ice::Int, const std::string& probleFile, const std::string& domainFile, const Ice::Current&);  
  void cancelTask(Ice::Int, const Ice::Current&);

protected:
  void configure(const cast::cdl::StringMap& _config, const Ice::Current& _current){}
   void start(){}
   void runComponent(){}
   void stop(){};
  
// private:
//     std::map<int, > ;
};


#endif
