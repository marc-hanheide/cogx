#ifndef PLANNER_ICE
#define PLANNER_ICE

#include <cast/slice/CDL.ice>

module autogen {
  module Planner {

    enum Completion {
      PENDING,     // hasn't started yet
      INPROGRESS,  // started but no plan found yet
      ABORTED,     // aborted (if e.g a more important query occurs)???
      FAILED,      // no plan found
      SUCCEEDED  // plan found
    };

    sequence<string> stringSeq;

    class ObjectDeclaration {
      string name;
      string type;
    };

    sequence<ObjectDeclaration> objDeclSeq;

    enum ModalityEnum {FACTMOD, BELIEFMOD, KVALMOD};

    class Fact {
      ModalityEnum modality;        
      stringSeq believers;  
      string name;              /* name of the state variable used */
      stringSeq arguments;  	/* argument list */
      string value;             /* any value from the domain of 'name' + "unknown" */
    };
    sequence<Fact> factSeq;

    class PlanningState {
      factSeq facts;
    };

    class PlanningTask
    {
      int id;
      string domain;
      string task;
      string plan;
      Completion status;
      //string planningAgent;  // the name of the planning agent as used in the state description
      //objDeclSeq objects;
      //PlanningState state;
      //string goal;
    };

    // this is for planning-internal use only and takes care of the communication between
    // (the c++ based) cast and the (python) components.

    interface CppServer
    {
      void deliverPlan(PlanningTask task);
    };

    interface PythonServer extends cast::interfaces::CASTComponent
    {
      void registerClient(CppServer* client);
      void registerTask(PlanningTask task);
    };
  };
};

#endif
