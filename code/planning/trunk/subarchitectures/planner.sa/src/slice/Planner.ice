#ifndef PLANNER_ICE
#define PLANNER_ICE

#include <cast/slice/CDL.ice>

module autogen {
  module Planner {
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
      string planningAgent;  // the name of the planning agent as in used in the state description
      objDeclSeq objects;
      PlanningState state;
      string goal;
      void loadMAPLTask(string taskFile, string domainFile, string planningAgent);
      void markChanged();
      void activateChangeDetection();	
      bool planAvailable();
      string getPlan();
    };

    // this is for planning-internal use only and takes care of the communication between
    // (the c++ based) cast and the (python) components.
    interface CppServer
    {
      void deliverPlan(int taskID);
    };

    interface PythonServer extends cast::interfaces::CASTComponent
    {
      void registerClient(CppServer* client);
      int addTask(PlanningTask task);
      //PlanningTask newTask();
      //void registerTask(PlanningTask task);
      //void printString(string astring);
    };
  };
};

#endif
