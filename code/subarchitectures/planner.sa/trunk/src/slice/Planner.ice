#ifndef PLANNER_ICE
#define PLANNER_ICE

#include <cast/slice/CDL.ice>
#include <beliefs_cast.ice>

module autogen {
  module Planner {

    enum Completion {
      PENDING,     
      // hasn't started yet
      INPROGRESS,  
      // started but no plan found yet
      ABORTED,     
      // aborted (if e.g a more important query occurs)???
      FAILED,      
      // no plan found
      SUCCEEDED    
      // plan found
    };

    struct BeliefEntry {
          cast::cdl::WorkingMemoryAddress address;
          de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief belief;
    };

    sequence<string> stringSeq;
    sequence<de::dfki::lt::tr::beliefs::slice::logicalcontent::dFormula> ArgumentSeq;

    sequence<BeliefEntry> BeliefSeq;

    class StateChangeFilter {
      BeliefSeq removeFilter;
      stringSeq featureFilter;
    };

    class Action {
      int taskID;
      string name;
      ArgumentSeq arguments;
      ArgumentSeq allArguments;
      string fullName;
      float cost;
      Completion status;
    };
    sequence<Action> ActionSeq;

    struct Fact {
        string name;
        ArgumentSeq arguments;
        string modality;
        ArgumentSeq modalArguments;
        de::dfki::lt::tr::beliefs::slice::logicalcontent::dFormula value;
    };

    enum LinkType {
        DEPENDS,
        THREATENS
    };

    struct Link {
        LinkType type;
        int src;
        int dest;
        Fact reason;
    };
    sequence<Link> LinkSeq;

    class POPlan {
        int taskID;
        ActionSeq actions;
        LinkSeq links;
    };

    class Goal {
      float importance;
      int deadline;
      string goalString;
      bool isInPlan;
    };
    sequence<Goal> GoalSeq;

    class PlanningTask
    {
      int id;
      GoalSeq goals;
      bool executePlan;

      ActionSeq plan;
      float costs;
      string firstActionID;
      Completion executionStatus;
      int executionRetries;
      Completion planningStatus;
      int planningRetries;
    };
    
    class Observation{
      string predicate; 
      stringSeq arguments;
    };

    class PlannerVerbalisation {
          string phrase;
    };
    

    sequence<Observation> ObservationSeq;

    class PDDLAction {
      string name;
        stringSeq arguments;
    };

    /* Decision-Theoretic planning.

       A trace of execution is as follows:

       (1) - \class{PythonServer} invokes \method{newTask}. Planning
       proceeds in a new thread.

       (2) - When planning is done, a first action is passed to the
       \class{PythonServer} via its \method{deliverAction}

       (3) - At this point planning is halted until a call to
       \member{deliverObservation} is made, giving the planner the
       observation that resulted from action execution.

       (4) That process continues until the planner calls the
       \class{PythonServer} \member{updateStatus}, informing the
       planning server that POMDP planning is complete, or otherwise
       that it has somehow failed.


     */
    interface DTPServer extends cast::interfaces::CASTComponent {

      /*\class{PythonServer} informs this when an observation is
	received.*/
      void deliverObservation(int id, ObservationSeq observations);

      /*
	
	\argument{id} is the unique identifier of the planning
	process. The idea is to have multile DTP sessions
	concurrently. SHould be able to cope with getting a new id for
	each task. Also, allowed to get the same ID for subsequent
	tasks. Essentially this is derived from the ID of the
	continual planning session.

	The \argument{problemFile} includes everything, including default
	knowledge. File contents should be a DT-PDDL problem definition.
	
	assertions...

	 - Write locks both files until parsed.

	 - Automatically starts planning in a NEW THREAD.
      */
      void newTask(int id, string probleFile, string domainFile);

      /*If the \class{PythonServer} is unhappy with the quality of the
	policy at \argument{id}, it can ask DTP to try and improve the
	quality.*/
      void improvePlanQuality(int id);
      
      /*\class{PythonServer} can abort planning task
	\argument{id}. This will kill the THREAD started by the last
	call to \method{newTask}.*/
      void cancelTask(int id);
      
    };
    
    
    // this is for planning-internal use only and takes care of the communication between
    // (the c++ based) cast and the (python) components.

    interface CppServer
    {
      void deliverPlan(int id, ActionSeq plan, GoalSeq goals);
      void deliverPlanPO(int id, ActionSeq plan, GoalSeq goals, POPlan orderedPlan);
      void updateBeliefState(BeliefSeq beliefs);
      //void deliverPlan(PlanningTask task);
      void updateStatus(int id, Completion status);
      void setChangeFilter(int id, StateChangeFilter filter);
      void waitForChanges(int id, int timeout);
      bool queryGoal(string goal);
      void verbalise(string phrase);
    };

    interface PythonServer extends cast::interfaces::CASTComponent
    {
      void registerTask(PlanningTask task);
      void executeTask(PlanningTask task);
      void updateTask(PlanningTask task);
      void notifyFailure(PlanningTask task);
      void taskTimedOut(PlanningTask task);
      void updateState(BeliefSeq state, BeliefSeq percepts);
      bool queryGoal(BeliefSeq state, string goal);

//       /*DTP process with ID \argument{id} calls this method when: (1)
// 	A useful plan has been found, and execution of that plan
// 	commences, OR (2) during plan execution, an observation has
// 	been received, and the corresponding action has been
// 	determined.*/
//       void deliverAction(int id, PDDLAction action);

      /* (see \method{deliverAction}). \argument{value} is the
       * expected value of executing the current DTP policy
       * ---starting with \argument{action}--- at the current
       * belief-state.*/
      void deliverAction(int id, PDDLAction action, double value);

      /*The DT planner can fail for some reason. In this case it
	informs the \class{PythonServer} by posting a failed status
	update. */
      void updateStatus(int id, Completion status, string message);
    };
  };
};

#endif
