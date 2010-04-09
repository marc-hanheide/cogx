#ifndef PLANNER_ICE
#define PLANNER_ICE

#include <cast/slice/CDL.ice>
#include <beliefmodels.ice>

module autogen {
  module Planner {

    enum Completion {
      PENDING,     // hasn't started yet
      INPROGRESS,  // started but no plan found yet
      ABORTED,     // aborted (if e.g a more important query occurs)???
      FAILED,      // no plan found
      SUCCEEDED    // plan found
    };

    sequence<string> stringSeq;

    sequence<beliefmodels::autogen::beliefs::Belief> BeliefSeq;

    class StateChangeFilter {
      BeliefSeq removeFilter;
      stringSeq featureFilter;
    };

    class Action {
      int taskID;
      string name;
      stringSeq arguments;
      string fullName;
      Completion status;
    };

    sequence<Action> ActionSeq;

    class PlanningTask
    {
      int id;
      string goal;
      ActionSeq plan;
      string firstActionID;
      BeliefSeq state;
      Completion executionStatus;
      int executionRetries;
      Completion planningStatus;
      int planningRetries;
    };

    // this is for planning-internal use only and takes care of the communication between
    // (the c++ based) cast and the (python) components.

    interface CppServer
    {
      void deliverPlan(int id, ActionSeq plan);
      //void deliverPlan(PlanningTask task);
      void updateStatus(int id, Completion status);
      void setChangeFilter(int id, StateChangeFilter filter);
    };

    interface PythonServer extends cast::interfaces::CASTComponent
    {
      void registerTask(PlanningTask task);
      void updateTask(PlanningTask task);
    };
  };
};

#endif
