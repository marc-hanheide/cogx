package Planner;


/**
* Planner/PlanningTask.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from Planner.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class PlanningTask implements org.omg.CORBA.portable.IDLEntity
{
  public String task_id = null;
  public String planning_agent = null;

  // the name of the planning agent (as in objects)
  public Planner.ObjectDeclaration objects[] = null;
  public Planner.Fact facts[] = null;
  public String goals[] = null;

  // conjunction of goals
  public String domain_name = null;
  public String domain_fn = null;

  public PlanningTask ()
  {
  } // ctor

  public PlanningTask (String _task_id, String _planning_agent, Planner.ObjectDeclaration[] _objects, Planner.Fact[] _facts, String[] _goals, String _domain_name, String _domain_fn)
  {
    task_id = _task_id;
    planning_agent = _planning_agent;
    objects = _objects;
    facts = _facts;
    goals = _goals;
    domain_name = _domain_name;
    domain_fn = _domain_fn;
  } // ctor

} // class PlanningTask
