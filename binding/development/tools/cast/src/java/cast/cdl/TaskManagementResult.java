package cast.cdl;


/**
* cast/cdl/TaskManagementResult.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from CAST.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class TaskManagementResult implements org.omg.CORBA.portable.IDLEntity
{
  public String m_id = null;
  public cast.cdl.TaskManagementDecision m_decision = null;

  public TaskManagementResult ()
  {
  } // ctor

  public TaskManagementResult (String _m_id, cast.cdl.TaskManagementDecision _m_decision)
  {
    m_id = _m_id;
    m_decision = _m_decision;
  } // ctor

} // class TaskManagementResult
