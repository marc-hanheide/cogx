package cast.cdl;

/**
* cast/cdl/TaskManagementDecisionHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from CAST.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class TaskManagementDecisionHolder implements org.omg.CORBA.portable.Streamable
{
  public cast.cdl.TaskManagementDecision value = null;

  public TaskManagementDecisionHolder ()
  {
  }

  public TaskManagementDecisionHolder (cast.cdl.TaskManagementDecision initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = cast.cdl.TaskManagementDecisionHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    cast.cdl.TaskManagementDecisionHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return cast.cdl.TaskManagementDecisionHelper.type ();
  }

}
