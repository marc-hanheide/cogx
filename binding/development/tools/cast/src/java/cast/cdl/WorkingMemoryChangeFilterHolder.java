package cast.cdl;

/**
* cast/cdl/WorkingMemoryChangeFilterHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from CAST.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class WorkingMemoryChangeFilterHolder implements org.omg.CORBA.portable.Streamable
{
  public cast.cdl.WorkingMemoryChangeFilter value = null;

  public WorkingMemoryChangeFilterHolder ()
  {
  }

  public WorkingMemoryChangeFilterHolder (cast.cdl.WorkingMemoryChangeFilter initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = cast.cdl.WorkingMemoryChangeFilterHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    cast.cdl.WorkingMemoryChangeFilterHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return cast.cdl.WorkingMemoryChangeFilterHelper.type ();
  }

}
