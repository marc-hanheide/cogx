package cast.cdl;


/**
* cast/cdl/WorkingMemoryEntryListHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from CAST.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class WorkingMemoryEntryListHolder implements org.omg.CORBA.portable.Streamable
{
  public cast.cdl.WorkingMemoryEntry value[] = null;

  public WorkingMemoryEntryListHolder ()
  {
  }

  public WorkingMemoryEntryListHolder (cast.cdl.WorkingMemoryEntry[] initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = cast.cdl.WorkingMemoryEntryListHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    cast.cdl.WorkingMemoryEntryListHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return cast.cdl.WorkingMemoryEntryListHelper.type ();
  }

}
