package cast.cdl.ui;

/**
* cast/cdl/ui/ComponentStatusHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from CAST.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class ComponentStatusHolder implements org.omg.CORBA.portable.Streamable
{
  public cast.cdl.ui.ComponentStatus value = null;

  public ComponentStatusHolder ()
  {
  }

  public ComponentStatusHolder (cast.cdl.ui.ComponentStatus initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = cast.cdl.ui.ComponentStatusHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    cast.cdl.ui.ComponentStatusHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return cast.cdl.ui.ComponentStatusHelper.type ();
  }

}
