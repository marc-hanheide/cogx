package planning.autogen;

/**
* planning/autogen/PlanHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from PlanningData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class PlanHolder implements org.omg.CORBA.portable.Streamable
{
  public planning.autogen.Plan value = null;

  public PlanHolder ()
  {
  }

  public PlanHolder (planning.autogen.Plan initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = planning.autogen.PlanHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    planning.autogen.PlanHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return planning.autogen.PlanHelper.type ();
  }

}
