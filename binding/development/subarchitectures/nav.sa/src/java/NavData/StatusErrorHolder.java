package NavData;

/**
* NavData/StatusErrorHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from NavData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/


// finished successfully
public final class StatusErrorHolder implements org.omg.CORBA.portable.Streamable
{
  public NavData.StatusError value = null;

  public StatusErrorHolder ()
  {
  }

  public StatusErrorHolder (NavData.StatusError initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = NavData.StatusErrorHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    NavData.StatusErrorHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return NavData.StatusErrorHelper.type ();
  }

}
