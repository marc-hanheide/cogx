package NavData;

/**
* NavData/LineMapHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from NavData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class LineMapHolder implements org.omg.CORBA.portable.Streamable
{
  public NavData.LineMap value = null;

  public LineMapHolder ()
  {
  }

  public LineMapHolder (NavData.LineMap initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = NavData.LineMapHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    NavData.LineMapHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return NavData.LineMapHelper.type ();
  }

}
