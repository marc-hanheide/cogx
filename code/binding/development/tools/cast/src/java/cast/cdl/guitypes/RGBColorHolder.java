package cast.cdl.guitypes;

/**
* cast/cdl/guitypes/RGBColorHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from guitypes.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class RGBColorHolder implements org.omg.CORBA.portable.Streamable
{
  public cast.cdl.guitypes.RGBColor value = null;

  public RGBColorHolder ()
  {
  }

  public RGBColorHolder (cast.cdl.guitypes.RGBColor initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = cast.cdl.guitypes.RGBColorHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    cast.cdl.guitypes.RGBColorHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return cast.cdl.guitypes.RGBColorHelper.type ();
  }

}
