package cast.cdl.guitypes;

/**
* cast/cdl/guitypes/Line3DHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from guitypes.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class Line3DHolder implements org.omg.CORBA.portable.Streamable
{
  public cast.cdl.guitypes.Line3D value = null;

  public Line3DHolder ()
  {
  }

  public Line3DHolder (cast.cdl.guitypes.Line3D initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = cast.cdl.guitypes.Line3DHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    cast.cdl.guitypes.Line3DHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return cast.cdl.guitypes.Line3DHelper.type ();
  }

}
