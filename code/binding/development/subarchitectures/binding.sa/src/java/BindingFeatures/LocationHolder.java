package BindingFeatures;

/**
* BindingFeatures/LocationHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingFeatures.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class LocationHolder implements org.omg.CORBA.portable.Streamable
{
  public BindingFeatures.Location value = null;

  public LocationHolder ()
  {
  }

  public LocationHolder (BindingFeatures.Location initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = BindingFeatures.LocationHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    BindingFeatures.LocationHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return BindingFeatures.LocationHelper.type ();
  }

}
