package BindingData;

/**
* BindingData/FeatureComparatorQueryHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class FeatureComparatorQueryHolder implements org.omg.CORBA.portable.Streamable
{
  public BindingData.FeatureComparatorQuery value = null;

  public FeatureComparatorQueryHolder ()
  {
  }

  public FeatureComparatorQueryHolder (BindingData.FeatureComparatorQuery initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = BindingData.FeatureComparatorQueryHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    BindingData.FeatureComparatorQueryHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return BindingData.FeatureComparatorQueryHelper.type ();
  }

}
