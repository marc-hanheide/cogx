package BindingData;

/**
* BindingData/FeatureComparisonCompetenceHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class FeatureComparisonCompetenceHolder implements org.omg.CORBA.portable.Streamable
{
  public BindingData.FeatureComparisonCompetence value = null;

  public FeatureComparisonCompetenceHolder ()
  {
  }

  public FeatureComparisonCompetenceHolder (BindingData.FeatureComparisonCompetence initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = BindingData.FeatureComparisonCompetenceHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    BindingData.FeatureComparisonCompetenceHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return BindingData.FeatureComparisonCompetenceHelper.type ();
  }

}
