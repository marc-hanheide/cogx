package BindingFeatures;

/**
* BindingFeatures/ConceptHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingFeatures.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class ConceptHolder implements org.omg.CORBA.portable.Streamable
{
  public BindingFeatures.Concept value = null;

  public ConceptHolder ()
  {
  }

  public ConceptHolder (BindingFeatures.Concept initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = BindingFeatures.ConceptHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    BindingFeatures.ConceptHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return BindingFeatures.ConceptHelper.type ();
  }

}
