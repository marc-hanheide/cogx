package org.cognitivesystems.comsys.autogen.ComsysEssentials;

/**
* org/cognitivesystems/comsys/autogen/ComsysEssentials/LabelFormulaPairHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from ComsysEssentials.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class LabelFormulaPairHolder implements org.omg.CORBA.portable.Streamable
{
  public org.cognitivesystems.comsys.autogen.ComsysEssentials.LabelFormulaPair value = null;

  public LabelFormulaPairHolder ()
  {
  }

  public LabelFormulaPairHolder (org.cognitivesystems.comsys.autogen.ComsysEssentials.LabelFormulaPair initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = org.cognitivesystems.comsys.autogen.ComsysEssentials.LabelFormulaPairHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    org.cognitivesystems.comsys.autogen.ComsysEssentials.LabelFormulaPairHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return org.cognitivesystems.comsys.autogen.ComsysEssentials.LabelFormulaPairHelper.type ();
  }

}
