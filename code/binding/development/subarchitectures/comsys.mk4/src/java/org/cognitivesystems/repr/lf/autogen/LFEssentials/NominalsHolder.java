package org.cognitivesystems.repr.lf.autogen.LFEssentials;


/**
* org/cognitivesystems/repr/lf/autogen/LFEssentials/NominalsHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from LFEssentials.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/


// end LFNominal
public final class NominalsHolder implements org.omg.CORBA.portable.Streamable
{
  public org.cognitivesystems.repr.lf.autogen.LFEssentials.LFNominal value[] = null;

  public NominalsHolder ()
  {
  }

  public NominalsHolder (org.cognitivesystems.repr.lf.autogen.LFEssentials.LFNominal[] initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = org.cognitivesystems.repr.lf.autogen.LFEssentials.NominalsHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    org.cognitivesystems.repr.lf.autogen.LFEssentials.NominalsHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return org.cognitivesystems.repr.lf.autogen.LFEssentials.NominalsHelper.type ();
  }

}
