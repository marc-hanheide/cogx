package org.cognitivesystems.comsys.autogen.ComsysEssentials;

/**
* org/cognitivesystems/comsys/autogen/ComsysEssentials/InterpretationSupportHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from ComsysEssentials.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class InterpretationSupportHolder implements org.omg.CORBA.portable.Streamable
{
  public org.cognitivesystems.comsys.autogen.ComsysEssentials.InterpretationSupport value = null;

  public InterpretationSupportHolder ()
  {
  }

  public InterpretationSupportHolder (org.cognitivesystems.comsys.autogen.ComsysEssentials.InterpretationSupport initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = org.cognitivesystems.comsys.autogen.ComsysEssentials.InterpretationSupportHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    org.cognitivesystems.comsys.autogen.ComsysEssentials.InterpretationSupportHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return org.cognitivesystems.comsys.autogen.ComsysEssentials.InterpretationSupportHelper.type ();
  }

}
