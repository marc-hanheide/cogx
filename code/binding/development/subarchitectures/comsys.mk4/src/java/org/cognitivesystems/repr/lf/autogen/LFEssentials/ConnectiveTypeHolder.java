package org.cognitivesystems.repr.lf.autogen.LFEssentials;

/**
* org/cognitivesystems/repr/lf/autogen/LFEssentials/ConnectiveTypeHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from LFEssentials.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/


// CONJUNCTIVE ("AND").
public final class ConnectiveTypeHolder implements org.omg.CORBA.portable.Streamable
{
  public org.cognitivesystems.repr.lf.autogen.LFEssentials.ConnectiveType value = null;

  public ConnectiveTypeHolder ()
  {
  }

  public ConnectiveTypeHolder (org.cognitivesystems.repr.lf.autogen.LFEssentials.ConnectiveType initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = org.cognitivesystems.repr.lf.autogen.LFEssentials.ConnectiveTypeHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    org.cognitivesystems.repr.lf.autogen.LFEssentials.ConnectiveTypeHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return org.cognitivesystems.repr.lf.autogen.LFEssentials.ConnectiveTypeHelper.type ();
  }

}
