package org.cognitivesystems.repr.lf.autogen.LFPacking;

/**
* org/cognitivesystems/repr/lf/autogen/LFPacking/PackedOntologicalSortHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from LFPacking.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class PackedOntologicalSortHolder implements org.omg.CORBA.portable.Streamable
{
  public org.cognitivesystems.repr.lf.autogen.LFPacking.PackedOntologicalSort value = null;

  public PackedOntologicalSortHolder ()
  {
  }

  public PackedOntologicalSortHolder (org.cognitivesystems.repr.lf.autogen.LFPacking.PackedOntologicalSort initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = org.cognitivesystems.repr.lf.autogen.LFPacking.PackedOntologicalSortHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    org.cognitivesystems.repr.lf.autogen.LFPacking.PackedOntologicalSortHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return org.cognitivesystems.repr.lf.autogen.LFPacking.PackedOntologicalSortHelper.type ();
  }

}
