package org.cognitivesystems.common.autogen.VisualAttributes;

/**
* org/cognitivesystems/common/autogen/VisualAttributes/SizeHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from VisualAttributes.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/


/**
   * Size enumeration
   */
public final class SizeHolder implements org.omg.CORBA.portable.Streamable
{
  public org.cognitivesystems.common.autogen.VisualAttributes.Size value = null;

  public SizeHolder ()
  {
  }

  public SizeHolder (org.cognitivesystems.common.autogen.VisualAttributes.Size initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = org.cognitivesystems.common.autogen.VisualAttributes.SizeHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    org.cognitivesystems.common.autogen.VisualAttributes.SizeHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return org.cognitivesystems.common.autogen.VisualAttributes.SizeHelper.type ();
  }

}
