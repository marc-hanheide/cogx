package org.cognitivesystems.common.autogen.VisualAttributes;

/**
* org/cognitivesystems/common/autogen/VisualAttributes/ColourHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from VisualAttributes.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/


/**
   * Colour enumeration.
   */
public final class ColourHolder implements org.omg.CORBA.portable.Streamable
{
  public org.cognitivesystems.common.autogen.VisualAttributes.Colour value = null;

  public ColourHolder ()
  {
  }

  public ColourHolder (org.cognitivesystems.common.autogen.VisualAttributes.Colour initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = org.cognitivesystems.common.autogen.VisualAttributes.ColourHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    org.cognitivesystems.common.autogen.VisualAttributes.ColourHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return org.cognitivesystems.common.autogen.VisualAttributes.ColourHelper.type ();
  }

}
