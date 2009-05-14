package org.cognitivesystems.common.autogen.VisualAttributes;


/**
* org/cognitivesystems/common/autogen/VisualAttributes/SizeHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from VisualAttributes.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/


/**
   * Size enumeration
   */
abstract public class SizeHelper
{
  private static String  _id = "IDL:VisualAttributes/Size:1.0";

  public static void insert (org.omg.CORBA.Any a, org.cognitivesystems.common.autogen.VisualAttributes.Size that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static org.cognitivesystems.common.autogen.VisualAttributes.Size extract (org.omg.CORBA.Any a)
  {
    return read (a.create_input_stream ());
  }

  private static org.omg.CORBA.TypeCode __typeCode = null;
  synchronized public static org.omg.CORBA.TypeCode type ()
  {
    if (__typeCode == null)
    {
      __typeCode = org.omg.CORBA.ORB.init ().create_enum_tc (org.cognitivesystems.common.autogen.VisualAttributes.SizeHelper.id (), "Size", new String[] { "TINY", "SMALL", "BIG", "LARGE", "HUGE", "UNKNOW_SIZE"} );
    }
    return __typeCode;
  }

  public static String id ()
  {
    return _id;
  }

  public static org.cognitivesystems.common.autogen.VisualAttributes.Size read (org.omg.CORBA.portable.InputStream istream)
  {
    return org.cognitivesystems.common.autogen.VisualAttributes.Size.from_int (istream.read_long ());
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, org.cognitivesystems.common.autogen.VisualAttributes.Size value)
  {
    ostream.write_long (value.value ());
  }

}
