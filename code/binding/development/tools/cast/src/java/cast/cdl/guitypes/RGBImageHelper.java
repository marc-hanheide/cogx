package cast.cdl.guitypes;


/**
* cast/cdl/guitypes/RGBImageHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from guitypes.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class RGBImageHelper
{
  private static String  _id = "IDL:cast/cdl/guitypes/RGBImage:1.0";

  public static void insert (org.omg.CORBA.Any a, cast.cdl.guitypes.RGBImage that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static cast.cdl.guitypes.RGBImage extract (org.omg.CORBA.Any a)
  {
    return read (a.create_input_stream ());
  }

  private static org.omg.CORBA.TypeCode __typeCode = null;
  private static boolean __active = false;
  synchronized public static org.omg.CORBA.TypeCode type ()
  {
    if (__typeCode == null)
    {
      synchronized (org.omg.CORBA.TypeCode.class)
      {
        if (__typeCode == null)
        {
          if (__active)
          {
            return org.omg.CORBA.ORB.init().create_recursive_tc ( _id );
          }
          __active = true;
          org.omg.CORBA.StructMember[] _members0 = new org.omg.CORBA.StructMember [4];
          org.omg.CORBA.TypeCode _tcOf_members0 = null;
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[0] = new org.omg.CORBA.StructMember (
            "m_width",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[1] = new org.omg.CORBA.StructMember (
            "m_height",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_octet);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[2] = new org.omg.CORBA.StructMember (
            "m_rgbBuffer",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[3] = new org.omg.CORBA.StructMember (
            "m_flags",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (cast.cdl.guitypes.RGBImageHelper.id (), "RGBImage", _members0);
          __active = false;
        }
      }
    }
    return __typeCode;
  }

  public static String id ()
  {
    return _id;
  }

  public static cast.cdl.guitypes.RGBImage read (org.omg.CORBA.portable.InputStream istream)
  {
    cast.cdl.guitypes.RGBImage value = new cast.cdl.guitypes.RGBImage ();
    value.m_width = istream.read_long ();
    value.m_height = istream.read_long ();
    int _len0 = istream.read_long ();
    value.m_rgbBuffer = new byte[_len0];
    istream.read_octet_array (value.m_rgbBuffer, 0, _len0);
    value.m_flags = istream.read_long ();
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, cast.cdl.guitypes.RGBImage value)
  {
    ostream.write_long (value.m_width);
    ostream.write_long (value.m_height);
    ostream.write_long (value.m_rgbBuffer.length);
    ostream.write_octet_array (value.m_rgbBuffer, 0, value.m_rgbBuffer.length);
    ostream.write_long (value.m_flags);
  }

}
