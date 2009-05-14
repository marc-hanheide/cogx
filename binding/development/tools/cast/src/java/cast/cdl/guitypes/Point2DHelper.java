package cast.cdl.guitypes;


/**
* cast/cdl/guitypes/Point2DHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from guitypes.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class Point2DHelper
{
  private static String  _id = "IDL:cast/cdl/guitypes/Point2D:1.0";

  public static void insert (org.omg.CORBA.Any a, cast.cdl.guitypes.Point2D that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static cast.cdl.guitypes.Point2D extract (org.omg.CORBA.Any a)
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
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_double);
          _members0[0] = new org.omg.CORBA.StructMember (
            "m_x",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_double);
          _members0[1] = new org.omg.CORBA.StructMember (
            "m_y",
            _tcOf_members0,
            null);
          _tcOf_members0 = cast.cdl.guitypes.RGBColorHelper.type ();
          _members0[2] = new org.omg.CORBA.StructMember (
            "m_color",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[3] = new org.omg.CORBA.StructMember (
            "m_flags",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (cast.cdl.guitypes.Point2DHelper.id (), "Point2D", _members0);
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

  public static cast.cdl.guitypes.Point2D read (org.omg.CORBA.portable.InputStream istream)
  {
    cast.cdl.guitypes.Point2D value = new cast.cdl.guitypes.Point2D ();
    value.m_x = istream.read_double ();
    value.m_y = istream.read_double ();
    value.m_color = cast.cdl.guitypes.RGBColorHelper.read (istream);
    value.m_flags = istream.read_long ();
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, cast.cdl.guitypes.Point2D value)
  {
    ostream.write_double (value.m_x);
    ostream.write_double (value.m_y);
    cast.cdl.guitypes.RGBColorHelper.write (ostream, value.m_color);
    ostream.write_long (value.m_flags);
  }

}
