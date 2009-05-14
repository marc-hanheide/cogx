package cast.cdl.guitypes;


/**
* cast/cdl/guitypes/DrawBatchHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from guitypes.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class DrawBatchHelper
{
  private static String  _id = "IDL:cast/cdl/guitypes/DrawBatch:1.0";

  public static void insert (org.omg.CORBA.Any a, cast.cdl.guitypes.DrawBatch that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static cast.cdl.guitypes.DrawBatch extract (org.omg.CORBA.Any a)
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
          org.omg.CORBA.StructMember[] _members0 = new org.omg.CORBA.StructMember [13];
          org.omg.CORBA.TypeCode _tcOf_members0 = null;
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _members0[0] = new org.omg.CORBA.StructMember (
            "m_compID",
            _tcOf_members0,
            null);
          _tcOf_members0 = cast.cdl.guitypes.RGBImageHelper.type ();
          _members0[1] = new org.omg.CORBA.StructMember (
            "m_image",
            _tcOf_members0,
            null);
          _tcOf_members0 = cast.cdl.guitypes.Text2DHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[2] = new org.omg.CORBA.StructMember (
            "m_text2Ds",
            _tcOf_members0,
            null);
          _tcOf_members0 = cast.cdl.guitypes.Point2DHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[3] = new org.omg.CORBA.StructMember (
            "m_point2Ds",
            _tcOf_members0,
            null);
          _tcOf_members0 = cast.cdl.guitypes.Line2DHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[4] = new org.omg.CORBA.StructMember (
            "m_line2Ds",
            _tcOf_members0,
            null);
          _tcOf_members0 = cast.cdl.guitypes.Rect2DHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[5] = new org.omg.CORBA.StructMember (
            "m_rect2Ds",
            _tcOf_members0,
            null);
          _tcOf_members0 = cast.cdl.guitypes.Polygon2DHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[6] = new org.omg.CORBA.StructMember (
            "m_poly2Ds",
            _tcOf_members0,
            null);
          _tcOf_members0 = cast.cdl.guitypes.Text3DHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[7] = new org.omg.CORBA.StructMember (
            "m_text3Ds",
            _tcOf_members0,
            null);
          _tcOf_members0 = cast.cdl.guitypes.Point3DHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[8] = new org.omg.CORBA.StructMember (
            "m_point3Ds",
            _tcOf_members0,
            null);
          _tcOf_members0 = cast.cdl.guitypes.Line3DHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[9] = new org.omg.CORBA.StructMember (
            "m_line3Ds",
            _tcOf_members0,
            null);
          _tcOf_members0 = cast.cdl.guitypes.Box3DHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[10] = new org.omg.CORBA.StructMember (
            "m_box3Ds",
            _tcOf_members0,
            null);
          _tcOf_members0 = cast.cdl.guitypes.Frame3DHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[11] = new org.omg.CORBA.StructMember (
            "m_frame3Ds",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[12] = new org.omg.CORBA.StructMember (
            "m_texts",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (cast.cdl.guitypes.DrawBatchHelper.id (), "DrawBatch", _members0);
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

  public static cast.cdl.guitypes.DrawBatch read (org.omg.CORBA.portable.InputStream istream)
  {
    cast.cdl.guitypes.DrawBatch value = new cast.cdl.guitypes.DrawBatch ();
    value.m_compID = istream.read_string ();
    value.m_image = cast.cdl.guitypes.RGBImageHelper.read (istream);
    int _len0 = istream.read_long ();
    value.m_text2Ds = new cast.cdl.guitypes.Text2D[_len0];
    for (int _o1 = 0;_o1 < value.m_text2Ds.length; ++_o1)
      value.m_text2Ds[_o1] = cast.cdl.guitypes.Text2DHelper.read (istream);
    int _len1 = istream.read_long ();
    value.m_point2Ds = new cast.cdl.guitypes.Point2D[_len1];
    for (int _o2 = 0;_o2 < value.m_point2Ds.length; ++_o2)
      value.m_point2Ds[_o2] = cast.cdl.guitypes.Point2DHelper.read (istream);
    int _len2 = istream.read_long ();
    value.m_line2Ds = new cast.cdl.guitypes.Line2D[_len2];
    for (int _o3 = 0;_o3 < value.m_line2Ds.length; ++_o3)
      value.m_line2Ds[_o3] = cast.cdl.guitypes.Line2DHelper.read (istream);
    int _len3 = istream.read_long ();
    value.m_rect2Ds = new cast.cdl.guitypes.Rect2D[_len3];
    for (int _o4 = 0;_o4 < value.m_rect2Ds.length; ++_o4)
      value.m_rect2Ds[_o4] = cast.cdl.guitypes.Rect2DHelper.read (istream);
    int _len4 = istream.read_long ();
    value.m_poly2Ds = new cast.cdl.guitypes.Polygon2D[_len4];
    for (int _o5 = 0;_o5 < value.m_poly2Ds.length; ++_o5)
      value.m_poly2Ds[_o5] = cast.cdl.guitypes.Polygon2DHelper.read (istream);
    int _len5 = istream.read_long ();
    value.m_text3Ds = new cast.cdl.guitypes.Text3D[_len5];
    for (int _o6 = 0;_o6 < value.m_text3Ds.length; ++_o6)
      value.m_text3Ds[_o6] = cast.cdl.guitypes.Text3DHelper.read (istream);
    int _len6 = istream.read_long ();
    value.m_point3Ds = new cast.cdl.guitypes.Point3D[_len6];
    for (int _o7 = 0;_o7 < value.m_point3Ds.length; ++_o7)
      value.m_point3Ds[_o7] = cast.cdl.guitypes.Point3DHelper.read (istream);
    int _len7 = istream.read_long ();
    value.m_line3Ds = new cast.cdl.guitypes.Line3D[_len7];
    for (int _o8 = 0;_o8 < value.m_line3Ds.length; ++_o8)
      value.m_line3Ds[_o8] = cast.cdl.guitypes.Line3DHelper.read (istream);
    int _len8 = istream.read_long ();
    value.m_box3Ds = new cast.cdl.guitypes.Box3D[_len8];
    for (int _o9 = 0;_o9 < value.m_box3Ds.length; ++_o9)
      value.m_box3Ds[_o9] = cast.cdl.guitypes.Box3DHelper.read (istream);
    int _len9 = istream.read_long ();
    value.m_frame3Ds = new cast.cdl.guitypes.Frame3D[_len9];
    for (int _o10 = 0;_o10 < value.m_frame3Ds.length; ++_o10)
      value.m_frame3Ds[_o10] = cast.cdl.guitypes.Frame3DHelper.read (istream);
    int _len10 = istream.read_long ();
    value.m_texts = new String[_len10];
    for (int _o11 = 0;_o11 < value.m_texts.length; ++_o11)
      value.m_texts[_o11] = istream.read_string ();
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, cast.cdl.guitypes.DrawBatch value)
  {
    ostream.write_string (value.m_compID);
    cast.cdl.guitypes.RGBImageHelper.write (ostream, value.m_image);
    ostream.write_long (value.m_text2Ds.length);
    for (int _i0 = 0;_i0 < value.m_text2Ds.length; ++_i0)
      cast.cdl.guitypes.Text2DHelper.write (ostream, value.m_text2Ds[_i0]);
    ostream.write_long (value.m_point2Ds.length);
    for (int _i1 = 0;_i1 < value.m_point2Ds.length; ++_i1)
      cast.cdl.guitypes.Point2DHelper.write (ostream, value.m_point2Ds[_i1]);
    ostream.write_long (value.m_line2Ds.length);
    for (int _i2 = 0;_i2 < value.m_line2Ds.length; ++_i2)
      cast.cdl.guitypes.Line2DHelper.write (ostream, value.m_line2Ds[_i2]);
    ostream.write_long (value.m_rect2Ds.length);
    for (int _i3 = 0;_i3 < value.m_rect2Ds.length; ++_i3)
      cast.cdl.guitypes.Rect2DHelper.write (ostream, value.m_rect2Ds[_i3]);
    ostream.write_long (value.m_poly2Ds.length);
    for (int _i4 = 0;_i4 < value.m_poly2Ds.length; ++_i4)
      cast.cdl.guitypes.Polygon2DHelper.write (ostream, value.m_poly2Ds[_i4]);
    ostream.write_long (value.m_text3Ds.length);
    for (int _i5 = 0;_i5 < value.m_text3Ds.length; ++_i5)
      cast.cdl.guitypes.Text3DHelper.write (ostream, value.m_text3Ds[_i5]);
    ostream.write_long (value.m_point3Ds.length);
    for (int _i6 = 0;_i6 < value.m_point3Ds.length; ++_i6)
      cast.cdl.guitypes.Point3DHelper.write (ostream, value.m_point3Ds[_i6]);
    ostream.write_long (value.m_line3Ds.length);
    for (int _i7 = 0;_i7 < value.m_line3Ds.length; ++_i7)
      cast.cdl.guitypes.Line3DHelper.write (ostream, value.m_line3Ds[_i7]);
    ostream.write_long (value.m_box3Ds.length);
    for (int _i8 = 0;_i8 < value.m_box3Ds.length; ++_i8)
      cast.cdl.guitypes.Box3DHelper.write (ostream, value.m_box3Ds[_i8]);
    ostream.write_long (value.m_frame3Ds.length);
    for (int _i9 = 0;_i9 < value.m_frame3Ds.length; ++_i9)
      cast.cdl.guitypes.Frame3DHelper.write (ostream, value.m_frame3Ds[_i9]);
    ostream.write_long (value.m_texts.length);
    for (int _i10 = 0;_i10 < value.m_texts.length; ++_i10)
      ostream.write_string (value.m_texts[_i10]);
  }

}
