package BindingFeatures;


/**
* BindingFeatures/AreaIDHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingFeatures.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class AreaIDHelper
{
  private static String  _id = "IDL:BindingFeatures/AreaID:1.0";

  public static void insert (org.omg.CORBA.Any a, BindingFeatures.AreaID that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static BindingFeatures.AreaID extract (org.omg.CORBA.Any a)
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
          org.omg.CORBA.StructMember[] _members0 = new org.omg.CORBA.StructMember [2];
          org.omg.CORBA.TypeCode _tcOf_members0 = null;
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[0] = new org.omg.CORBA.StructMember (
            "m_id",
            _tcOf_members0,
            null);
          _tcOf_members0 = BindingFeaturesCommon.ParentFeatureHelper.type ();
          _members0[1] = new org.omg.CORBA.StructMember (
            "m_parent",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (BindingFeatures.AreaIDHelper.id (), "AreaID", _members0);
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

  public static BindingFeatures.AreaID read (org.omg.CORBA.portable.InputStream istream)
  {
    BindingFeatures.AreaID value = new BindingFeatures.AreaID ();
    value.m_id = istream.read_long ();
    value.m_parent = BindingFeaturesCommon.ParentFeatureHelper.read (istream);
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, BindingFeatures.AreaID value)
  {
    ostream.write_long (value.m_id);
    BindingFeaturesCommon.ParentFeatureHelper.write (ostream, value.m_parent);
  }

}
