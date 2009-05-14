package BindingFeaturesCommon;


/**
* BindingFeaturesCommon/StartTimeHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingFeaturesCommon.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class StartTimeHelper
{
  private static String  _id = "IDL:BindingFeaturesCommon/StartTime:1.0";

  public static void insert (org.omg.CORBA.Any a, BindingFeaturesCommon.StartTime that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static BindingFeaturesCommon.StartTime extract (org.omg.CORBA.Any a)
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
          org.omg.CORBA.StructMember[] _members0 = new org.omg.CORBA.StructMember [1];
          org.omg.CORBA.TypeCode _tcOf_members0 = null;
          _tcOf_members0 = balt.corba.autogen.FrameworkBasics.BALTTimeHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (BindingFeaturesCommon.TimeSeqHelper.id (), "TimeSeq", _tcOf_members0);
          _members0[0] = new org.omg.CORBA.StructMember (
            "m_t",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (BindingFeaturesCommon.StartTimeHelper.id (), "StartTime", _members0);
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

  public static BindingFeaturesCommon.StartTime read (org.omg.CORBA.portable.InputStream istream)
  {
    BindingFeaturesCommon.StartTime value = new BindingFeaturesCommon.StartTime ();
    value.m_t = BindingFeaturesCommon.TimeSeqHelper.read (istream);
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, BindingFeaturesCommon.StartTime value)
  {
    BindingFeaturesCommon.TimeSeqHelper.write (ostream, value.m_t);
  }

}
