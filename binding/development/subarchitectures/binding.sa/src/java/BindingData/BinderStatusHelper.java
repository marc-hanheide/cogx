package BindingData;


/**
* BindingData/BinderStatusHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class BinderStatusHelper
{
  private static String  _id = "IDL:BindingData/BinderStatus:1.0";

  public static void insert (org.omg.CORBA.Any a, BindingData.BinderStatus that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static BindingData.BinderStatus extract (org.omg.CORBA.Any a)
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
          org.omg.CORBA.StructMember[] _members0 = new org.omg.CORBA.StructMember [5];
          org.omg.CORBA.TypeCode _tcOf_members0 = null;
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_boolean);
          _members0[0] = new org.omg.CORBA.StructMember (
            "m_stable",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[1] = new org.omg.CORBA.StructMember (
            "m_scoringTasks",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[2] = new org.omg.CORBA.StructMember (
            "m_bindingTasks",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[3] = new org.omg.CORBA.StructMember (
            "m_unboundProxies",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[4] = new org.omg.CORBA.StructMember (
            "m_boundProxies",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (BindingData.BinderStatusHelper.id (), "BinderStatus", _members0);
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

  public static BindingData.BinderStatus read (org.omg.CORBA.portable.InputStream istream)
  {
    BindingData.BinderStatus value = new BindingData.BinderStatus ();
    value.m_stable = istream.read_boolean ();
    value.m_scoringTasks = istream.read_long ();
    value.m_bindingTasks = istream.read_long ();
    value.m_unboundProxies = istream.read_long ();
    value.m_boundProxies = istream.read_long ();
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, BindingData.BinderStatus value)
  {
    ostream.write_boolean (value.m_stable);
    ostream.write_long (value.m_scoringTasks);
    ostream.write_long (value.m_bindingTasks);
    ostream.write_long (value.m_unboundProxies);
    ostream.write_long (value.m_boundProxies);
  }

}
