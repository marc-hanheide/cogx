package planning.autogen;


/**
* planning/autogen/ActionRegistrationHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from PlanningData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class ActionRegistrationHelper
{
  private static String  _id = "IDL:planning/autogen/ActionRegistration:1.0";

  public static void insert (org.omg.CORBA.Any a, planning.autogen.ActionRegistration that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static planning.autogen.ActionRegistration extract (org.omg.CORBA.Any a)
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
          org.omg.CORBA.StructMember[] _members0 = new org.omg.CORBA.StructMember [3];
          org.omg.CORBA.TypeCode _tcOf_members0 = null;
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (balt.corba.autogen.FrameworkBasics.FrameworkProcessIDHelper.id (), "FrameworkProcessID", _tcOf_members0);
          _members0[0] = new org.omg.CORBA.StructMember (
            "m_component",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (cast.cdl.SubarchitectureIDHelper.id (), "SubarchitectureID", _tcOf_members0);
          _members0[1] = new org.omg.CORBA.StructMember (
            "m_subarchitecture",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _members0[2] = new org.omg.CORBA.StructMember (
            "m_action",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (planning.autogen.ActionRegistrationHelper.id (), "ActionRegistration", _members0);
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

  public static planning.autogen.ActionRegistration read (org.omg.CORBA.portable.InputStream istream)
  {
    planning.autogen.ActionRegistration value = new planning.autogen.ActionRegistration ();
    value.m_component = istream.read_string ();
    value.m_subarchitecture = istream.read_string ();
    value.m_action = istream.read_string ();
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, planning.autogen.ActionRegistration value)
  {
    ostream.write_string (value.m_component);
    ostream.write_string (value.m_subarchitecture);
    ostream.write_string (value.m_action);
  }

}
