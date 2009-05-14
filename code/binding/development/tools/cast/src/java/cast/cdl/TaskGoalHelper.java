package cast.cdl;


/**
* cast/cdl/TaskGoalHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from CAST.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class TaskGoalHelper
{
  private static String  _id = "IDL:cast/cdl/TaskGoal:1.0";

  public static void insert (org.omg.CORBA.Any a, cast.cdl.TaskGoal that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static cast.cdl.TaskGoal extract (org.omg.CORBA.Any a)
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
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (cast.cdl.TaskIDHelper.id (), "TaskID", _tcOf_members0);
          _members0[0] = new org.omg.CORBA.StructMember (
            "m_id",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (cast.cdl.TaskNameHelper.id (), "TaskName", _tcOf_members0);
          _members0[1] = new org.omg.CORBA.StructMember (
            "m_goalName",
            _tcOf_members0,
            null);
          _tcOf_members0 = cast.cdl.WorkingMemoryAddressHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (cast.cdl.WorkingMemoryAddressListHelper.id (), "WorkingMemoryAddressList", _tcOf_members0);
          _members0[2] = new org.omg.CORBA.StructMember (
            "m_taskAddresses",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (cast.cdl.TaskGoalHelper.id (), "TaskGoal", _members0);
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

  public static cast.cdl.TaskGoal read (org.omg.CORBA.portable.InputStream istream)
  {
    cast.cdl.TaskGoal value = new cast.cdl.TaskGoal ();
    value.m_id = istream.read_string ();
    value.m_goalName = istream.read_string ();
    value.m_taskAddresses = cast.cdl.WorkingMemoryAddressListHelper.read (istream);
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, cast.cdl.TaskGoal value)
  {
    ostream.write_string (value.m_id);
    ostream.write_string (value.m_goalName);
    cast.cdl.WorkingMemoryAddressListHelper.write (ostream, value.m_taskAddresses);
  }

}
