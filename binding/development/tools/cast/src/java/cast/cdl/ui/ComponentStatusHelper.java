package cast.cdl.ui;


/**
* cast/cdl/ui/ComponentStatusHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from CAST.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class ComponentStatusHelper
{
  private static String  _id = "IDL:cast/cdl/ui/ComponentStatus:1.0";

  public static void insert (org.omg.CORBA.Any a, cast.cdl.ui.ComponentStatus that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static cast.cdl.ui.ComponentStatus extract (org.omg.CORBA.Any a)
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
          org.omg.CORBA.StructMember[] _members0 = new org.omg.CORBA.StructMember [16];
          org.omg.CORBA.TypeCode _tcOf_members0 = null;
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (balt.corba.autogen.FrameworkBasics.FrameworkProcessIDHelper.id (), "FrameworkProcessID", _tcOf_members0);
          _members0[0] = new org.omg.CORBA.StructMember (
            "m_component",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _members0[1] = new org.omg.CORBA.StructMember (
            "m_subarchitecture",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_boolean);
          _members0[2] = new org.omg.CORBA.StructMember (
            "m_log",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_boolean);
          _members0[3] = new org.omg.CORBA.StructMember (
            "m_debug",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_boolean);
          _members0[4] = new org.omg.CORBA.StructMember (
            "m_locked",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_boolean);
          _members0[5] = new org.omg.CORBA.StructMember (
            "m_sleeping",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[6] = new org.omg.CORBA.StructMember (
            "m_changeQueue",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[7] = new org.omg.CORBA.StructMember (
            "m_totalChangeEventsFiltered",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[8] = new org.omg.CORBA.StructMember (
            "m_totalChangeEventsReceived",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[9] = new org.omg.CORBA.StructMember (
            "m_totalAdds",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[10] = new org.omg.CORBA.StructMember (
            "m_totalOverwrites",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[11] = new org.omg.CORBA.StructMember (
            "m_totalDeletes",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[12] = new org.omg.CORBA.StructMember (
            "m_totalReads",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[13] = new org.omg.CORBA.StructMember (
            "m_totalProposals",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[14] = new org.omg.CORBA.StructMember (
            "m_totalStarts",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[15] = new org.omg.CORBA.StructMember (
            "m_totalEnds",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (cast.cdl.ui.ComponentStatusHelper.id (), "ComponentStatus", _members0);
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

  public static cast.cdl.ui.ComponentStatus read (org.omg.CORBA.portable.InputStream istream)
  {
    cast.cdl.ui.ComponentStatus value = new cast.cdl.ui.ComponentStatus ();
    value.m_component = istream.read_string ();
    value.m_subarchitecture = istream.read_string ();
    value.m_log = istream.read_boolean ();
    value.m_debug = istream.read_boolean ();
    value.m_locked = istream.read_boolean ();
    value.m_sleeping = istream.read_boolean ();
    value.m_changeQueue = istream.read_long ();
    value.m_totalChangeEventsFiltered = istream.read_long ();
    value.m_totalChangeEventsReceived = istream.read_long ();
    value.m_totalAdds = istream.read_long ();
    value.m_totalOverwrites = istream.read_long ();
    value.m_totalDeletes = istream.read_long ();
    value.m_totalReads = istream.read_long ();
    value.m_totalProposals = istream.read_long ();
    value.m_totalStarts = istream.read_long ();
    value.m_totalEnds = istream.read_long ();
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, cast.cdl.ui.ComponentStatus value)
  {
    ostream.write_string (value.m_component);
    ostream.write_string (value.m_subarchitecture);
    ostream.write_boolean (value.m_log);
    ostream.write_boolean (value.m_debug);
    ostream.write_boolean (value.m_locked);
    ostream.write_boolean (value.m_sleeping);
    ostream.write_long (value.m_changeQueue);
    ostream.write_long (value.m_totalChangeEventsFiltered);
    ostream.write_long (value.m_totalChangeEventsReceived);
    ostream.write_long (value.m_totalAdds);
    ostream.write_long (value.m_totalOverwrites);
    ostream.write_long (value.m_totalDeletes);
    ostream.write_long (value.m_totalReads);
    ostream.write_long (value.m_totalProposals);
    ostream.write_long (value.m_totalStarts);
    ostream.write_long (value.m_totalEnds);
  }

}
