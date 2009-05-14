package Planner;


/**
* Planner/PlanningStateHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from Planner.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class PlanningStateHelper
{
  private static String  _id = "IDL:Planner/PlanningState:1.0";

  public static void insert (org.omg.CORBA.Any a, Planner.PlanningState that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static Planner.PlanningState extract (org.omg.CORBA.Any a)
  {
    return read (a.create_input_stream ());
  }

  private static org.omg.CORBA.TypeCode __typeCode = null;
  synchronized public static org.omg.CORBA.TypeCode type ()
  {
    if (__typeCode == null)
    {
      __typeCode = org.omg.CORBA.ORB.init ().create_enum_tc (Planner.PlanningStateHelper.id (), "PlanningState", new String[] { "OLD_PLAN_KEPT", "CHANGED_PLAN"} );
    }
    return __typeCode;
  }

  public static String id ()
  {
    return _id;
  }

  public static Planner.PlanningState read (org.omg.CORBA.portable.InputStream istream)
  {
    return Planner.PlanningState.from_int (istream.read_long ());
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, Planner.PlanningState value)
  {
    ostream.write_long (value.value ());
  }

}
