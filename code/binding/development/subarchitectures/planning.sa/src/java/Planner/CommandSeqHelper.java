package Planner;


/**
* Planner/CommandSeqHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from Planner.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class CommandSeqHelper
{
  private static String  _id = "IDL:Planner/CommandSeq:1.0";

  public static void insert (org.omg.CORBA.Any a, Planner.Command[] that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static Planner.Command[] extract (org.omg.CORBA.Any a)
  {
    return read (a.create_input_stream ());
  }

  private static org.omg.CORBA.TypeCode __typeCode = null;
  synchronized public static org.omg.CORBA.TypeCode type ()
  {
    if (__typeCode == null)
    {
      __typeCode = Planner.CommandHelper.type ();
      __typeCode = org.omg.CORBA.ORB.init ().create_sequence_tc (0, __typeCode);
      __typeCode = org.omg.CORBA.ORB.init ().create_alias_tc (Planner.CommandSeqHelper.id (), "CommandSeq", __typeCode);
    }
    return __typeCode;
  }

  public static String id ()
  {
    return _id;
  }

  public static Planner.Command[] read (org.omg.CORBA.portable.InputStream istream)
  {
    Planner.Command value[] = null;
    int _len0 = istream.read_long ();
    value = new Planner.Command[_len0];
    for (int _o1 = 0;_o1 < value.length; ++_o1)
      value[_o1] = Planner.CommandHelper.read (istream);
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, Planner.Command[] value)
  {
    ostream.write_long (value.length);
    for (int _i0 = 0;_i0 < value.length; ++_i0)
      Planner.CommandHelper.write (ostream, value[_i0]);
  }

}
