package motivation.idl;


/**
* motivation/idl/MotiveStatusHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from MotivationData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/


/**
       * Various states that a motive can be in. This is not based on
       * requirements yet, just a prototype structure.
       **/
abstract public class MotiveStatusHelper
{
  private static String  _id = "IDL:motivation/idl/MotiveStatus:1.0";

  public static void insert (org.omg.CORBA.Any a, motivation.idl.MotiveStatus that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static motivation.idl.MotiveStatus extract (org.omg.CORBA.Any a)
  {
    return read (a.create_input_stream ());
  }

  private static org.omg.CORBA.TypeCode __typeCode = null;
  synchronized public static org.omg.CORBA.TypeCode type ()
  {
    if (__typeCode == null)
    {
      __typeCode = org.omg.CORBA.ORB.init ().create_enum_tc (motivation.idl.MotiveStatusHelper.id (), "MotiveStatus", new String[] { "MOTIVE_COMPLETE", "MOTIVE_PROPOSED", "MOTIVE_QUEUED", "MOTIVE_REJECTED", "MOTIVE_ADOPTED"} );
    }
    return __typeCode;
  }

  public static String id ()
  {
    return _id;
  }

  public static motivation.idl.MotiveStatus read (org.omg.CORBA.portable.InputStream istream)
  {
    return motivation.idl.MotiveStatus.from_int (istream.read_long ());
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, motivation.idl.MotiveStatus value)
  {
    ostream.write_long (value.value ());
  }

}
