package balt.corba.autogen.FrameworkBasics;


/**
* balt/corba/autogen/FrameworkBasics/DescriptionListHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from FrameworkBasics.idl
* Donnerstag, 14. Mai 2009 22.09 Uhr CEST
*/


/**
   * Connection graph is a sequence of connections
   **/
abstract public class DescriptionListHelper
{
  private static String  _id = "IDL:FrameworkBasics/DescriptionList:1.0";

  public static void insert (org.omg.CORBA.Any a, balt.corba.autogen.FrameworkBasics.ProcessDescription[] that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static balt.corba.autogen.FrameworkBasics.ProcessDescription[] extract (org.omg.CORBA.Any a)
  {
    return read (a.create_input_stream ());
  }

  private static org.omg.CORBA.TypeCode __typeCode = null;
  synchronized public static org.omg.CORBA.TypeCode type ()
  {
    if (__typeCode == null)
    {
      __typeCode = balt.corba.autogen.FrameworkBasics.ProcessDescriptionHelper.type ();
      __typeCode = org.omg.CORBA.ORB.init ().create_sequence_tc (0, __typeCode);
      __typeCode = org.omg.CORBA.ORB.init ().create_alias_tc (balt.corba.autogen.FrameworkBasics.DescriptionListHelper.id (), "DescriptionList", __typeCode);
    }
    return __typeCode;
  }

  public static String id ()
  {
    return _id;
  }

  public static balt.corba.autogen.FrameworkBasics.ProcessDescription[] read (org.omg.CORBA.portable.InputStream istream)
  {
    balt.corba.autogen.FrameworkBasics.ProcessDescription value[] = null;
    int _len0 = istream.read_long ();
    value = new balt.corba.autogen.FrameworkBasics.ProcessDescription[_len0];
    for (int _o1 = 0;_o1 < value.length; ++_o1)
      value[_o1] = balt.corba.autogen.FrameworkBasics.ProcessDescriptionHelper.read (istream);
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, balt.corba.autogen.FrameworkBasics.ProcessDescription[] value)
  {
    ostream.write_long (value.length);
    for (int _i0 = 0;_i0 < value.length; ++_i0)
      balt.corba.autogen.FrameworkBasics.ProcessDescriptionHelper.write (ostream, value[_i0]);
  }

}
