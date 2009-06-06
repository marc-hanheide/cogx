package balt.corba.autogen.FrameworkBasics;


/**
* balt/corba/autogen/FrameworkBasics/ProcessLanguageHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from FrameworkBasics.idl
* Donnerstag, 14. Mai 2009 22.09 Uhr CEST
*/


/**
   * Definition of process languages.
   */
abstract public class ProcessLanguageHelper
{
  private static String  _id = "IDL:FrameworkBasics/ProcessLanguage:1.0";

  public static void insert (org.omg.CORBA.Any a, balt.corba.autogen.FrameworkBasics.ProcessLanguage that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static balt.corba.autogen.FrameworkBasics.ProcessLanguage extract (org.omg.CORBA.Any a)
  {
    return read (a.create_input_stream ());
  }

  private static org.omg.CORBA.TypeCode __typeCode = null;
  synchronized public static org.omg.CORBA.TypeCode type ()
  {
    if (__typeCode == null)
    {
      __typeCode = org.omg.CORBA.ORB.init ().create_enum_tc (balt.corba.autogen.FrameworkBasics.ProcessLanguageHelper.id (), "ProcessLanguage", new String[] { "CPP_PROCESS", "JAVA_PROCESS"} );
    }
    return __typeCode;
  }

  public static String id ()
  {
    return _id;
  }

  public static balt.corba.autogen.FrameworkBasics.ProcessLanguage read (org.omg.CORBA.portable.InputStream istream)
  {
    return balt.corba.autogen.FrameworkBasics.ProcessLanguage.from_int (istream.read_long ());
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, balt.corba.autogen.FrameworkBasics.ProcessLanguage value)
  {
    ostream.write_long (value.value ());
  }

}
