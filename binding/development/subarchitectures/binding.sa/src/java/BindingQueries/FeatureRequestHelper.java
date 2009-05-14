package BindingQueries;


/**
* BindingQueries/FeatureRequestHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingQueries.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class FeatureRequestHelper
{
  private static String  _id = "IDL:BindingQueries/FeatureRequest:1.0";

  public static void insert (org.omg.CORBA.Any a, BindingQueries.FeatureRequest that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static BindingQueries.FeatureRequest extract (org.omg.CORBA.Any a)
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
          org.omg.CORBA.StructMember[] _members0 = new org.omg.CORBA.StructMember [2];
          org.omg.CORBA.TypeCode _tcOf_members0 = null;
          _tcOf_members0 = BindingQueries.BasicQueryHelper.type ();
          _members0[0] = new org.omg.CORBA.StructMember (
            "m_request",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (cast.cdl.SubarchitectureIDHelper.id (), "SubarchitectureID", _tcOf_members0);
          _members0[1] = new org.omg.CORBA.StructMember (
            "m_fromSA",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (BindingQueries.FeatureRequestHelper.id (), "FeatureRequest", _members0);
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

  public static BindingQueries.FeatureRequest read (org.omg.CORBA.portable.InputStream istream)
  {
    BindingQueries.FeatureRequest value = new BindingQueries.FeatureRequest ();
    value.m_request = BindingQueries.BasicQueryHelper.read (istream);
    value.m_fromSA = istream.read_string ();
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, BindingQueries.FeatureRequest value)
  {
    BindingQueries.BasicQueryHelper.write (ostream, value.m_request);
    ostream.write_string (value.m_fromSA);
  }

}
