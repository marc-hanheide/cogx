package BindingData;


/**
* BindingData/FeatureComparisonHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class FeatureComparisonHelper
{
  private static String  _id = "IDL:BindingData/FeatureComparison:1.0";

  public static void insert (org.omg.CORBA.Any a, BindingData.FeatureComparison that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static BindingData.FeatureComparison extract (org.omg.CORBA.Any a)
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
          org.omg.CORBA.StructMember[] _members0 = new org.omg.CORBA.StructMember [6];
          org.omg.CORBA.TypeCode _tcOf_members0 = null;
          _tcOf_members0 = BindingData.FeaturePointerHelper.type ();
          _members0[0] = new org.omg.CORBA.StructMember (
            "m_proxyFeature",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (cast.cdl.WorkingMemoryIDHelper.id (), "WorkingMemoryID", _tcOf_members0);
          _members0[1] = new org.omg.CORBA.StructMember (
            "m_proxyID",
            _tcOf_members0,
            null);
          _tcOf_members0 = BindingData.FeaturePointerHelper.type ();
          _members0[2] = new org.omg.CORBA.StructMember (
            "m_unionFeature",
            _tcOf_members0,
            null);
          _tcOf_members0 = cast.cdl.TriBoolHelper.type ();
          _members0[3] = new org.omg.CORBA.StructMember (
            "m_featuresEquivalent",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (cast.cdl.SubarchitectureIDHelper.id (), "SubarchitectureID", _tcOf_members0);
          _members0[4] = new org.omg.CORBA.StructMember (
            "m_bindingSubarchitectureID",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_boolean);
          _members0[5] = new org.omg.CORBA.StructMember (
            "m_insistOnExternalComparison",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (BindingData.FeatureComparisonHelper.id (), "FeatureComparison", _members0);
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

  public static BindingData.FeatureComparison read (org.omg.CORBA.portable.InputStream istream)
  {
    BindingData.FeatureComparison value = new BindingData.FeatureComparison ();
    value.m_proxyFeature = BindingData.FeaturePointerHelper.read (istream);
    value.m_proxyID = istream.read_string ();
    value.m_unionFeature = BindingData.FeaturePointerHelper.read (istream);
    value.m_featuresEquivalent = cast.cdl.TriBoolHelper.read (istream);
    value.m_bindingSubarchitectureID = istream.read_string ();
    value.m_insistOnExternalComparison = istream.read_boolean ();
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, BindingData.FeatureComparison value)
  {
    BindingData.FeaturePointerHelper.write (ostream, value.m_proxyFeature);
    ostream.write_string (value.m_proxyID);
    BindingData.FeaturePointerHelper.write (ostream, value.m_unionFeature);
    cast.cdl.TriBoolHelper.write (ostream, value.m_featuresEquivalent);
    ostream.write_string (value.m_bindingSubarchitectureID);
    ostream.write_boolean (value.m_insistOnExternalComparison);
  }

}
