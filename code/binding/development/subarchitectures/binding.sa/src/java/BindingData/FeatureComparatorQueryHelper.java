package BindingData;


/**
* BindingData/FeatureComparatorQueryHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class FeatureComparatorQueryHelper
{
  private static String  _id = "IDL:BindingData/FeatureComparatorQuery:1.0";

  public static void insert (org.omg.CORBA.Any a, BindingData.FeatureComparatorQuery that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static BindingData.FeatureComparatorQuery extract (org.omg.CORBA.Any a)
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
          org.omg.CORBA.StructMember[] _members0 = new org.omg.CORBA.StructMember [4];
          org.omg.CORBA.TypeCode _tcOf_members0 = null;
          _tcOf_members0 = BindingData.FeaturePointerHelper.type ();
          _members0[0] = new org.omg.CORBA.StructMember (
            "m_knownFeature",
            _tcOf_members0,
            null);
          _tcOf_members0 = BindingData.FeaturePointerHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (BindingData.FeaturePointersHelper.id (), "FeaturePointers", _tcOf_members0);
          _members0[1] = new org.omg.CORBA.StructMember (
            "m_equivalentFeatures",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (cast.cdl.SubarchitectureIDHelper.id (), "SubarchitectureID", _tcOf_members0);
          _members0[2] = new org.omg.CORBA.StructMember (
            "m_bindingSubarchitectureID",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_boolean);
          _members0[3] = new org.omg.CORBA.StructMember (
            "m_answered",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (BindingData.FeatureComparatorQueryHelper.id (), "FeatureComparatorQuery", _members0);
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

  public static BindingData.FeatureComparatorQuery read (org.omg.CORBA.portable.InputStream istream)
  {
    BindingData.FeatureComparatorQuery value = new BindingData.FeatureComparatorQuery ();
    value.m_knownFeature = BindingData.FeaturePointerHelper.read (istream);
    value.m_equivalentFeatures = BindingData.FeaturePointersHelper.read (istream);
    value.m_bindingSubarchitectureID = istream.read_string ();
    value.m_answered = istream.read_boolean ();
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, BindingData.FeatureComparatorQuery value)
  {
    BindingData.FeaturePointerHelper.write (ostream, value.m_knownFeature);
    BindingData.FeaturePointersHelper.write (ostream, value.m_equivalentFeatures);
    ostream.write_string (value.m_bindingSubarchitectureID);
    ostream.write_boolean (value.m_answered);
  }

}
