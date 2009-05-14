package org.cognitivesystems.repr.lf.autogen.LFPacking;


/**
* org/cognitivesystems/repr/lf/autogen/LFPacking/PackedLogicalFormHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from LFPacking.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class PackedLogicalFormHelper
{
  private static String  _id = "IDL:org/cognitivesystems/repr/lf/autogen/LFPacking/PackedLogicalForm:1.0";

  public static void insert (org.omg.CORBA.Any a, org.cognitivesystems.repr.lf.autogen.LFPacking.PackedLogicalForm that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static org.cognitivesystems.repr.lf.autogen.LFPacking.PackedLogicalForm extract (org.omg.CORBA.Any a)
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
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (org.cognitivesystems.repr.lf.autogen.LFPacking.PackedLogicalFormIdHelper.id (), "PackedLogicalFormId", _tcOf_members0);
          _members0[0] = new org.omg.CORBA.StructMember (
            "packedLFId",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.cognitivesystems.repr.lf.autogen.LFPacking.PackingNodeHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (org.cognitivesystems.repr.lf.autogen.LFPacking.PackingNodesHelper.id (), "PackingNodes", _tcOf_members0);
          _members0[1] = new org.omg.CORBA.StructMember (
            "pNodes",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (org.cognitivesystems.repr.lf.autogen.LFPacking.PackingNodeIdHelper.id (), "PackingNodeId", _tcOf_members0);
          _members0[2] = new org.omg.CORBA.StructMember (
            "root",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (org.cognitivesystems.repr.lf.autogen.LFPacking.PackedLogicalFormHelper.id (), "PackedLogicalForm", _members0);
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

  public static org.cognitivesystems.repr.lf.autogen.LFPacking.PackedLogicalForm read (org.omg.CORBA.portable.InputStream istream)
  {
    org.cognitivesystems.repr.lf.autogen.LFPacking.PackedLogicalForm value = new org.cognitivesystems.repr.lf.autogen.LFPacking.PackedLogicalForm ();
    value.packedLFId = istream.read_string ();
    value.pNodes = org.cognitivesystems.repr.lf.autogen.LFPacking.PackingNodesHelper.read (istream);
    value.root = istream.read_string ();
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, org.cognitivesystems.repr.lf.autogen.LFPacking.PackedLogicalForm value)
  {
    ostream.write_string (value.packedLFId);
    org.cognitivesystems.repr.lf.autogen.LFPacking.PackingNodesHelper.write (ostream, value.pNodes);
    ostream.write_string (value.root);
  }

}
