package org.cognitivesystems.comsys.autogen.ComsysEssentials;


/**
* org/cognitivesystems/comsys/autogen/ComsysEssentials/PackedLFsHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from ComsysEssentials.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class PackedLFsHelper
{
  private static String  _id = "IDL:org/cognitivesystems/comsys/autogen/ComsysEssentials/PackedLFs:1.0";

  public static void insert (org.omg.CORBA.Any a, org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs extract (org.omg.CORBA.Any a)
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
          org.omg.CORBA.StructMember[] _members0 = new org.omg.CORBA.StructMember [8];
          org.omg.CORBA.TypeCode _tcOf_members0 = null;
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (org.cognitivesystems.comsys.autogen.ComsysEssentials.UniqueIdHelper.id (), "UniqueId", _tcOf_members0);
          _members0[0] = new org.omg.CORBA.StructMember (
            "id",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonStringLFPairHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[1] = new org.omg.CORBA.StructMember (
            "phonStringLFPairs",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonStringHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[2] = new org.omg.CORBA.StructMember (
            "nonParsablePhonStrings",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[3] = new org.omg.CORBA.StructMember (
            "stringPos",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.cognitivesystems.repr.lf.autogen.LFPacking.PackedLogicalFormHelper.type ();
          _members0[4] = new org.omg.CORBA.StructMember (
            "packedLF",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_long);
          _members0[5] = new org.omg.CORBA.StructMember (
            "finalized",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _members0[6] = new org.omg.CORBA.StructMember (
            "type",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.cognitivesystems.comsys.autogen.ComsysEssentials.NonStandardRulesAppliedForLFHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[7] = new org.omg.CORBA.StructMember (
            "nonStandardRulesForLF",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFsHelper.id (), "PackedLFs", _members0);
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

  public static org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs read (org.omg.CORBA.portable.InputStream istream)
  {
    org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs value = new org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs ();
    value.id = istream.read_string ();
    int _len0 = istream.read_long ();
    value.phonStringLFPairs = new org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonStringLFPair[_len0];
    for (int _o1 = 0;_o1 < value.phonStringLFPairs.length; ++_o1)
      value.phonStringLFPairs[_o1] = org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonStringLFPairHelper.read (istream);
    int _len1 = istream.read_long ();
    value.nonParsablePhonStrings = new org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonString[_len1];
    for (int _o2 = 0;_o2 < value.nonParsablePhonStrings.length; ++_o2)
      value.nonParsablePhonStrings[_o2] = org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonStringHelper.read (istream);
    value.stringPos = istream.read_long ();
    value.packedLF = org.cognitivesystems.repr.lf.autogen.LFPacking.PackedLogicalFormHelper.read (istream);
    value.finalized = istream.read_long ();
    value.type = istream.read_string ();
    int _len2 = istream.read_long ();
    value.nonStandardRulesForLF = new org.cognitivesystems.comsys.autogen.ComsysEssentials.NonStandardRulesAppliedForLF[_len2];
    for (int _o3 = 0;_o3 < value.nonStandardRulesForLF.length; ++_o3)
      value.nonStandardRulesForLF[_o3] = org.cognitivesystems.comsys.autogen.ComsysEssentials.NonStandardRulesAppliedForLFHelper.read (istream);
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs value)
  {
    ostream.write_string (value.id);
    ostream.write_long (value.phonStringLFPairs.length);
    for (int _i0 = 0;_i0 < value.phonStringLFPairs.length; ++_i0)
      org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonStringLFPairHelper.write (ostream, value.phonStringLFPairs[_i0]);
    ostream.write_long (value.nonParsablePhonStrings.length);
    for (int _i1 = 0;_i1 < value.nonParsablePhonStrings.length; ++_i1)
      org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonStringHelper.write (ostream, value.nonParsablePhonStrings[_i1]);
    ostream.write_long (value.stringPos);
    org.cognitivesystems.repr.lf.autogen.LFPacking.PackedLogicalFormHelper.write (ostream, value.packedLF);
    ostream.write_long (value.finalized);
    ostream.write_string (value.type);
    ostream.write_long (value.nonStandardRulesForLF.length);
    for (int _i2 = 0;_i2 < value.nonStandardRulesForLF.length; ++_i2)
      org.cognitivesystems.comsys.autogen.ComsysEssentials.NonStandardRulesAppliedForLFHelper.write (ostream, value.nonStandardRulesForLF[_i2]);
  }

}
