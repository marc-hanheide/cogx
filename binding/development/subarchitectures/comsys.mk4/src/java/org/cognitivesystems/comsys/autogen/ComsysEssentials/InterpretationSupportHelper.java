package org.cognitivesystems.comsys.autogen.ComsysEssentials;


/**
* org/cognitivesystems/comsys/autogen/ComsysEssentials/InterpretationSupportHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from ComsysEssentials.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class InterpretationSupportHelper
{
  private static String  _id = "IDL:org/cognitivesystems/comsys/autogen/ComsysEssentials/InterpretationSupport:1.0";

  public static void insert (org.omg.CORBA.Any a, org.cognitivesystems.comsys.autogen.ComsysEssentials.InterpretationSupport that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static org.cognitivesystems.comsys.autogen.ComsysEssentials.InterpretationSupport extract (org.omg.CORBA.Any a)
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
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (org.cognitivesystems.comsys.autogen.ComsysEssentials.UniqueIdHelper.id (), "UniqueId", _tcOf_members0);
          _members0[0] = new org.omg.CORBA.StructMember (
            "plfId",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _members0[1] = new org.omg.CORBA.StructMember (
            "headNomVar",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _members0[2] = new org.omg.CORBA.StructMember (
            "depNomVar",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _members0[3] = new org.omg.CORBA.StructMember (
            "mode",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().get_primitive_tc (org.omg.CORBA.TCKind.tk_boolean);
          _members0[4] = new org.omg.CORBA.StructMember (
            "isSupported",
            _tcOf_members0,
            null);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[5] = new org.omg.CORBA.StructMember (
            "LFids",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (org.cognitivesystems.comsys.autogen.ComsysEssentials.InterpretationSupportHelper.id (), "InterpretationSupport", _members0);
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

  public static org.cognitivesystems.comsys.autogen.ComsysEssentials.InterpretationSupport read (org.omg.CORBA.portable.InputStream istream)
  {
    org.cognitivesystems.comsys.autogen.ComsysEssentials.InterpretationSupport value = new org.cognitivesystems.comsys.autogen.ComsysEssentials.InterpretationSupport ();
    value.plfId = istream.read_string ();
    value.headNomVar = istream.read_string ();
    value.depNomVar = istream.read_string ();
    value.mode = istream.read_string ();
    value.isSupported = istream.read_boolean ();
    int _len0 = istream.read_long ();
    value.LFids = new String[_len0];
    for (int _o1 = 0;_o1 < value.LFids.length; ++_o1)
      value.LFids[_o1] = istream.read_string ();
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, org.cognitivesystems.comsys.autogen.ComsysEssentials.InterpretationSupport value)
  {
    ostream.write_string (value.plfId);
    ostream.write_string (value.headNomVar);
    ostream.write_string (value.depNomVar);
    ostream.write_string (value.mode);
    ostream.write_boolean (value.isSupported);
    ostream.write_long (value.LFids.length);
    for (int _i0 = 0;_i0 < value.LFids.length; ++_i0)
      ostream.write_string (value.LFids[_i0]);
  }

}
