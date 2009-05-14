package BindingData;


/**
* BindingData/ProxyPortsHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class ProxyPortsHelper
{
  private static String  _id = "IDL:BindingData/ProxyPorts:1.0";

  public static void insert (org.omg.CORBA.Any a, BindingData.ProxyPorts that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static BindingData.ProxyPorts extract (org.omg.CORBA.Any a)
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
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_string_tc (0);
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_alias_tc (cast.cdl.WorkingMemoryIDHelper.id (), "WorkingMemoryID", _tcOf_members0);
          _members0[0] = new org.omg.CORBA.StructMember (
            "m_ownerProxyID",
            _tcOf_members0,
            null);
          _tcOf_members0 = BindingData.ProxyPortHelper.type ();
          _tcOf_members0 = org.omg.CORBA.ORB.init ().create_sequence_tc (0, _tcOf_members0);
          _members0[1] = new org.omg.CORBA.StructMember (
            "m_ports",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (BindingData.ProxyPortsHelper.id (), "ProxyPorts", _members0);
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

  public static BindingData.ProxyPorts read (org.omg.CORBA.portable.InputStream istream)
  {
    BindingData.ProxyPorts value = new BindingData.ProxyPorts ();
    value.m_ownerProxyID = istream.read_string ();
    int _len0 = istream.read_long ();
    value.m_ports = new BindingData.ProxyPort[_len0];
    for (int _o1 = 0;_o1 < value.m_ports.length; ++_o1)
      value.m_ports[_o1] = BindingData.ProxyPortHelper.read (istream);
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, BindingData.ProxyPorts value)
  {
    ostream.write_string (value.m_ownerProxyID);
    ostream.write_long (value.m_ports.length);
    for (int _i0 = 0;_i0 < value.m_ports.length; ++_i0)
      BindingData.ProxyPortHelper.write (ostream, value.m_ports[_i0]);
  }

}
