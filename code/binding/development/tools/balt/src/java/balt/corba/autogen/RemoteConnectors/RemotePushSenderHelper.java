package balt.corba.autogen.RemoteConnectors;


/**
* balt/corba/autogen/RemoteConnectors/RemotePushSenderHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from RemoteConnector.idl
* Donnerstag, 14. Mai 2009 22.09 Uhr CEST
*/


/**
     * Interface to define an object that can send push information over
     * a remote connection.
    **/
abstract public class RemotePushSenderHelper
{
  private static String  _id = "IDL:RemoteConnectors/RemotePushSender:1.0";

  public static void insert (org.omg.CORBA.Any a, balt.corba.autogen.RemoteConnectors.RemotePushSender that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static balt.corba.autogen.RemoteConnectors.RemotePushSender extract (org.omg.CORBA.Any a)
  {
    return read (a.create_input_stream ());
  }

  private static org.omg.CORBA.TypeCode __typeCode = null;
  synchronized public static org.omg.CORBA.TypeCode type ()
  {
    if (__typeCode == null)
    {
      __typeCode = org.omg.CORBA.ORB.init ().create_interface_tc (balt.corba.autogen.RemoteConnectors.RemotePushSenderHelper.id (), "RemotePushSender");
    }
    return __typeCode;
  }

  public static String id ()
  {
    return _id;
  }

  public static balt.corba.autogen.RemoteConnectors.RemotePushSender read (org.omg.CORBA.portable.InputStream istream)
  {
    return narrow (istream.read_Object (_RemotePushSenderStub.class));
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, balt.corba.autogen.RemoteConnectors.RemotePushSender value)
  {
    ostream.write_Object ((org.omg.CORBA.Object) value);
  }

  public static balt.corba.autogen.RemoteConnectors.RemotePushSender narrow (org.omg.CORBA.Object obj)
  {
    if (obj == null)
      return null;
    else if (obj instanceof balt.corba.autogen.RemoteConnectors.RemotePushSender)
      return (balt.corba.autogen.RemoteConnectors.RemotePushSender)obj;
    else if (!obj._is_a (id ()))
      throw new org.omg.CORBA.BAD_PARAM ();
    else
    {
      org.omg.CORBA.portable.Delegate delegate = ((org.omg.CORBA.portable.ObjectImpl)obj)._get_delegate ();
      balt.corba.autogen.RemoteConnectors._RemotePushSenderStub stub = new balt.corba.autogen.RemoteConnectors._RemotePushSenderStub ();
      stub._set_delegate(delegate);
      return stub;
    }
  }

  public static balt.corba.autogen.RemoteConnectors.RemotePushSender unchecked_narrow (org.omg.CORBA.Object obj)
  {
    if (obj == null)
      return null;
    else if (obj instanceof balt.corba.autogen.RemoteConnectors.RemotePushSender)
      return (balt.corba.autogen.RemoteConnectors.RemotePushSender)obj;
    else
    {
      org.omg.CORBA.portable.Delegate delegate = ((org.omg.CORBA.portable.ObjectImpl)obj)._get_delegate ();
      balt.corba.autogen.RemoteConnectors._RemotePushSenderStub stub = new balt.corba.autogen.RemoteConnectors._RemotePushSenderStub ();
      stub._set_delegate(delegate);
      return stub;
    }
  }

}
