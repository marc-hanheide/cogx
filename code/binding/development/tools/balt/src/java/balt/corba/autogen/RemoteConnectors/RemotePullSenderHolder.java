package balt.corba.autogen.RemoteConnectors;

/**
* balt/corba/autogen/RemoteConnectors/RemotePullSenderHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from RemoteConnector.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class RemotePullSenderHolder implements org.omg.CORBA.portable.Streamable
{
  public balt.corba.autogen.RemoteConnectors.RemotePullSender value = null;

  public RemotePullSenderHolder ()
  {
  }

  public RemotePullSenderHolder (balt.corba.autogen.RemoteConnectors.RemotePullSender initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = balt.corba.autogen.RemoteConnectors.RemotePullSenderHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    balt.corba.autogen.RemoteConnectors.RemotePullSenderHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return balt.corba.autogen.RemoteConnectors.RemotePullSenderHelper.type ();
  }

}
