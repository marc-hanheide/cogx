package balt.corba.autogen.RemoteConnectors;


/**
* balt/corba/autogen/RemoteConnectors/RemotePullSenderPOATie.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from RemoteConnector.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public class RemotePullSenderPOATie extends RemotePullSenderPOA
{

  // Constructors

  public RemotePullSenderPOATie ( balt.corba.autogen.RemoteConnectors.RemotePullSenderOperations delegate ) {
      this._impl = delegate;
  }
  public RemotePullSenderPOATie ( balt.corba.autogen.RemoteConnectors.RemotePullSenderOperations delegate , org.omg.PortableServer.POA poa ) {
      this._impl = delegate;
      this._poa      = poa;
  }
  public balt.corba.autogen.RemoteConnectors.RemotePullSenderOperations _delegate() {
      return this._impl;
  }
  public void _delegate (balt.corba.autogen.RemoteConnectors.RemotePullSenderOperations delegate ) {
      this._impl = delegate;
  }
  public org.omg.PortableServer.POA _default_POA() {
      if(_poa != null) {
          return _poa;
      }
      else {
          return super._default_POA();
      }
  }

  /**
       * Set the connection object to be used. "set" is not correct, as
       * it could be many-to-many.
       *
       * \todo Fix this,
       *
       * @param _out The connector to use.
       */
  public void setPullConnector (balt.corba.autogen.RemoteConnectors.RemotePullConnector _out)
  {
    _impl.setPullConnector(_out);
  } // setPullConnector

  private balt.corba.autogen.RemoteConnectors.RemotePullSenderOperations _impl;
  private org.omg.PortableServer.POA _poa;

} // class RemotePullSenderPOATie
