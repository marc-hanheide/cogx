package balt.corba.autogen.RemoteConnectors;


/**
* balt/corba/autogen/RemoteConnectors/RemotePullSenderPOA.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from RemoteConnector.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public abstract class RemotePullSenderPOA extends org.omg.PortableServer.Servant
 implements balt.corba.autogen.RemoteConnectors.RemotePullSenderOperations, org.omg.CORBA.portable.InvokeHandler
{

  // Constructors

  private static java.util.Hashtable _methods = new java.util.Hashtable ();
  static
  {
    _methods.put ("setPullConnector", new java.lang.Integer (0));
  }

  public org.omg.CORBA.portable.OutputStream _invoke (String $method,
                                org.omg.CORBA.portable.InputStream in,
                                org.omg.CORBA.portable.ResponseHandler $rh)
  {
    org.omg.CORBA.portable.OutputStream out = null;
    java.lang.Integer __method = (java.lang.Integer)_methods.get ($method);
    if (__method == null)
      throw new org.omg.CORBA.BAD_OPERATION (0, org.omg.CORBA.CompletionStatus.COMPLETED_MAYBE);

    switch (__method.intValue ())
    {

  /**
       * Set the connection object to be used. "set" is not correct, as
       * it could be many-to-many.
       *
       * \todo Fix this,
       *
       * @param _out The connector to use.
       */
       case 0:  // RemoteConnectors/RemotePullSender/setPullConnector
       {
         balt.corba.autogen.RemoteConnectors.RemotePullConnector _out = balt.corba.autogen.RemoteConnectors.RemotePullConnectorHelper.read (in);
         this.setPullConnector (_out);
         out = $rh.createReply();
         break;
       }

       default:
         throw new org.omg.CORBA.BAD_OPERATION (0, org.omg.CORBA.CompletionStatus.COMPLETED_MAYBE);
    }

    return out;
  } // _invoke

  // Type-specific CORBA::Object operations
  private static String[] __ids = {
    "IDL:RemoteConnectors/RemotePullSender:1.0"};

  public String[] _all_interfaces (org.omg.PortableServer.POA poa, byte[] objectId)
  {
    return (String[])__ids.clone ();
  }

  public RemotePullSender _this() 
  {
    return RemotePullSenderHelper.narrow(
    super._this_object());
  }

  public RemotePullSender _this(org.omg.CORBA.ORB orb) 
  {
    return RemotePullSenderHelper.narrow(
    super._this_object(orb));
  }


} // class RemotePullSenderPOA
