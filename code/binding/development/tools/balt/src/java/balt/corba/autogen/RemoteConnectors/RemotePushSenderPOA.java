package balt.corba.autogen.RemoteConnectors;


/**
* balt/corba/autogen/RemoteConnectors/RemotePushSenderPOA.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from RemoteConnector.idl
* Donnerstag, 14. Mai 2009 22.09 Uhr CEST
*/


/**
     * Interface to define an object that can send push information over
     * a remote connection.
    **/
public abstract class RemotePushSenderPOA extends org.omg.PortableServer.Servant
 implements balt.corba.autogen.RemoteConnectors.RemotePushSenderOperations, org.omg.CORBA.portable.InvokeHandler
{

  // Constructors

  private static java.util.Hashtable _methods = new java.util.Hashtable ();
  static
  {
    _methods.put ("setPushConnector", new java.lang.Integer (0));
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
       case 0:  // RemoteConnectors/RemotePushSender/setPushConnector
       {
         balt.corba.autogen.RemoteConnectors.RemotePushConnector _out = balt.corba.autogen.RemoteConnectors.RemotePushConnectorHelper.read (in);
         this.setPushConnector (_out);
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
    "IDL:RemoteConnectors/RemotePushSender:1.0"};

  public String[] _all_interfaces (org.omg.PortableServer.POA poa, byte[] objectId)
  {
    return (String[])__ids.clone ();
  }

  public RemotePushSender _this() 
  {
    return RemotePushSenderHelper.narrow(
    super._this_object());
  }

  public RemotePushSender _this(org.omg.CORBA.ORB orb) 
  {
    return RemotePushSenderHelper.narrow(
    super._this_object(orb));
  }


} // class RemotePushSenderPOA
