package balt.corba.autogen.RemoteConnectors;


/**
* balt/corba/autogen/RemoteConnectors/RemotePullReceiverOperations.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from RemoteConnector.idl
* Donnerstag, 14. Mai 2009 22.09 Uhr CEST
*/

public interface RemotePullReceiverOperations 
{

  /**
       * Handle a pull query from a remote source.
       *
       * @param _src The source of the pull query.
       * @param _query The query attached to the pull.
       */
  org.omg.CORBA.Any receivePullQuery (String _src, String _query);
} // interface RemotePullReceiverOperations
