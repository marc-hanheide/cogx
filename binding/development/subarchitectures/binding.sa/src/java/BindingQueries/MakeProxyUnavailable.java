package BindingQueries;


/**
* BindingQueries/MakeProxyUnavailable.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingQueries.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class MakeProxyUnavailable implements org.omg.CORBA.portable.IDLEntity
{

  /// the proxy that should be made unavailable
  public String m_proxyID = null;

  public MakeProxyUnavailable ()
  {
  } // ctor

  public MakeProxyUnavailable (String _m_proxyID)
  {
    m_proxyID = _m_proxyID;
  } // ctor

} // class MakeProxyUnavailable
