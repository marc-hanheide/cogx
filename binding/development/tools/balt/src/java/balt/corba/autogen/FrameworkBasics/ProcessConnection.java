package balt.corba.autogen.FrameworkBasics;


/**
* balt/corba/autogen/FrameworkBasics/ProcessConnection.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from FrameworkBasics.idl
* Donnerstag, 14. Mai 2009 22.09 Uhr CEST
*/

public final class ProcessConnection implements org.omg.CORBA.portable.IDLEntity
{
  public balt.corba.autogen.FrameworkBasics.ProcessDescription m_senders[] = null;
  public balt.corba.autogen.FrameworkBasics.ProcessDescription m_receivers[] = null;
  public balt.corba.autogen.FrameworkBasics.FrameworkConnectionType m_connectionType = null;
  public String m_dataType = null;
  public String m_connectionID = null;

  public ProcessConnection ()
  {
  } // ctor

  public ProcessConnection (balt.corba.autogen.FrameworkBasics.ProcessDescription[] _m_senders, balt.corba.autogen.FrameworkBasics.ProcessDescription[] _m_receivers, balt.corba.autogen.FrameworkBasics.FrameworkConnectionType _m_connectionType, String _m_dataType, String _m_connectionID)
  {
    m_senders = _m_senders;
    m_receivers = _m_receivers;
    m_connectionType = _m_connectionType;
    m_dataType = _m_dataType;
    m_connectionID = _m_connectionID;
  } // ctor

} // class ProcessConnection
