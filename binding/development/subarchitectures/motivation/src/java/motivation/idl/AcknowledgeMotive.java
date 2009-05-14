package motivation.idl;


/**
* motivation/idl/AcknowledgeMotive.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from MotivationData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class AcknowledgeMotive implements org.omg.CORBA.portable.IDLEntity
{

  /**
         * The decision on the motive. Whether it has been accepted for
         * execution, rejected, or was already true.
         */
  public motivation.idl.MotiveAcknowledgement m_ack = null;

  /** 
         * What type is the motive that is being acknowledged 
         **/
  public motivation.idl.MotiveType m_motiveType = null;

  /**
         * A list of source entries that contributed to the
         * motive. E.g. the proxies in the motive sa.
         */
  public cast.cdl.WorkingMemoryPointer m_motiveCause[] = null;

  /**
         * The motive struct that is being worked on for you.
         */
  public cast.cdl.WorkingMemoryAddress m_motiveAddress = null;

  public AcknowledgeMotive ()
  {
  } // ctor

  public AcknowledgeMotive (motivation.idl.MotiveAcknowledgement _m_ack, motivation.idl.MotiveType _m_motiveType, cast.cdl.WorkingMemoryPointer[] _m_motiveCause, cast.cdl.WorkingMemoryAddress _m_motiveAddress)
  {
    m_ack = _m_ack;
    m_motiveType = _m_motiveType;
    m_motiveCause = _m_motiveCause;
    m_motiveAddress = _m_motiveAddress;
  } // ctor

} // class AcknowledgeMotive
