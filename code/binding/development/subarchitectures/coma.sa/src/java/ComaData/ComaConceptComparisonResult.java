package ComaData;


/**
* ComaData/ComaConceptComparisonResult.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from ComaData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class ComaConceptComparisonResult implements org.omg.CORBA.portable.IDLEntity
{
  public ComaData.ComaConcept m_con1 = null;
  public ComaData.ComaConcept m_con2 = null;
  public cast.cdl.TriBool m_result = null;

  public ComaConceptComparisonResult ()
  {
  } // ctor

  public ComaConceptComparisonResult (ComaData.ComaConcept _m_con1, ComaData.ComaConcept _m_con2, cast.cdl.TriBool _m_result)
  {
    m_con1 = _m_con1;
    m_con2 = _m_con2;
    m_result = _m_result;
  } // ctor

} // class ComaConceptComparisonResult
