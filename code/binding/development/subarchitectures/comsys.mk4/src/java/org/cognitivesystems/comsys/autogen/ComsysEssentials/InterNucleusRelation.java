package org.cognitivesystems.comsys.autogen.ComsysEssentials;


/**
* org/cognitivesystems/comsys/autogen/ComsysEssentials/InterNucleusRelation.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from ComsysEssentials.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class InterNucleusRelation implements org.omg.CORBA.portable.IDLEntity
{
  public String sourceId = null;
  public String targetId = null;
  public String mode = null;

  public InterNucleusRelation ()
  {
  } // ctor

  public InterNucleusRelation (String _sourceId, String _targetId, String _mode)
  {
    sourceId = _sourceId;
    targetId = _targetId;
    mode = _mode;
  } // ctor

} // class InterNucleusRelation
