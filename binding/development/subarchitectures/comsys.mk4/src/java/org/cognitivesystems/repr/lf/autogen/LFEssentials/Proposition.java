package org.cognitivesystems.repr.lf.autogen.LFEssentials;


/**
* org/cognitivesystems/repr/lf/autogen/LFEssentials/Proposition.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from LFEssentials.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class Proposition implements org.omg.CORBA.portable.IDLEntity
{
  public String prop = null;
  public org.cognitivesystems.repr.lf.autogen.LFEssentials.ConnectiveType connective = null;

  public Proposition ()
  {
  } // ctor

  public Proposition (String _prop, org.cognitivesystems.repr.lf.autogen.LFEssentials.ConnectiveType _connective)
  {
    prop = _prop;
    connective = _connective;
  } // ctor

} // class Proposition
