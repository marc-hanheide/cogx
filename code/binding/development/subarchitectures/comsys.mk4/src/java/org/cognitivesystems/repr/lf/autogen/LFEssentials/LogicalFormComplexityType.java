package org.cognitivesystems.repr.lf.autogen.LFEssentials;


/**
* org/cognitivesystems/repr/lf/autogen/LFEssentials/LogicalFormComplexityType.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from LFEssentials.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public class LogicalFormComplexityType implements org.omg.CORBA.portable.IDLEntity
{
  private        int __value;
  private static int __size = 4;
  private static org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalFormComplexityType[] __array = new org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalFormComplexityType [__size];

  public static final int _SIMPLEX = 0;
  public static final org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalFormComplexityType SIMPLEX = new org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalFormComplexityType(_SIMPLEX);
  public static final int _COMPLX_CONJUNCTIVE = 1;
  public static final org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalFormComplexityType COMPLX_CONJUNCTIVE = new org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalFormComplexityType(_COMPLX_CONJUNCTIVE);
  public static final int _COMPLEX_DISJUNCTIVE = 2;
  public static final org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalFormComplexityType COMPLEX_DISJUNCTIVE = new org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalFormComplexityType(_COMPLEX_DISJUNCTIVE);
  public static final int _COMPLEX_MIXED = 3;
  public static final org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalFormComplexityType COMPLEX_MIXED = new org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalFormComplexityType(_COMPLEX_MIXED);

  public int value ()
  {
    return __value;
  }

  public static org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalFormComplexityType from_int (int value)
  {
    if (value >= 0 && value < __size)
      return __array[value];
    else
      throw new org.omg.CORBA.BAD_PARAM ();
  }

  protected LogicalFormComplexityType (int value)
  {
    __value = value;
    __array[__value] = this;
  }
} // class LogicalFormComplexityType
