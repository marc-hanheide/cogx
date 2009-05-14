package BindingFeaturesCommon;


/**
* BindingFeaturesCommon/TruthValue.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingFeaturesCommon.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public class TruthValue implements org.omg.CORBA.portable.IDLEntity
{
  private        int __value;
  private static int __size = 2;
  private static BindingFeaturesCommon.TruthValue[] __array = new BindingFeaturesCommon.TruthValue [__size];

  public static final int _NEGATIVE = 0;
  public static final BindingFeaturesCommon.TruthValue NEGATIVE = new BindingFeaturesCommon.TruthValue(_NEGATIVE);
  public static final int _POSITIVE = 1;
  public static final BindingFeaturesCommon.TruthValue POSITIVE = new BindingFeaturesCommon.TruthValue(_POSITIVE);

  public int value ()
  {
    return __value;
  }

  public static BindingFeaturesCommon.TruthValue from_int (int value)
  {
    if (value >= 0 && value < __size)
      return __array[value];
    else
      throw new org.omg.CORBA.BAD_PARAM ();
  }

  protected TruthValue (int value)
  {
    __value = value;
    __array[__value] = this;
  }
} // class TruthValue
