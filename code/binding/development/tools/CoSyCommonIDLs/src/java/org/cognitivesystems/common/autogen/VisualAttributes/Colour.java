package org.cognitivesystems.common.autogen.VisualAttributes;


/**
* org/cognitivesystems/common/autogen/VisualAttributes/Colour.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from VisualAttributes.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/


/**
   * Colour enumeration.
   */
public class Colour implements org.omg.CORBA.portable.IDLEntity
{
  private        int __value;
  private static int __size = 14;
  private static org.cognitivesystems.common.autogen.VisualAttributes.Colour[] __array = new org.cognitivesystems.common.autogen.VisualAttributes.Colour [__size];

  public static final int _RED = 0;
  public static final org.cognitivesystems.common.autogen.VisualAttributes.Colour RED = new org.cognitivesystems.common.autogen.VisualAttributes.Colour(_RED);
  public static final int _ORANGE = 1;
  public static final org.cognitivesystems.common.autogen.VisualAttributes.Colour ORANGE = new org.cognitivesystems.common.autogen.VisualAttributes.Colour(_ORANGE);
  public static final int _YELLOW = 2;
  public static final org.cognitivesystems.common.autogen.VisualAttributes.Colour YELLOW = new org.cognitivesystems.common.autogen.VisualAttributes.Colour(_YELLOW);
  public static final int _GREEN = 3;
  public static final org.cognitivesystems.common.autogen.VisualAttributes.Colour GREEN = new org.cognitivesystems.common.autogen.VisualAttributes.Colour(_GREEN);
  public static final int _BLUE = 4;
  public static final org.cognitivesystems.common.autogen.VisualAttributes.Colour BLUE = new org.cognitivesystems.common.autogen.VisualAttributes.Colour(_BLUE);
  public static final int _INDIGO = 5;
  public static final org.cognitivesystems.common.autogen.VisualAttributes.Colour INDIGO = new org.cognitivesystems.common.autogen.VisualAttributes.Colour(_INDIGO);
  public static final int _VIOLET = 6;
  public static final org.cognitivesystems.common.autogen.VisualAttributes.Colour VIOLET = new org.cognitivesystems.common.autogen.VisualAttributes.Colour(_VIOLET);
  public static final int _PINK = 7;
  public static final org.cognitivesystems.common.autogen.VisualAttributes.Colour PINK = new org.cognitivesystems.common.autogen.VisualAttributes.Colour(_PINK);
  public static final int _BLACK = 8;
  public static final org.cognitivesystems.common.autogen.VisualAttributes.Colour BLACK = new org.cognitivesystems.common.autogen.VisualAttributes.Colour(_BLACK);
  public static final int _WHITE = 9;
  public static final org.cognitivesystems.common.autogen.VisualAttributes.Colour WHITE = new org.cognitivesystems.common.autogen.VisualAttributes.Colour(_WHITE);
  public static final int _BROWN = 10;
  public static final org.cognitivesystems.common.autogen.VisualAttributes.Colour BROWN = new org.cognitivesystems.common.autogen.VisualAttributes.Colour(_BROWN);
  public static final int _GREY = 11;
  public static final org.cognitivesystems.common.autogen.VisualAttributes.Colour GREY = new org.cognitivesystems.common.autogen.VisualAttributes.Colour(_GREY);
  public static final int _PURPLE = 12;
  public static final org.cognitivesystems.common.autogen.VisualAttributes.Colour PURPLE = new org.cognitivesystems.common.autogen.VisualAttributes.Colour(_PURPLE);
  public static final int _UNKNOWN_COLOUR = 13;
  public static final org.cognitivesystems.common.autogen.VisualAttributes.Colour UNKNOWN_COLOUR = new org.cognitivesystems.common.autogen.VisualAttributes.Colour(_UNKNOWN_COLOUR);

  public int value ()
  {
    return __value;
  }

  public static org.cognitivesystems.common.autogen.VisualAttributes.Colour from_int (int value)
  {
    if (value >= 0 && value < __size)
      return __array[value];
    else
      throw new org.omg.CORBA.BAD_PARAM ();
  }

  protected Colour (int value)
  {
    __value = value;
    __array[__value] = this;
  }
} // class Colour
