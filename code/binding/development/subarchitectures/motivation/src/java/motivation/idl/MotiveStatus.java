package motivation.idl;


/**
* motivation/idl/MotiveStatus.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from MotivationData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/


/**
       * Various states that a motive can be in. This is not based on
       * requirements yet, just a prototype structure.
       **/
public class MotiveStatus implements org.omg.CORBA.portable.IDLEntity
{
  private        int __value;
  private static int __size = 5;
  private static motivation.idl.MotiveStatus[] __array = new motivation.idl.MotiveStatus [__size];

  public static final int _MOTIVE_COMPLETE = 0;
  public static final motivation.idl.MotiveStatus MOTIVE_COMPLETE = new motivation.idl.MotiveStatus(_MOTIVE_COMPLETE);
  public static final int _MOTIVE_PROPOSED = 1;
  public static final motivation.idl.MotiveStatus MOTIVE_PROPOSED = new motivation.idl.MotiveStatus(_MOTIVE_PROPOSED);
  public static final int _MOTIVE_QUEUED = 2;
  public static final motivation.idl.MotiveStatus MOTIVE_QUEUED = new motivation.idl.MotiveStatus(_MOTIVE_QUEUED);
  public static final int _MOTIVE_REJECTED = 3;
  public static final motivation.idl.MotiveStatus MOTIVE_REJECTED = new motivation.idl.MotiveStatus(_MOTIVE_REJECTED);
  public static final int _MOTIVE_ADOPTED = 4;
  public static final motivation.idl.MotiveStatus MOTIVE_ADOPTED = new motivation.idl.MotiveStatus(_MOTIVE_ADOPTED);

  public int value ()
  {
    return __value;
  }

  public static motivation.idl.MotiveStatus from_int (int value)
  {
    if (value >= 0 && value < __size)
      return __array[value];
    else
      throw new org.omg.CORBA.BAD_PARAM ();
  }

  protected MotiveStatus (int value)
  {
    __value = value;
    __array[__value] = this;
  }
} // class MotiveStatus
