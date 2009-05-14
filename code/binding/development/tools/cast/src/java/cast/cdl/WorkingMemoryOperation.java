package cast.cdl;


/**
* cast/cdl/WorkingMemoryOperation.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from CAST.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public class WorkingMemoryOperation implements org.omg.CORBA.portable.IDLEntity
{
  private        int __value;
  private static int __size = 5;
  private static cast.cdl.WorkingMemoryOperation[] __array = new cast.cdl.WorkingMemoryOperation [__size];

  public static final int _ADD = 0;
  public static final cast.cdl.WorkingMemoryOperation ADD = new cast.cdl.WorkingMemoryOperation(_ADD);
  public static final int _OVERWRITE = 1;
  public static final cast.cdl.WorkingMemoryOperation OVERWRITE = new cast.cdl.WorkingMemoryOperation(_OVERWRITE);
  public static final int _DELETE = 2;
  public static final cast.cdl.WorkingMemoryOperation DELETE = new cast.cdl.WorkingMemoryOperation(_DELETE);
  public static final int _GET = 3;
  public static final cast.cdl.WorkingMemoryOperation GET = new cast.cdl.WorkingMemoryOperation(_GET);
  public static final int _WILDCARD = 4;
  public static final cast.cdl.WorkingMemoryOperation WILDCARD = new cast.cdl.WorkingMemoryOperation(_WILDCARD);

  public int value ()
  {
    return __value;
  }

  public static cast.cdl.WorkingMemoryOperation from_int (int value)
  {
    if (value >= 0 && value < __size)
      return __array[value];
    else
      throw new org.omg.CORBA.BAD_PARAM ();
  }

  protected WorkingMemoryOperation (int value)
  {
    __value = value;
    __array[__value] = this;
  }
} // class WorkingMemoryOperation
