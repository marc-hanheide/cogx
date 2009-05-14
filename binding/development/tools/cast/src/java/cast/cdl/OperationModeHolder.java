package cast.cdl;

/**
* cast/cdl/OperationModeHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from CAST.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/


/*
    * Enum indicating how a working memory operation should be
    * performed.
    */
public final class OperationModeHolder implements org.omg.CORBA.portable.Streamable
{
  public cast.cdl.OperationMode value = null;

  public OperationModeHolder ()
  {
  }

  public OperationModeHolder (cast.cdl.OperationMode initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = cast.cdl.OperationModeHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    cast.cdl.OperationModeHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return cast.cdl.OperationModeHelper.type ();
  }

}
