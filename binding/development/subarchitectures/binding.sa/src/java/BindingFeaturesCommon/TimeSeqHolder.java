package BindingFeaturesCommon;


/**
* BindingFeaturesCommon/TimeSeqHolder.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingFeaturesCommon.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class TimeSeqHolder implements org.omg.CORBA.portable.Streamable
{
  public balt.corba.autogen.FrameworkBasics.BALTTime value[] = null;

  public TimeSeqHolder ()
  {
  }

  public TimeSeqHolder (balt.corba.autogen.FrameworkBasics.BALTTime[] initialValue)
  {
    value = initialValue;
  }

  public void _read (org.omg.CORBA.portable.InputStream i)
  {
    value = BindingFeaturesCommon.TimeSeqHelper.read (i);
  }

  public void _write (org.omg.CORBA.portable.OutputStream o)
  {
    BindingFeaturesCommon.TimeSeqHelper.write (o, value);
  }

  public org.omg.CORBA.TypeCode _type ()
  {
    return BindingFeaturesCommon.TimeSeqHelper.type ();
  }

}
