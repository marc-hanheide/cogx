package NavData;


/**
* NavData/PlaceLaserFeatureExtractorCommandHelper.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from NavData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

abstract public class PlaceLaserFeatureExtractorCommandHelper
{
  private static String  _id = "IDL:NavData/PlaceLaserFeatureExtractorCommand:1.0";

  public static void insert (org.omg.CORBA.Any a, NavData.PlaceLaserFeatureExtractorCommand that)
  {
    org.omg.CORBA.portable.OutputStream out = a.create_output_stream ();
    a.type (type ());
    write (out, that);
    a.read_value (out.create_input_stream (), type ());
  }

  public static NavData.PlaceLaserFeatureExtractorCommand extract (org.omg.CORBA.Any a)
  {
    return read (a.create_input_stream ());
  }

  private static org.omg.CORBA.TypeCode __typeCode = null;
  private static boolean __active = false;
  synchronized public static org.omg.CORBA.TypeCode type ()
  {
    if (__typeCode == null)
    {
      synchronized (org.omg.CORBA.TypeCode.class)
      {
        if (__typeCode == null)
        {
          if (__active)
          {
            return org.omg.CORBA.ORB.init().create_recursive_tc ( _id );
          }
          __active = true;
          org.omg.CORBA.StructMember[] _members0 = new org.omg.CORBA.StructMember [1];
          org.omg.CORBA.TypeCode _tcOf_members0 = null;
          _tcOf_members0 = NavData.PlaceLaserFeatureExtractorCommandTypeHelper.type ();
          _members0[0] = new org.omg.CORBA.StructMember (
            "m_command",
            _tcOf_members0,
            null);
          __typeCode = org.omg.CORBA.ORB.init ().create_struct_tc (NavData.PlaceLaserFeatureExtractorCommandHelper.id (), "PlaceLaserFeatureExtractorCommand", _members0);
          __active = false;
        }
      }
    }
    return __typeCode;
  }

  public static String id ()
  {
    return _id;
  }

  public static NavData.PlaceLaserFeatureExtractorCommand read (org.omg.CORBA.portable.InputStream istream)
  {
    NavData.PlaceLaserFeatureExtractorCommand value = new NavData.PlaceLaserFeatureExtractorCommand ();
    value.m_command = NavData.PlaceLaserFeatureExtractorCommandTypeHelper.read (istream);
    return value;
  }

  public static void write (org.omg.CORBA.portable.OutputStream ostream, NavData.PlaceLaserFeatureExtractorCommand value)
  {
    NavData.PlaceLaserFeatureExtractorCommandTypeHelper.write (ostream, value.m_command);
  }

}
