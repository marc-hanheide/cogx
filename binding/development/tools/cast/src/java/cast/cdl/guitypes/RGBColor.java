package cast.cdl.guitypes;


/**
* cast/cdl/guitypes/RGBColor.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from guitypes.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class RGBColor implements org.omg.CORBA.portable.IDLEntity
{
  public int m_r = (int)0;

  // red value in the range [0, 255]
  public int m_g = (int)0;

  // green value in the range [0, 255]
  public int m_b = (int)0;

  public RGBColor ()
  {
  } // ctor

  public RGBColor (int _m_r, int _m_g, int _m_b)
  {
    m_r = _m_r;
    m_g = _m_g;
    m_b = _m_b;
  } // ctor

} // class RGBColor
