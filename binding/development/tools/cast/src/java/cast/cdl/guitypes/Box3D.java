package cast.cdl.guitypes;


/**
* cast/cdl/guitypes/Box3D.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from guitypes.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class Box3D implements org.omg.CORBA.portable.IDLEntity
{
  public double m_cx = (double)0;
  public double m_cy = (double)0;
  public double m_cz = (double)0;
  public double m_sx = (double)0;
  public double m_sy = (double)0;
  public double m_sz = (double)0;
  public cast.cdl.guitypes.RGBColor m_color = null;
  public int m_flags = (int)0;

  public Box3D ()
  {
  } // ctor

  public Box3D (double _m_cx, double _m_cy, double _m_cz, double _m_sx, double _m_sy, double _m_sz, cast.cdl.guitypes.RGBColor _m_color, int _m_flags)
  {
    m_cx = _m_cx;
    m_cy = _m_cy;
    m_cz = _m_cz;
    m_sx = _m_sx;
    m_sy = _m_sy;
    m_sz = _m_sz;
    m_color = _m_color;
    m_flags = _m_flags;
  } // ctor

} // class Box3D
