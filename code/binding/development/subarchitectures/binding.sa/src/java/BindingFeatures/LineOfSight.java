package BindingFeatures;


/**
* BindingFeatures/LineOfSight.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingFeatures.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class LineOfSight implements org.omg.CORBA.portable.IDLEntity
{
  public BindingFeaturesCommon.Vector2 m_from = null;
  public BindingFeaturesCommon.Vector2 m_direction = null;
  public BindingFeaturesCommon.ReferenceFrame m_frame = null;
  public BindingFeaturesCommon.ParentFeature m_parent = null;

  public LineOfSight ()
  {
  } // ctor

  public LineOfSight (BindingFeaturesCommon.Vector2 _m_from, BindingFeaturesCommon.Vector2 _m_direction, BindingFeaturesCommon.ReferenceFrame _m_frame, BindingFeaturesCommon.ParentFeature _m_parent)
  {
    m_from = _m_from;
    m_direction = _m_direction;
    m_frame = _m_frame;
    m_parent = _m_parent;
  } // ctor

} // class LineOfSight
