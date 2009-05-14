package BindingFeatures;


/**
* BindingFeatures/CircularArea.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingFeatures.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class CircularArea implements org.omg.CORBA.portable.IDLEntity
{
  public BindingFeaturesCommon.Vector2 m_center = null;

  /// inside this, a coordinate would match
  public BindingFeaturesCommon.Vector2 m_innerRadius = null;

  /// inside this, a coordinate would at least not mismatch
  public BindingFeaturesCommon.Vector2 m_outerRadius = null;
  public BindingFeaturesCommon.ReferenceFrame m_frame = null;
  public BindingFeaturesCommon.ParentFeature m_parent = null;

  public CircularArea ()
  {
  } // ctor

  public CircularArea (BindingFeaturesCommon.Vector2 _m_center, BindingFeaturesCommon.Vector2 _m_innerRadius, BindingFeaturesCommon.Vector2 _m_outerRadius, BindingFeaturesCommon.ReferenceFrame _m_frame, BindingFeaturesCommon.ParentFeature _m_parent)
  {
    m_center = _m_center;
    m_innerRadius = _m_innerRadius;
    m_outerRadius = _m_outerRadius;
    m_frame = _m_frame;
    m_parent = _m_parent;
  } // ctor

} // class CircularArea
