package BindingFeatures;


/**
* BindingFeatures/Name.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from BindingFeatures.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class Name implements org.omg.CORBA.portable.IDLEntity
{
  public String m_name = null;
  public BindingFeaturesCommon.ParentFeature m_parent = null;

  public Name ()
  {
  } // ctor

  public Name (String _m_name, BindingFeaturesCommon.ParentFeature _m_parent)
  {
    m_name = _m_name;
    m_parent = _m_parent;
  } // ctor

} // class Name
