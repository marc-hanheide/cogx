package coma.refex;

import java.util.Collection;
import java.util.HashMap;

public class GREAttributeTemplateFiller {
	
	public enum GREAttribute { 
		TYPE, 
		COLOUR, 
		SHAPE, 
		SIZE,
		TOPOLOGICAL_IN,
		TOPOLOGICAL_ON,
		TOPOLOGICAL_AT,
		TOPOLOGICAL_INCLUSION,
		SPATIAL_RELATION,
		OWNERSHIP,
		NEXT_TO,
		NEAR,
		NAME,
		NUMBER_TAG
	}; 	


	
	private HashMap<GREAttribute, String> m_templateMap;
	private HashMap<GREAttribute, Object> m_valueMap;  
	
	
	public GREAttributeTemplateFiller() {
		m_templateMap = new HashMap<GREAttribute, String>();

		m_templateMap.put(GREAttribute.TYPE, "Xval");
		m_templateMap.put(GREAttribute.COLOUR, "<Color>Xval");
		m_templateMap.put(GREAttribute.SHAPE, "<Shape>Xval");
		m_templateMap.put(GREAttribute.SIZE, "<Size>Xval");
		m_templateMap.put(GREAttribute.TOPOLOGICAL_INCLUSION, "<RelXmod>(Xval)");
		m_templateMap.put(GREAttribute.TOPOLOGICAL_IN, "<RelXmod>(Xval)");
		m_templateMap.put(GREAttribute.TOPOLOGICAL_ON, "<RelXmod>(Xval)");
		m_templateMap.put(GREAttribute.OWNERSHIP, "<Owner>(Xval)");
		m_templateMap.put(GREAttribute.NEXT_TO, "<NextTo>(Xval)");
		m_templateMap.put(GREAttribute.NEAR, "<NearTo>(Xval)");
		m_templateMap.put(GREAttribute.NAME, "<Name>Xval");
		m_templateMap.put(GREAttribute.NUMBER_TAG, "<NumberID>Xval");
		
		m_valueMap = new HashMap<GREAttribute, Object>();
	}
	

	
	public Collection<GREAttribute> getAttributes () { 
		return m_valueMap.keySet();
	}// end getAttributes
	
	
	/** 
	 The method <i>getAttributeValue</i> retrieves the value for the give attribute. The method
	 returns <tt>null</tt> if the attribute has not been set. 
	 */ 
	
	public Object getAttributeValue (GREAttribute att) { 
		Object result = null; 
		if (m_valueMap.containsKey(att)) { result = m_valueMap.get(att); } 
		return result;
	} // end getAttributeValue
	
	/** 
	 The method <i>setAttributeValue</i> stores the value for a particular
	 attribute in a map, separate from the template map. The attribute-value map
	 can be translated into a String representation using the method <i>fillTemplate</i> 
	 (with no arguments). 
	*/ 
	
	public void setAttributeValue (GREAttribute att, Object val) { 
		m_valueMap.put(att,val);
	} // end 

	public void setAttributeValue (GREAttribute att, String val) { 
		m_valueMap.put(att,(Object)val);
	} // end 
	
	
	/**
	 The method <i>fillTemplate</i> cycles over the key attributes in the value map, 
	 and fills the template with their values. The method returns a String representation
	 of the template, as a conjunction of attributes.
	 
	<p>
	 <b>NOTE:</b> The method currently assumes that all values can be mapped to String objects. 
	 
	*/ 
	
	public String fillTemplate () { 
		String result = "";
		for (GREAttribute _mapAtt : m_valueMap.keySet() ) { 	
			result = result + fillTemplate(_mapAtt,(String)m_valueMap.get(_mapAtt)) + " ^ "; 
		} // end for over keys
		return result;
	} // end fillTemplate	
	
	
	/**
	 The method <i>fillTemplate</i> returns a String representation for the attribute being set, 
	 with the given value. 
	 */ 
	
	public String OLDfillTemplate(GREAttribute _attribute, String _value) {
		if (_attribute.equals(GREAttribute.TOPOLOGICAL_INCLUSION)) {
			String _xmodCapped = _value.split(";")[1].substring(0, 1).toUpperCase() + 
									_value.split(";")[1].substring(1);
			return m_templateMap.get(_attribute).
			replaceAll("Xval", _value.split(";")[0]).
			replaceAll("Xmod", _xmodCapped);
		}
		else if (_attribute.equals(GREAttribute.TOPOLOGICAL_IN)) {
				String _xmodCapped = _value.split(";")[1].substring(0, 1).toUpperCase() + 
										_value.split(";")[1].substring(1);
				return m_templateMap.get(_attribute).
				replaceAll("Xval", _value.split(";")[0]).
				replaceAll("Xmod", _xmodCapped);
		}
		else if (_attribute.equals(GREAttribute.TOPOLOGICAL_ON)) {
				String _xmodCapped = _value.split(";")[1].substring(0, 1).toUpperCase() + 
										_value.split(";")[1].substring(1);
				return m_templateMap.get(_attribute).
				replaceAll("Xval", _value.split(";")[0]).
				replaceAll("Xmod", _xmodCapped);
		} else {
			return m_templateMap.get(_attribute).replaceAll("Xval", _value);
		}
	} // end fillTemplate

	/**
	 The method <i>fillTemplate</i> returns a String representation for the attribute being set, 
	 with the given value. 
	 */ 
	
	public String fillTemplate(GREAttribute _attribute, String _value) {
		if (_attribute.equals(GREAttribute.TOPOLOGICAL_INCLUSION)) {
			String _xmodCapped = capitalize(_value.split(";")[1].split(":")[1]);
			return m_templateMap.get(_attribute).
					replaceAll("Xval", capitalize(_value.split(";")[0].split(":")[1])).
					replaceAll("Xmod", _xmodCapped);
		}
		else if (_attribute.equals(GREAttribute.TOPOLOGICAL_IN)) {
				String _xmodCapped = capitalize(_value.split(";")[1].split(":")[1]);
				return m_templateMap.get(_attribute).
						replaceAll("Xval", capitalize(_value.split(";")[0].split(":")[1])).
						replaceAll("Xmod", _xmodCapped);
		}
		else if (_attribute.equals(GREAttribute.TOPOLOGICAL_ON)) {
				String _xmodCapped = capitalize(_value.split(";")[1].split(":")[1]);
				return m_templateMap.get(_attribute).
					replaceAll("Xval", capitalize(_value.split(";")[0].split(":")[1])).
					replaceAll("Xmod", _xmodCapped);
		} else {
			return m_templateMap.get(_attribute).replaceAll("Xval", _value.split(":")[1]);
		}
	} // end fillTemplate
	
	private String capitalize(String _string) {
		return _string.substring(0, 1).toUpperCase() + _string.substring(1);
	}

} // end class
