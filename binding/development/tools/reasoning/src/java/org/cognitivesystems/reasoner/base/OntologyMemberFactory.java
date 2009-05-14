package org.cognitivesystems.reasoner.base;

import java.util.HashSet;

public class OntologyMemberFactory {

	private String m_namespace;
	private String m_sep;
	
	
	/**
	 * This creates a factory that creates ontology members
	 * (instances, concepts, relations). You once specify
	 * the namespace and separator to use and then only
	 * pass the short name of the ontology member.
	 * 
	 * If you, however, pass a String that contains
	 * the separator (which usually happens when you
	 * get a member directly from the reasoner), 
	 * the previously set namespace will be ignored for 
	 * this one creation and instead the passed String 
	 * will be decomposed into name and namespace, and 
	 * those will be used for creating the ontology member.
	 * 
	 * There exist also static members that take three values
	 * (namespace, separator, name) and create the respective
	 * ontology member.
	 * 
	 * @param _namespace
	 * @param _sep
	 */
	public OntologyMemberFactory(String _namespace, String _sep) {
		m_namespace = _namespace;
		m_sep = _sep;
	}
	
	public ReasonerInstance createInstance(String _name) {
		if (_name.lastIndexOf(m_sep) >= 0) {
			if (_name.lastIndexOf(m_sep) > 0) {
				return new OntologyInstance(_name.split(m_sep)[0], m_sep, firstToLower(_name.split(m_sep)[1]));
			} else {
				return new OntologyInstance(m_namespace, m_sep, firstToLower(_name.split(m_sep)[1]));
			}
		} else {
			return new OntologyInstance(m_namespace, m_sep, firstToLower(_name));
		}
	}
	
	public ReasonerConcept createConcept(String _name) {
		if (_name.lastIndexOf(m_sep) >= 0) {
			if (_name.lastIndexOf(m_sep) > 0) {
				return new OntologyConcept(_name.split(m_sep)[0], m_sep, firstToLower(_name.split(m_sep)[1]));
			} else {
				return new OntologyConcept(m_namespace, m_sep, firstToLower(_name.split(m_sep)[1]));
			}
		} else {
			return new OntologyConcept(m_namespace, m_sep, firstToLower(_name));
		}
	}

	public ReasonerRelation createRelation(String _name) {
		if (_name.lastIndexOf(m_sep) >= 0) {
			if (_name.lastIndexOf(m_sep) > 0) {
				return new OntologyRelation(_name.split(m_sep)[0], m_sep, _name.split(m_sep)[1], null, null);
			} else {
				return new OntologyRelation(m_namespace, m_sep, _name.split(m_sep)[1], null, null);				
			}			
		} else {
			return new OntologyRelation(m_namespace, m_sep, _name, null, null);
		}
	}

	public ReasonerRelation createRelation(String _name, String _ins1, String _ins2) {
		if (_name.lastIndexOf(m_sep) >= 0) {
			if (_name.lastIndexOf(m_sep) > 0) {
				return new OntologyRelation(_name.split(m_sep)[0], m_sep, _name.split(m_sep)[1], createInstance(_ins1), createInstance(_ins2));
			} else {
				return new OntologyRelation(m_namespace, m_sep, _name.split(m_sep)[1], createInstance(_ins1), createInstance(_ins2));				
			}
		} else {
			return new OntologyRelation(m_namespace, m_sep, _name, createInstance(_ins1), createInstance(_ins2));
		}
	}

	public ReasonerRelation createRelation(String _name, ReasonerInstance _ins1, ReasonerInstance _ins2) {
		if (_name.lastIndexOf(m_sep) >= 0) {
			if (_name.lastIndexOf(m_sep) > 0) {
				return new OntologyRelation(_name.split(m_sep)[0], m_sep, _name.split(m_sep)[1], _ins1, _ins2);
			} else {
				return new OntologyRelation(m_namespace, m_sep, _name.split(m_sep)[1], _ins1, _ins2);
			}
		} else {
			return new OntologyRelation(m_namespace, m_sep, _name, _ins1, _ins2);
		}
	}
	
	// static members:	
	public static ReasonerInstance createInstance(String _namespace, String _sep, String _name) {
		return new OntologyInstance(_namespace, _sep, _name);
	}
	
	public static ReasonerConcept createConcept(String _namespace, String _sep, String _name) {
		return new OntologyConcept(_namespace, _sep, firstToLower(_name));
	}
	
	public static ReasonerRelation createRelation(String _namespace, String _sep, String _name, ReasonerInstance _ins1, ReasonerInstance _ins2) {
		return new OntologyRelation(_namespace, _sep, _name, _ins1, _ins2);
	}
	
	// helper methods
	private static String firstToLower(String s) {
        if (s.length() == 0) return s;
        return s.substring(0, 1).toLowerCase() + s.substring(1);
    }

}
