package org.cognitivesystems.reasoner.base;

public class OntologyConcept implements ReasonerConcept {

	private String m_namespace;
	private String m_sep;
	private String m_name;
	
	protected OntologyConcept(String _namespace, String _sep, String _name) {
		m_name = _name;
		m_namespace = _namespace;
		m_sep = _sep;
	}
	
	public String getFullName() {
		return m_namespace + m_sep + capitalize(m_name);
	}

	public String getName() {
		return m_name;
	}

	public String getNamespace() {
		return m_namespace;
	}

	public int compareTo(Object _con) {
		return this.getFullName().compareTo(((ReasonerConcept) _con).getFullName());
	}
	
	public String toString() {
		return getName();
	}

	public String getSep() {
		return m_sep;
	}

	private String capitalize(String s) {
        if (s.length() == 0) return s;
        return s.substring(0, 1).toUpperCase() + s.substring(1);
    }
}
