package org.cognitivesystems.reasoner.base;

public class OntologyInstance implements ReasonerInstance {

	private String m_namespace;
	private String m_sep;
	private String m_name;
	
	protected OntologyInstance(String _namespace, String _sep, String _name) {
		m_name = _name;
		m_namespace = _namespace;
		m_sep = _sep;
	}
	
	public String getFullName() {
		return m_namespace + m_sep + m_name;
	}

	public String getName() {
		return m_name;
	}

	public String getNamespace() {
		return m_namespace;
	}

	public int compareTo(Object _ins) {
		return this.getFullName().compareTo(((ReasonerInstance) _ins).getFullName());
	}
	
	public String toString() {
		return getName();
	}

	public String getSep() {
		return m_sep;
	}

}
