package org.cognitivesystems.reasoner.base;

public class OntologyRelation implements ReasonerRelation {

	private String m_namespace;
	private String m_sep;
	private String m_name;

	private ReasonerInstance m_arg1;
	private ReasonerInstance m_arg2;
	
	protected OntologyRelation(String _namespace, String _sep, String _name, ReasonerInstance _arg1, ReasonerInstance _arg2) {
		m_namespace = _namespace;
		m_sep = _sep;
		m_name = _name;
		m_arg1 = _arg1;
		m_arg2 = _arg2;
	}
	 
	public ReasonerInstance getArg1() {
		return m_arg1;
	}

	public ReasonerInstance getArg2() {
		return m_arg2;
	}

	public String getFullRelationName() {
		return m_namespace + m_sep + m_name;
	}

	public String getRelationName() {
		return m_name;
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#toString()
	 */
	public String toString() {
		
		return getRelationName()+"("+(m_arg1!=null ? m_arg1.getFullName() : "")+","+(m_arg2!=null ? m_arg2.getFullName() : "")+")";	}

	/* (non-Javadoc)
	 * @see java.lang.Comparable#compareTo(java.lang.Object)
	 */
	public int compareTo(Object _rel) {
		return this.toString().compareTo(((ReasonerRelation) _rel).toString());
	}

	public String getRelationNamespace() {
		return m_namespace;
	}

	public String getRelationSep() {
		return m_sep;
	}


}
