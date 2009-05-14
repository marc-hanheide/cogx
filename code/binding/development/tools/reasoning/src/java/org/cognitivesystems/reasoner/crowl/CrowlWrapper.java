package org.cognitivesystems.reasoner.crowl;

import java.io.PrintStream;
import java.util.TreeSet;
import java.util.Iterator;
import java.util.Set;

import org.cognitivesystems.reasoner.base.*;

import com.hp.hpl.jena.ontology.Individual;
import com.hp.hpl.jena.query.QuerySolution;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.rdf.model.Literal;
import com.hp.hpl.jena.rdf.model.Resource;

import de.dfki.lt.crowl.*;
//import de.dfki.lt.crowl.ConfigError;
//import de.dfki.lt.crowl.Crowl;

public class CrowlWrapper extends Object implements ReasonerInterface {

	private Crowl  m_mycrowl;
//	private String m_ontologyFile = "subarchitectures/coma.sa/ontologies/officeenv_wn.owl";
	private OntologyMemberFactory m_ontoMemberFactory;
	private String m_ns;
	private String m_sep;
	private String m_crowlConfigFile = "subarchitectures/coma/ontologies/crowl.cfg";


	/**
	 * The class constructor that allows you to specify 
	 * the ontology file location and the ontology namespace.
	 * 
	 * @param _ns  The namespace URI or PREFIX of the ontology
	 * @param _sep The separator between namespace and name
	 * @param _crowlConfigFile  The cfg file with the details of which reasoner and ontology to use
	 * @throws ReasonerException 
	 */
	public CrowlWrapper(String _ns, String _sep, String _crowlConfigFile) throws ReasonerException {
//		m_ontologyFile = _ontologyFile;
//		System.out.println("CrowlWrapper: ontology file = "+m_ontologyFile);
		m_crowlConfigFile = _crowlConfigFile;
		System.out.println("CrowlConfigFile: ontology file = "+m_crowlConfigFile);
		m_ontoMemberFactory = new OntologyMemberFactory(_ns, _sep);
		m_ns = _ns;
		m_sep = _sep;
		try {
			m_mycrowl = new Crowl();
			System.out.println("CrowlWrapper: created new Crowl()");
			m_mycrowl.readConfig(m_crowlConfigFile);
			System.out.println("CrowlWrapper: read crowl config file = "+m_crowlConfigFile);
			m_mycrowl.prepare();
		} catch (Exception e) {
			throw new ReasonerException(e.getMessage()+e.getStackTrace());
		}
	}


	public void addInstance(ReasonerInstance _ins, ReasonerConcept _con) throws ReasonerException {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		m_mycrowl.execute("INSERT { " +
				_ins.getFullName() +
				" rdf:type " +
				_con.getFullName() +
		" }");
		if (isABoxInconsistent()) {
			m_mycrowl.execute("DELETE { " +
					_ins.getFullName() +
					" rdf:type " +
					_con.getFullName() +
			" }");
			m_mycrowl.execute("INSERT { " +
					_ins.getFullName() +
					" rdf:type " +
					" oe:Failure" +
			" }");
			throw new ReasonerException("ABox was inconsistent; retracted previous assertion.");
		}
		if (isABoxInconsistent()) {
			throw new RuntimeException("ABox still inconsistent!");
		}
	}

	public void assertRelation(ReasonerRelation _rel) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		m_mycrowl.execute("INSERT { " +
				_rel.getArg1().getFullName() +
				" " +
				_rel.getFullRelationName() +
				" " +
				_rel.getArg2().getFullName() +
		" }");
	}

	public ResultSet executeSPARQLQuery(String _query) {
		ResultSet results = (ResultSet) m_mycrowl.execute(_query);
		return results;
	}
	
	public Set<ReasonerConcept> getAllConcepts(ReasonerInstance _ins) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		ResultSet results = (ResultSet) m_mycrowl.execute("SELECT ?con WHERE { " +
				_ins.getFullName() + 
				" rdf:type " +
		" ?con . FILTER (!isblank(?con)) . FILTER (?con != rdfs:Resource) . FILTER (?con != owl:Thing) }");
		TreeSet<ReasonerConcept> _returnSet = new TreeSet<ReasonerConcept>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?con");
//			String _currAnswerNS = _currAnswerRersource.getNameSpace().substring(0, _currAnswerRersource.getNameSpace().length()-1);
//			String _currAnswerName = _currAnswerRersource.getLocalName();
//			String _currAnswerSep = _currAnswerRersource.toString().substring(_currAnswerNS.length());
//			_currAnswerSep = _currAnswerSep.substring(0, _currAnswerSep.length()-_currAnswerName.length());
//			_returnSet.add(OntologyMemberFactory.createConcept(_currAnswerNS,_currAnswerSep,_currAnswerName));
			try {
				_returnSet.add(m_ontoMemberFactory.createConcept(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString())));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	public Set<ReasonerConcept> getDirectConcepts(ReasonerInstance _ins) {
		return getMostSpecificConcepts(_ins);
	}


	public Set<ReasonerConcept> getMostSpecificConcepts(ReasonerInstance _ins) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		TreeSet<ReasonerConcept> _returnSet = new TreeSet<ReasonerConcept>();
		try {
			Individual _ind  = m_mycrowl.getOWLOntoModel().getIndividual(m_mycrowl.getOWLOntoModel().expandPrefix(_ins.getFullName().replace(m_ns, "")));
			for (Iterator i = _ind.listRDFTypes(true); i.hasNext(); ) {
				Resource _currAnswerRersource = (Resource) i.next();
				if (!_currAnswerRersource.isAnon()) {
//					String _currAnswerNS = _currAnswerRersource.getNameSpace().substring(0, _currAnswerRersource.getNameSpace().length()-1);
//					String _currAnswerName = _currAnswerRersource.getLocalName();
//					String _currAnswerSep = _currAnswerRersource.toString().substring(_currAnswerNS.length());
//					_currAnswerSep = _currAnswerSep.substring(0, _currAnswerSep.length()-_currAnswerName.length());
//					_returnSet.add(OntologyMemberFactory.createConcept(_currAnswerNS,_currAnswerSep,_currAnswerName));
					_returnSet.add(m_ontoMemberFactory.createConcept(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString())));
				}
			}
		} catch (ConfigError e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return _returnSet;
	}

	public Set<ReasonerConcept> getBasicLevelConcepts(ReasonerInstance _ins) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		ResultSet results = (ResultSet) m_mycrowl.execute("SELECT ?con WHERE { " +
				_ins.getFullName() + 
				" rdf:type " +
				" ?con . " +
				" ?con rdfs:subClassOf oe:BasicLevelCat . " +
		" FILTER (?con != oe:BasicLevelCat). FILTER (!isblank(?con)) . FILTER (?con != rdfs:Resource) . FILTER (?con != owl:Thing) }");
		TreeSet<ReasonerConcept> _returnSet = new TreeSet<ReasonerConcept>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?con");
			try {
				_returnSet.add(m_ontoMemberFactory.createConcept(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString())));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}


	public Set<ReasonerInstance> getInstances(ReasonerConcept _con) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		ResultSet results = (ResultSet) m_mycrowl.execute("SELECT ?ins WHERE { " +
				" ?ins " +
				" rdf:type " +
				_con.getFullName() + " }");
		TreeSet<ReasonerInstance> _returnSet = new TreeSet<ReasonerInstance>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_ontoMemberFactory.createInstance(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString())));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	public Set<ReasonerInstance> getInstances(Set<ReasonerConcept> _cons) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String _query = "SELECT ?ins WHERE { ";
		for (ReasonerConcept concept : _cons) {
			_query += " ?ins rdf:type " + concept.getFullName() + ". "; 			
		}
		_query += " }";
		ResultSet results = (ResultSet) m_mycrowl.execute(_query);
		TreeSet<ReasonerInstance> _returnSet = new TreeSet<ReasonerInstance>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_ontoMemberFactory.createInstance(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString())));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	public Set<ReasonerInstance> getInstancesByName(String _givenName) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String _query = "SELECT ?ins WHERE { " +
		" ?ins " +
		" oe:name " +
		"'"+ _givenName + "' }";
//		System.out.println(_query);
		ResultSet results = (ResultSet) m_mycrowl.execute(_query);
		TreeSet<ReasonerInstance> _returnSet = new TreeSet<ReasonerInstance>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_ontoMemberFactory.createInstance(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString())));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	
	public Set<ReasonerInstance> getInstancesByNumberTag(Integer _number) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String _query = "SELECT ?ins WHERE { " +
		" ?ins " +
		" oe:numberTag " +
		_number + " }";
//		System.out.println(_query);
		ResultSet results = (ResultSet) m_mycrowl.execute(_query);
		TreeSet<ReasonerInstance> _returnSet = new TreeSet<ReasonerInstance>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_ontoMemberFactory.createInstance(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString())));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	
	public Set<ReasonerInstance> getRelatedInstances(ReasonerInstance _ins) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "SELECT DISTINCT ?ins WHERE { " +
		" { " + _ins.getFullName() + " ?x ?ins . " +
		" ?ins rdf:type oe:Entity. ?x rdfs:subPropertyOf oe:oeRelation. }" +
		" UNION { ?ins ?y " + _ins.getFullName() +  " . " +
		" ?ins rdf:type oe:Entity. ?y rdfs:subPropertyOf oe:oeRelation. } ." +
		" FILTER (?ins != " + _ins.getFullName() + " ) }";
//		String query = "SELECT DISTINCT ?ins ?x ?y WHERE { " +
//		" { " + _ins.getFullName() + " ?x ?ins . " +
//		" ?ins rdf:type oe:Entity. ?x rdfs:subPropertyOf oe:oeRelation. }" +
//		" UNION { ?ins ?y " + _ins.getFullName() +  " . " +
//		" ?ins rdf:type oe:Entity. ?y rdfs:subPropertyOf oe:oeRelation. } ." +
//		" FILTER (?ins != " + _ins.getFullName() + " ) }";
//		String query = "SELECT DISTINCT ?ins WHERE { " +
//		" { " + _ins.getFullName() + " oe:oeRelation ?ins . " +
//		" ?ins rdf:type oe:Entity. }" +
//		" UNION { ?ins oe:oeRelation " + _ins.getFullName() +  " . " +
//		" ?ins rdf:type oe:Entity. } ." +
//		" FILTER (?ins != " + _ins.getFullName() + " ) }";
//		String query = "SELECT DISTINCT ?ins ?x ?y WHERE { " +
//		" { " + _ins.getFullName() + " ?x ?ins . " +
//		" ?ins rdf:type oe:Entity. ?x rdfs:subPropertyOf oe:oeRelation. }" +
//		" UNION { ?ins ?y " + _ins.getFullName() +  " . " +
//		" ?ins rdf:type oe:Entity. ?y rdfs:subPropertyOf oe:oeRelation. } ." +
//		" FILTER (?ins != " + _ins.getFullName() + " ) }";
//		System.out.println(query);
		ResultSet results = (ResultSet) m_mycrowl.execute(query);
		TreeSet<ReasonerInstance> _returnSet = new TreeSet<ReasonerInstance>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
//			Resource _currRelX = _qs.getResource("?x");
//			Resource _currRelY = _qs.getResource("?y");
			try {
//				System.out.println(_ins.getFullName() + " related to " + (_currAnswerRersource!=null ? m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()) : "null") + " via X="+(_currRelX!=null ? m_mycrowl.getOWLOntoModel().shortForm(_currRelX.toString()) : "null")+" and Y="+(_currRelY!=null ? m_mycrowl.getOWLOntoModel().shortForm(_currRelY.toString()) : "null"));
				_returnSet.add(m_ontoMemberFactory.createInstance(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString())));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	
	/* 
	 * This method returns a set of all instances that occur in object position
	 * for the given relation (with the given instance in subject position). 
	 * 
	 * (non-Javadoc)
	 * @see org.cognitivesystems.reasoner.base.ReasonerInterface#getRelatedInstances(org.cognitivesystems.reasoner.base.ReasonerInstance, org.cognitivesystems.reasoner.base.ReasonerRelation)
	 */
	public Set<ReasonerInstance> getRelatedInstances(ReasonerInstance _ins, ReasonerRelation _rel) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "SELECT DISTINCT ?ins WHERE { " +
		" { " + _ins.getFullName() + " " + _rel.getFullRelationName() + " ?ins . " +
		" ?ins rdf:type oe:Entity. }" +
//		" UNION { ?ins " + _rel.getFullRelationName() + " " + _ins.getFullName() +  " . " +
//		" ?ins rdf:type oe:Entity. } ." +
		" FILTER (?ins != " + _ins.getFullName() + " ) }";
//		System.out.println(query);
		ResultSet results = (ResultSet) m_mycrowl.execute(query);
		TreeSet<ReasonerInstance> _returnSet = new TreeSet<ReasonerInstance>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_ontoMemberFactory.createInstance(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString())));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	
	public Set<ReasonerInstance> getInverseRelatedInstances(ReasonerInstance _ins, ReasonerRelation _rel) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "SELECT DISTINCT ?ins WHERE { " +
//		" { " + _ins.getFullName() + " " + _rel.getFullRelationName() + " ?ins . " +
		" { ?ins " + _rel.getFullRelationName() + " " + _ins.getFullName() +  " . " +
		" ?ins rdf:type oe:Entity. }" +
//		" UNION { ?ins " + _rel.getFullRelationName() + " " + _ins.getFullName() +  " . " +
//		" ?ins rdf:type oe:Entity. } ." +
		" FILTER (?ins != " + _ins.getFullName() + " ) }";
//		System.out.println(query);
		ResultSet results = (ResultSet) m_mycrowl.execute(query);
		TreeSet<ReasonerInstance> _returnSet = new TreeSet<ReasonerInstance>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_ontoMemberFactory.createInstance(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString())));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}
	public Set<ReasonerInstance> getImmediateRelatedInstances(ReasonerInstance _ins, ReasonerRelation _rel) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "SELECT ?ins ?y WHERE { " +
		" { " + _ins.getFullName() + " " + _rel.getFullRelationName() + " ?ins . " +
		" ?ins rdf:type oe:Entity. }" +
		" OPTIONAL { ?y " + _rel.getFullRelationName() + " ?ins. " +
		_ins.getFullName() + " " + _rel.getFullRelationName() + " ?y . " +
		" FILTER(?y != ?ins). } " +
		" FILTER (!BOUND(?y)). " +
		" FILTER (!isblank(?ins)). }";
//		System.out.println(query);
		ResultSet results = (ResultSet) m_mycrowl.execute(query);
		TreeSet<ReasonerInstance> _returnSet = new TreeSet<ReasonerInstance>();
		while (results.hasNext()) {
//			System.out.println("results has next");
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
//				System.out.println(_currAnswerRersource);
				_returnSet.add(m_ontoMemberFactory.createInstance(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString())));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
//		System.out.println("result set has size " + _returnSet.size());
		return _returnSet;
	}

	public boolean areConceptsEquivalent(ReasonerConcept _concept1, ReasonerConcept _concept2) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE " +
		" { " + _concept1.getFullName() + " owl:equivalentClass " +  _concept2.getFullName() + " } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}


	public boolean conceptExists(ReasonerConcept _con) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE { " +
		_con.getFullName() + " rdf:type owl:Class } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}

	public boolean relationExists(ReasonerRelation _rel) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE { " +
		_rel.getFullRelationName() + " rdf:type owl:ObjectProperty } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}

	public boolean areInstancesRelated(ReasonerInstance _ins1, ReasonerInstance _ins2, ReasonerRelation _rel) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE { " +
		_ins1.getFullName() + " "+ _rel.getFullRelationName() + " " + _ins2.getFullName() + " } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}
 
	public boolean instanceExists(ReasonerInstance _ins) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE { " +
		_ins.getFullName() + " rdf:type owl:Thing } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}

	public boolean isInstanceOf(ReasonerInstance _instance, ReasonerConcept _concept) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE { " +
		_instance.getFullName() + " rdf:type " + _concept.getFullName() + " } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}


	public boolean isSubConcept(ReasonerConcept _concept1, ReasonerConcept _concept2) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE { " +
		_concept1.getFullName() + " rdfs:subClassOf " + _concept2.getFullName() + " } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}

	public boolean isSuperConcept(ReasonerConcept _concept1, ReasonerConcept _concept2) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE { " +
		_concept2.getFullName() + " rdfs:subClassOf " + _concept1.getFullName() + " } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}

	public boolean deleteInstance(ReasonerInstance _instance) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "DELETE { " + _instance.getFullName() + " ?x ?y . " +
						" ?a ?b " + _instance.getFullName() + " . }" +
						" WHERE { " + _instance.getFullName() + " ?x ?y . " +
						" ?a ?b " + _instance.getFullName() + " . }";
		m_mycrowl.execute(query);
		return (!instanceExists(_instance));
	}

	public boolean deleteRelation(ReasonerRelation _relation) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "DELETE { " + _relation.getArg1().getFullName() + 
									_relation.getFullRelationName() + 
									_relation.getArg2().getFullName() + " . }";
		m_mycrowl.execute(query);
		return (!getRelatedInstances(_relation.getArg1(), _relation).contains(_relation.getArg2()));
	}

	public Set<ReasonerInstance> getPredecessors(ReasonerInstance _ins) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "SELECT DISTINCT ?ins WHERE { " +
		" ?ins ?y " + _ins.getFullName() +  " . " +
		" ?ins rdf:type oe:Entity. ?y rdfs:subPropertyOf oe:oeRelation. " +
		" FILTER (?ins != " + _ins.getFullName() + " ) }";
		ResultSet results = (ResultSet) m_mycrowl.execute(query);
		TreeSet<ReasonerInstance> _returnSet = new TreeSet<ReasonerInstance>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_ontoMemberFactory.createInstance(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString())));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	public Set<ReasonerInstance> getSuccessors(ReasonerInstance _ins) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "SELECT DISTINCT ?ins WHERE { " +
		_ins.getFullName() + " ?y ?ins . " +
		" ?ins rdf:type oe:Entity. ?y rdfs:subPropertyOf oe:oeRelation. " +
		" FILTER (?ins != " + _ins.getFullName() + " ) }";
		ResultSet results = (ResultSet) m_mycrowl.execute(query);
		TreeSet<ReasonerInstance> _returnSet = new TreeSet<ReasonerInstance>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_ontoMemberFactory.createInstance(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString())));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	public Set<ReasonerConcept> getAllSubConcepts(ReasonerConcept _con) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "SELECT ?con WHERE {?con rdfs:subClassOf oe:Entity. " +
					   " ?con rdfs:subClassOf " + _con.getFullName() + ". " +	
					   " FILTER (!isblank(?con)) . FILTER (?con != rdfs:Resource) . "+
					   " FILTER (?con != owl:Thing). FILTER (?con != owl:Nothing) ."+
					   " FILTER (?con != " + _con.getFullName() + ") } ";
		ResultSet results = (ResultSet) m_mycrowl.execute(query);
		TreeSet<ReasonerConcept> _returnSet = new TreeSet<ReasonerConcept>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?con");
			try {
				_returnSet.add(m_ontoMemberFactory.createConcept(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString())));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;	}
	
	private boolean isABoxInconsistent() {
//		return (getInstances(m_ontoMemberFactory.createConcept("owl", ":", "Nothing")).size() > 0);
		ResultSet results = (ResultSet) m_mycrowl.execute("SELECT ?ins WHERE { " +
				" ?ins " +
				" rdf:type " +
				m_ontoMemberFactory.createConcept("owl", ":", "Nothing").getFullName() + " } ");
		TreeSet<ReasonerInstance> _returnSet = new TreeSet<ReasonerInstance>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_ontoMemberFactory.createInstance(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString())));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet.size()>0;
	}
	
	public Set<String> getNames(ReasonerInstance _ins) {
		String query = "SELECT DISTINCT ?name WHERE { " +
			_ins.getFullName() + " oe:name ?name . }";
//		System.out.println(query);
		ResultSet results = (ResultSet) m_mycrowl.execute(query);

		TreeSet<String> _returnSet = new TreeSet<String>();

		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Literal _currAnswerRersource = _qs.getLiteral("?name");
//			System.out.println(_currAnswerRersource);
			_returnSet.add(_currAnswerRersource.getString());
		}
		return _returnSet;
	}

	public Set<Integer> getNumberTags(ReasonerInstance _ins) {
		String query = "SELECT DISTINCT ?num WHERE { " +
			_ins.getFullName() + " oe:numberTag ?num . }";
//		System.out.println(query);
		ResultSet results = (ResultSet) m_mycrowl.execute(query);
		
		TreeSet<Integer> _returnSet = new TreeSet<Integer>();

		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Literal _currAnswerRersource = _qs.getLiteral("?num");
//			System.out.println(_currAnswerRersource);
			_returnSet.add(_currAnswerRersource.getInt());
		}
		return _returnSet;
	}

	public void addName(ReasonerInstance _ins, String name) {
		m_mycrowl.execute("INSERT { " +
				_ins.getFullName() +
				" oe:name \" " + name + " \" .}");
	}
	
	public void addNumberTag(ReasonerInstance _ins, Integer _number) {
		m_mycrowl.execute("INSERT { " +
				_ins.getFullName() +
				" oe:numberTag " + _number.toString() + " .}");
	}

	public Set<Object> getPropertyValues(ReasonerInstance _ins, String _prop) {
		String query = "SELECT DISTINCT ?val WHERE { " +
		_ins.getFullName() + " " + _prop + " ?val . }";
		ResultSet results = (ResultSet) m_mycrowl.execute(query);

		TreeSet<Object> _returnSet = new TreeSet<Object>();

		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?val");
			_returnSet.add(_currAnswerRersource);
		}
		return _returnSet;
	}
	
	public void addProperty(ReasonerInstance _ins, String _property, Object _value) {
		m_mycrowl.execute("INSERT { " +
				_ins.getFullName() +
				" " +  _property + " " + _value.toString() + " .}");
	}
	
	
	public void setLogging(boolean _logging) {
		// TODO Auto-generated method stub
	}

	public void setLogging(boolean _logging, PrintStream _outstream) {
		// TODO Auto-generated method stub
	}

}
