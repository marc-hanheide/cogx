package coma.reasoning;


import java.util.Collection;
import java.util.Iterator;
import java.util.Set;
import java.util.TreeSet;

import org.apache.log4j.Logger;

import cast.core.logging.ComponentLogger;

import com.hp.hpl.jena.ontology.Individual;
import com.hp.hpl.jena.query.QuerySolution;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.rdf.model.Literal;
import com.hp.hpl.jena.rdf.model.Resource;
import com.hp.hpl.jena.reasoner.rulesys.BuiltinRegistry;

import de.dfki.lt.crowl.ConfigError;
import de.dfki.lt.crowl.Crowl;
import de.dfki.lt.crowl.GenSym;
import de.dfki.lt.crowl.GetDirectClass;
import de.dfki.lt.crowl.WriteDirectClasses;

public class CrowlWrapper {
	private Crowl  m_mycrowl;
	private String m_crowlConfigFile = "subarchitectures/coma/ontologies/crowl.cfg";
	private boolean m_logging;
	private Logger m_logger = ComponentLogger.getLogger(CrowlWrapper.class);
	
	/**
	 * The class constructor that allows you to specify 
	 * the ontology file location and the ontology namespace.
	 * 
	 * @param _crowlConfigFile  The cfg file with the details of which reasoner and ontology to use
	 * @throws ReasonerException 
	 */
	public CrowlWrapper(String _crowlConfigFile, boolean _logging) throws ReasonerException {
		m_crowlConfigFile = _crowlConfigFile;
		m_logging = _logging;
		log("CrowlConfigFile: ontology file = "+m_crowlConfigFile);
		try {
			m_mycrowl = new Crowl();
		    BuiltinRegistry.theRegistry.register(new GenSym());
		    BuiltinRegistry.theRegistry.register(new GetDirectClass());
		    BuiltinRegistry.theRegistry.register(new WriteDirectClasses());
		    BuiltinRegistry.theRegistry.register(m_mycrowl.new DoorCleaner());
		    BuiltinRegistry.theRegistry.register(m_mycrowl.new RoomCleaner());

			log("CrowlWrapper: created new Crowl()");
			m_mycrowl.readConfig(m_crowlConfigFile);
			log("CrowlWrapper: read crowl config file = "+m_crowlConfigFile);
			m_mycrowl.prepare();
		} catch (Exception e) {
			throw new ReasonerException(e.getMessage()+e.getStackTrace());
		}
	}


	public void addInstance(String _ins, String _con) throws ReasonerException {
		if (isABoxInconsistent()) {
			TreeSet<String> inconsistencies = getABoxInconsistencies();
			StringBuilder inc_sb = new StringBuilder();
			for (String string : inconsistencies) {
				inc_sb.append(string);
				inc_sb.append(" ");
			}
			throw new RuntimeException("Inconsistent ABox! ABORT! Inconsistencies = " + inc_sb.toString());
		}
		
//		OntModel m;
//		try {
//			m = m_mycrowl.getOWLOntoModel();
//			OntClass c = m.createClass(_con);
//			
//			// first way: use a call on OntModel
////			Individual ind0 = m.createIndividual( _ins, c );
//			
////		// second way: use a call on OntClass
////			Individual ind1 = c.createIndividual( _ins );
//		} catch (ConfigError e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
		
// SPARQL way of doing it
		log("addInstance(" + _ins + ", " + _con + ") called.");
		
		String addQuery = "INSERT { " +
				_ins +
				" rdf:type " +
				_con +
		" }";
		
		log("going to execute SPARQL update query: " + addQuery);
		
		m_mycrowl.execute(addQuery);
		
		if (isABoxInconsistent()) {
			TreeSet<String> inconsistencies = getABoxInconsistencies();
			StringBuilder inc_sb = new StringBuilder();
			for (String string : inconsistencies) {
				inc_sb.append(string);
				inc_sb.append(" ");
			}

			m_mycrowl.execute("DELETE { " +
					_ins +
					" rdf:type " +
					_con +
			" }");
			m_mycrowl.execute("INSERT { " +
					_ins +
					" rdf:type " +
					" oe:Failure" +
			" }");
			
			TreeSet<String> inconsistencies_new = getABoxInconsistencies();
			StringBuilder inc_sb2 = new StringBuilder();
			for (String string : inconsistencies_new) {
				inc_sb2.append(string);
				inc_sb2.append(" ");
			}

			throw new ReasonerException("ABox was inconsistent; retracted previous assertion. Old inconsistencies = " + inc_sb.toString() + " -- current inconsistencies = " + inc_sb2.toString());
		}
		if (isABoxInconsistent()) {
			TreeSet<String> inconsistencies = getABoxInconsistencies();
			StringBuilder inc_sb = new StringBuilder();
			for (String string : inconsistencies) {
				inc_sb.append(string);
				inc_sb.append(" ");
			}
			throw new RuntimeException("ABox still inconsistent! Inconsistencies = " + inc_sb.toString());
		}
	}

	public void assertRelation(String _ins1, String _rel, String _ins2) {
		log("assertRelation(" + _ins1 + ", " + _rel + ", " + _ins2 +") called.");
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String updateQuery = "INSERT { " + 	_ins1 + " " + _rel + " " + _ins2 + ". }";
		log("going to execute SPARQL update query: " + updateQuery);
		m_mycrowl.execute(updateQuery);
//		m_mycrowl.executeAndPrintQuery(updateQuery, false);
		log("leaving assertRelation()");
	}

	public ResultSet executeSPARQLQuery(String _query) {
		ResultSet results = (ResultSet) m_mycrowl.execute(_query);
		return results;
	}
	
	public Set<String> getAllConcepts(String _ins) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		ResultSet results = (ResultSet) m_mycrowl.execute("SELECT ?con WHERE { " +
				_ins + 
				" rdf:type " +
		" ?con . FILTER (!isblank(?con)) . FILTER (?con != rdfs:Resource) . FILTER (?con != owl:Thing) }");
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?con");
//			String _currAnswerNS = _currAnswerRersource.getNameSpace().substring(0, _currAnswerRersource.getNameSpace().length()-1);
//			String _currAnswerName = _currAnswerRersource.getLocalName();
//			String _currAnswerSep = _currAnswerRersource.toString().substring(_currAnswerNS.length());
//			_currAnswerSep = _currAnswerSep.substring(0, _currAnswerSep.length()-_currAnswerName.length());
//			_returnSet.add(OntologyMemberFactory.createConcept(_currAnswerNS,_currAnswerSep,_currAnswerName));
			try {
				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	public Set<String> getDirectConcepts(String _ins) {
		return getMostSpecificConcepts(_ins);
	}


	public Set<String> getMostSpecificConcepts(String _ins) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		TreeSet<String> _returnSet = new TreeSet<String>();
		try {
			Individual _ind  = m_mycrowl.getOWLOntoModel().getIndividual(_ins);
			for (Iterator i = _ind.listRDFTypes(true); i.hasNext(); ) {
				Resource _currAnswerRersource = (Resource) i.next();
				if (!_currAnswerRersource.isAnon()) {
//					String _currAnswerNS = _currAnswerRersource.getNameSpace().substring(0, _currAnswerRersource.getNameSpace().length()-1);
//					String _currAnswerName = _currAnswerRersource.getLocalName();
//					String _currAnswerSep = _currAnswerRersource.toString().substring(_currAnswerNS.length());
//					_currAnswerSep = _currAnswerSep.substring(0, _currAnswerSep.length()-_currAnswerName.length());
//					_returnSet.add(OntologyMemberFactory.createConcept(_currAnswerNS,_currAnswerSep,_currAnswerName));
					_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
				}
			}
		} catch (ConfigError e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return _returnSet;
	}

	public Set<String> getBasicLevelConcepts(String _ins) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		ResultSet results = (ResultSet) m_mycrowl.execute("SELECT ?con WHERE { " +
				_ins + 
				" rdf:type " +
				" ?con . " +
				" ?con rdfs:subClassOf oe:BasicLevelCat . " +
		" FILTER (?con != oe:BasicLevelCat). FILTER (!isblank(?con)) . FILTER (?con != rdfs:Resource) . FILTER (?con != owl:Thing) }");
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?con");
			try {
				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}


	public Set<String> getInstances(String _con) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		ResultSet results = (ResultSet) m_mycrowl.execute("SELECT ?ins WHERE { " +
				" ?ins " +
				" rdf:type " +
				_con + " }");
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
//			try {
//				log(m_mycrowl.getOWLOntoModel().getBaseModel().shortForm(_currAnswerRersource.toString()));
//				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
				_returnSet.add(_currAnswerRersource.toString()); 
//				log("qname: " + m_mycrowl.getOWLOntoModel().qnameFor(_currAnswerRersource.toString()));
//				log("short form: " + m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
//				log("local name: " + _currAnswerRersource.getLocalName());				
//			} catch (ConfigError e) {
////				 TODO Auto-generated catch block
//				e.printStackTrace();
//			}
		}
		return _returnSet;
	}

	public Set<String> getInstances(Set<String> _cons) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String _query = "SELECT ?ins WHERE { ";
		for (String concept : _cons) {
			_query += " ?ins rdf:type " + concept + ". "; 			
		}
		_query += " }";
		ResultSet results = (ResultSet) m_mycrowl.execute(_query);
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	public Set<String> getInstancesByName(String _givenName) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String _query = "SELECT ?ins WHERE { " +
		" ?ins " +
		" oe:name " +
		"'"+ _givenName + "' }";
//		System.out.println(_query);
		ResultSet results = (ResultSet) m_mycrowl.execute(_query);
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	public Set<String> getInstancesByPropertyValue(String _property, String _value) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String _query = "SELECT ?ins WHERE { " +
		" ?ins " +
		" " + _property + 
		" " + _value + " }";
//		System.out.println(_query);
		ResultSet results = (ResultSet) m_mycrowl.execute(_query);
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	
	public Set<String> getInstancesByNumberTag(Integer _number) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String _query = "SELECT ?ins WHERE { " +
		" ?ins " +
		" oe:numberTag " +
		_number + " }";
//		System.out.println(_query);
		ResultSet results = (ResultSet) m_mycrowl.execute(_query);
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	
	public Set<String> getRelatedInstances(String _ins) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "SELECT DISTINCT ?ins WHERE { " +
		" { " + _ins + " ?x ?ins . " +
		" ?ins rdf:type owl:Thing. " + 
		" }" +
		" UNION { ?ins ?y " + _ins + " . " +
		" ?ins rdf:type owl:Thing.  " +
		"} ." +
		" FILTER (?ins != " + _ins + " ) }";
		
//		2009 09 22: 
//		String query = "SELECT DISTINCT ?ins WHERE { " +
//		" { " + _ins + " ?x ?ins . " +
//		" ?ins rdf:type oe:Entity. ?x rdfs:subPropertyOf oe:oeRelation. }" +
//		" UNION { ?ins ?y " + _ins +  " . " +
//		" ?ins rdf:type oe:Entity. ?y rdfs:subPropertyOf oe:oeRelation. } ." +
//		" FILTER (?ins != " + _ins + " ) }";

		
		
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
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
//			Resource _currRelX = _qs.getResource("?x");
//			Resource _currRelY = _qs.getResource("?y");
			try {
//				System.out.println(_ins.getFullName() + " related to " + (_currAnswerRersource!=null ? m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()) : "null") + " via X="+(_currRelX!=null ? m_mycrowl.getOWLOntoModel().shortForm(_currRelX.toString()) : "null")+" and Y="+(_currRelY!=null ? m_mycrowl.getOWLOntoModel().shortForm(_currRelY.toString()) : "null"));

				
				log(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
//				log(_currAnswerRersource.toString());
//				_returnSet.add(_currAnswerRersource.toString());

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
	public Set<String> getRelatedInstances(String _ins, String _rel) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "SELECT DISTINCT ?ins WHERE " +
//		" { " +
		" { " + _ins + " " + _rel + " ?ins . " +
//		" ?ins rdf:type owl:Thing. }" +
//		" UNION { ?ins " + _rel + _ins +  " . " +
//		" ?ins rdf:type owl:Thing. } ." +
		" FILTER (?ins != " + _ins + " ) }";
//		System.out.println(query);
		ResultSet results = (ResultSet) m_mycrowl.execute(query);
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}


	
	public Set<String> getInverseRelatedInstances(String _ins, String _rel) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "SELECT DISTINCT ?ins WHERE { " +
//		" { " + _ins.getFullName() + " " + _rel.getFullRelationName() + " ?ins . " +
		" { ?ins " + _rel + " " + _ins +  " . " +
		" ?ins rdf:type oe:Entity. }" +
//		" UNION { ?ins " + _rel.getFullRelationName() + " " + _ins.getFullName() +  " . " +
//		" ?ins rdf:type oe:Entity. } ." +
		" FILTER (?ins != " + _ins + " ) }";
//		System.out.println(query);
		ResultSet results = (ResultSet) m_mycrowl.execute(query);
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}
	public Set<String> getImmediateRelatedInstances(String _ins, String _rel) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "SELECT ?ins ?y WHERE { " +
		" { " + _ins + " " + _rel + " ?ins . " +
		" ?ins rdf:type oe:Entity. }" +
		" OPTIONAL { ?y " + _rel + " ?ins. " +
		_ins + " " + _rel  + " ?y . " +
		" FILTER(?y != ?ins). } " +
		" FILTER (!BOUND(?y)). " +
		" FILTER (!isblank(?ins)). }";
//		System.out.println(query);
		ResultSet results = (ResultSet) m_mycrowl.execute(query);
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
//			System.out.println("results has next");
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
//				System.out.println(_currAnswerRersource);
				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
//		System.out.println("result set has size " + _returnSet.size());
		return _returnSet;
	}

	public Set<String> getRelations(String _ins1, String _ins2) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "SELECT DISTINCT ?rel WHERE " +
//		" { " +
		" { " + _ins1 + " ?rel " + _ins2+ " . }";
		ResultSet results = (ResultSet) m_mycrowl.execute(query);
		TreeSet<String> _returnSet = new TreeSet<String>();
		if (results==null) return _returnSet;
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?rel");
			try {
				_returnSet.add((_currAnswerRersource.toString()));
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}
	
	public boolean areConceptsEquivalent(String _concept1, String _concept2) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE " +
		" { " + _concept1 + " owl:equivalentClass " +  _concept2 + " } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}


	public boolean conceptExists(String _con) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE { " +
		_con + " rdf:type owl:Class } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}

	public boolean relationExists(String _rel) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE { " +
		_rel + " rdf:type owl:ObjectProperty } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}

	public boolean areInstancesRelated(String _ins1, String _ins2, String _rel) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE { " +
		_ins1 + " "+ _rel + " " + _ins2 + " } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}
 
	public boolean instanceExists(String _ins) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE { " +
		_ins + " rdf:type owl:Thing } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}

	public boolean isInstanceOf(String _instance, String _concept) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE { " +
		_instance  + " rdf:type " + _concept + " } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}


	public boolean isSubConcept(String _concept1, String _concept2) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE { " +
		_concept1 + " rdfs:subClassOf " + _concept2 + " } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}

	public boolean isSuperConcept(String _concept1, String _concept2) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE { " +
		_concept2 + " rdfs:subClassOf " + _concept1 + " } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}

	public boolean isSubRelation(String rel1, String rel2) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "ASK WHERE { " +
		rel1 + " rdfs:subPropertyOf " + rel2 + " } ";
		return ((Boolean) m_mycrowl.execute(query)).booleanValue();
	}
	
	public boolean deleteInstance(String _instance) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "DELETE { " + _instance + " ?x ?y . " +
						" ?a ?b " + _instance + " . }" +
						" WHERE { " + _instance + " ?x ?y . " +
						" ?a ?b " + _instance + " . }";
		m_mycrowl.execute(query);
		return (!instanceExists(_instance));
	}

	public boolean deleteInstanceConceptAssertion(String _instance, String _concept) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String aquery = "DELETE { " + _instance  + " ?x ?y . " +
						" ?a ?b " + _instance + " . }" +
						" WHERE { " + _instance + " ?x ?y . " +
						" ?a ?b " + _instance + " . }";

		String query = "DELETE { " +  
		 				_instance +
		 				" rdf:type " +
		 				_concept + 
		 				" . }";

		m_mycrowl.execute(query);
		return (!instanceExists(_instance));
	}

	public boolean deleteRelation(String _ins1, String _relation, String _ins2) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "DELETE { " + _ins1 + " " + _relation + " " + _ins2 + " . }";
		m_mycrowl.execute(query);
		return (!getRelatedInstances(_ins1, _relation).contains(_ins2));
	}

	public Set<String> getPredecessors(String _ins) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "SELECT DISTINCT ?ins WHERE { " +
		" ?ins ?y " + _ins +  " . " +
		" ?ins rdf:type oe:Entity. ?y rdfs:subPropertyOf oe:oeRelation. " +
		" FILTER (?ins != " + _ins + " ) }";
		ResultSet results = (ResultSet) m_mycrowl.execute(query);
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	public Set<String> getSuccessors(String _ins) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "SELECT DISTINCT ?ins WHERE { " +
		_ins + " ?y ?ins . " +
		" ?ins rdf:type oe:Entity. ?y rdfs:subPropertyOf oe:oeRelation. " +
		" FILTER (?ins != " + _ins + " ) }";
		ResultSet results = (ResultSet) m_mycrowl.execute(query);
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;
	}

	public Set<String> getAllSubConcepts(String _con) {
		if (isABoxInconsistent()) throw new RuntimeException("Inconsistent ABox! ABORT!");
		String query = "SELECT ?con WHERE { " +
					   " ?con rdfs:subClassOf " + _con + ". " +	
					   " FILTER (!isblank(?con)) . FILTER (?con != rdfs:Resource) . "+
					   " FILTER (?con != owl:Thing). FILTER (?con != owl:Nothing) ."+
					   " FILTER (?con != " + _con + ") } ";
		ResultSet results = (ResultSet) m_mycrowl.execute(query);
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?con");
			try {
				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
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
				" rdf:type owl:Nothing } "); 
//				m_ontoMemberFactory.createConcept("owl", ":", "Nothing").getFullName() + " } ");
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet.size()>0;
	}
	
	private TreeSet<String> getABoxInconsistencies() {
		ResultSet results = (ResultSet) m_mycrowl.execute("SELECT ?ins WHERE { " +
				" ?ins " +
				" rdf:type owl:Nothing } "); 
//				m_ontoMemberFactory.createConcept("owl", ":", "Nothing").getFullName() + " } ");
		TreeSet<String> _returnSet = new TreeSet<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?ins");
			try {
				_returnSet.add(m_mycrowl.getOWLOntoModel().shortForm(_currAnswerRersource.toString()));
			} catch (ConfigError e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return _returnSet;		
	}
	
	public Set<String> getNames(String _ins) {
		String query = "SELECT DISTINCT ?name WHERE { " +
			_ins + " oe:name ?name . }";
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

	public Set<Integer> getNumberTags(String _ins) {
		String query = "SELECT DISTINCT ?num WHERE { " +
			_ins + " oe:numberTag ?num . }";
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

	public void addName(String _ins, String name) {
		m_mycrowl.execute("INSERT { " +
				_ins +
				" oe:name \" " + name + " \" .}");
	}
	
	public void addNumberTag(String _ins, Integer _number) {
		m_mycrowl.execute("INSERT { " +
				_ins +
				" oe:numberTag " + _number.toString() + " .}");
	}

	public Set<Object> getPropertyValues(String _ins, String _prop) {
		String query = "SELECT DISTINCT ?val WHERE { " +
		_ins + " " + _prop + " ?val . }";
		ResultSet results = (ResultSet) m_mycrowl.execute(query);

		TreeSet<Object> _returnSet = new TreeSet<Object>();

		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			Resource _currAnswerRersource = _qs.getResource("?val");
			_returnSet.add(_currAnswerRersource);
		}
		return _returnSet;
	}
	
	public void addProperty(String _ins, String _property, Object _value) {
		m_mycrowl.execute("INSERT { " +
				_ins +
				" " +  _property + " " + _value.toString() + " .}");
	}
	
	
	private void log(String _msg) {
//		if (m_logging) System.out.println("[CrowlWrapper: " + _msg + "]");

		m_logger.debug(_msg);
	}
	
}
