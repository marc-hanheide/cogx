package org.cognitivesystems.reasoner.pellet;

// Java API
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;
import java.io.StringWriter;
import java.net.URI;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

// Reasoner API
import org.cognitivesystems.reasoner.*;
import org.cognitivesystems.reasoner.base.ReasonerConcept;
import org.cognitivesystems.reasoner.base.ReasonerInstance;
import org.cognitivesystems.reasoner.base.OntologyMemberFactory;
import org.cognitivesystems.reasoner.base.ReasonerException;
import org.cognitivesystems.reasoner.base.ReasonerInterface;
import org.cognitivesystems.reasoner.base.ReasonerRelation;

// External API: pellet, owlapi, aterm-java-1.6
import org.mindswap.pellet.EdgeList;
import org.mindswap.pellet.Individual;
import org.mindswap.pellet.Literal;
import org.mindswap.pellet.Node;
import org.mindswap.pellet.Role;
import org.mindswap.pellet.exceptions.InternalReasonerException;
import org.mindswap.pellet.output.OutputFormatter;
import org.mindswap.pellet.owlapi.ConceptConverter;
import org.mindswap.pellet.owlapi.Reasoner;
import org.semanticweb.owl.apibinding.OWLManager;
import org.semanticweb.owl.io.StringInputSource;
import org.semanticweb.owl.model.OWLClass;
import org.semanticweb.owl.model.OWLConstant;
import org.semanticweb.owl.model.OWLDataProperty;
import org.semanticweb.owl.model.OWLDescription;
import org.semanticweb.owl.model.OWLIndividual;
import org.semanticweb.owl.model.OWLObjectProperty;
import org.semanticweb.owl.model.OWLObjectPropertyExpression;
import org.semanticweb.owl.model.OWLOntology;
import org.semanticweb.owl.model.OWLOntologyCreationException;
import org.semanticweb.owl.model.OWLOntologyManager;
import org.mindswap.pellet.utils.ATermUtils;

import aterm.ATerm;
import aterm.ATermAppl;
import aterm.ATermList;

/*
 * Created on June 20, 2007
 */

/**
 * This class implements the ReasonerInterface API. 
 * It uses Pellet as an OWL-DL reasoning service.
 * It works on String input and output and it hides the
 * ontology namespace from the user, thus accepting and
 * returning only shot names, e.g.
 * "my_instance" and "My_Concept"
 * rather than
 * "http://www.cognitivesystems.org/ontologies/myOntology.owl#my_instance"
 * and "http://www.cognitivesystems.org/ontologies/myOntology.owl#My_Concept".
 * Internally, however, Pellet operates on fully qualified names.
 * 
 * You can run this class's main method in order to have a small
 * test example program
 * 
 * STATUS: deletion of instances -- incl. their relations with other instances --
 * works. However, previously drawn inferences are still present!
 * TO-DO: find a way to exclude currently invalid classifications...
 * 
 * @author      Hendrik Zender, zender@dfki.de
 * @version     July 13, 2007
 */
public class PelletWrapper implements ReasonerInterface {
	
	// default settings, use binary constructor to override
	private String m_ontologyFile;
//	private String m_ns = "http://www.dfki.de/cosy/officeenv.owl";
//	private String m_sep = "#";
	private OntologyMemberFactory m_ontoMemberFactory;
	
	// flag for logging on/off
	private boolean m_logging = true;
	// which PrintWriter should be logged to
	private PrintStream m_logStream = System.out; 
	
	// Class fields
	private Reasoner m_reasoner;
	private OWLOntology m_OWLontology;
	private OWLOntologyManager m_OWLmanager;

	
	/**
	 * The class constructor that allows you to specify 
	 * the ontology file location and the ontology namespace.
	 * 
	 * @param _ontologyFile  The file location of the ontology
	 * @param _ns  The namespace URI of the ontology
	 * @throws OWLOntologyCreationException
	 */
	public PelletWrapper(String _ontologyFile, String _ns, String _sep) throws ReasonerException {
		m_ontologyFile = _ontologyFile;
		m_ontoMemberFactory = new OntologyMemberFactory(_ns, _sep);
		try {
			this.init();
		} catch (OWLOntologyCreationException e) {
			log("ERROR: Init failed!");
			System.err.println("ERROR: Initialization failed!");
			throw new ReasonerException(e.getMessage()+e.getStackTrace());
		}
	}
	
	/**
	 * The class constructor that allows you to specify 
	 * the ontology file location and the ontology namespace.
	 * Additionally you can set the logging behavior.
	 * 
	 * @param _ontologyFile  The file location of the ontology
	 * @param _ns  The namespace URI of the ontology
	 * @param _logging whether to output log messages
	 * @throws OWLOntologyCreationException
	 */
	public PelletWrapper(String _ontologyFile, String _ns, String _sep, boolean _logging) throws ReasonerException {
		m_ontologyFile = _ontologyFile;
		m_ontoMemberFactory = new OntologyMemberFactory(_ns, _sep);
		this.setLogging(_logging);
		try {
			this.init();
		} catch (OWLOntologyCreationException e) {
			log("ERROR: Init failed!");
			System.err.println("ERROR: Initialization failed!");
			throw new ReasonerException(e.getMessage()+e.getStackTrace());
		}
	}

	/**
	 * This private method initializes the Pellet connection etc.
	 * The method is called automatically from the constructor.
	 *
	 * @throws OWLOntologyCreationException
	 */
	private void init() throws OWLOntologyCreationException {
		log("Creating new OWL Ontology manager.");
		m_OWLmanager = OWLManager.createOWLOntologyManager();

		try {
			m_OWLontology = m_OWLmanager.loadOntology(
					new StringInputSource(readFileAsString(m_ontologyFile)));
			// (URI.create(m_ontologyFile));
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.exit(0);
		}

		log("Creating new reasoner.");
		m_reasoner = new Reasoner(m_OWLmanager);		
		m_reasoner.loadOntology(m_OWLontology);
		
		log("Initialization done.");
	}
	
    /** @param filePath the name of the file to open. Not sure if it can accept URLs or just filenames. Path handling could be better, and buffer sizes are hardcoded
	    */ 
	    private static String readFileAsString(String filePath)
	    throws java.io.IOException{
	        StringBuffer fileData = new StringBuffer(1000);
	        BufferedReader reader = new BufferedReader(
	                new FileReader(filePath));
	        char[] buf = new char[1024];
	        int numRead=0;
	        while((numRead=reader.read(buf)) != -1){
	            fileData.append(buf, 0, numRead);
	        }
	        reader.close();
	        return fileData.toString();
	    }
	
	/**
	 * This private method takes care of generating a fully qualified name
	 * for OWL ontology entities.
	 * 
	 * @param _entity  the short name of an entity
	 * @return the fully qualified name with the namespace and separator as prefix

	private String getFullName(String _entity) {
		//log("getting full name of " + _entity);
		String returnString;
		if ("_TOP_".equals(_entity) || "".equals(_entity)) { 
			returnString =  _entity;
		}
		else {
			returnString =  m_ns + m_sep + _entity;
		}
		//log(returnString);
		return returnString;
	}
	*/

	
	
	/**
	 * The method calls the pellet method for generating a String
	 * representation of the taxonomy tree, i.e. the concept hierarchy of the ontology.
	 * 
	 * @param _printAsHTML
	 * @return a String representation of the concept hierarchy
	 */
	public String getTaxonomyTree(boolean _printAsHTML) {
		log("Generating String of the taxonomy tree");
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();
		StringWriter sw = new StringWriter();
		m_reasoner.getKB().getTaxonomy().print(new OutputFormatter(sw, _printAsHTML));
		try {
			sw.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		return sw.toString();
	}
	
	/*
	public static void taxonomyToGraph(String _taxonomyTree, String graphName) {

		String DOTFile = graphName + ".dot" ;
		String PNGFile = graphName + ".png" ;

//		String DOTText = "digraph G { rnode -> dnode1 ; rnode -> dnode2,}" ;
		String DOTText = createDOTSpecsFromTree(_taxonomyTree) ;

		// bis hier OK

		writeDOTFile(DOTText,DOTFile);

		try	{
			Runtime.getRuntime().exec("dot -Tpng "+DOTFile+" -o "+PNGFile);
		}
		catch (Exception e) {
			e.printStackTrace();
		}

		// showPNGGraph(PNGFile);
	}

	private static String createDOTSpecsFromTree(String _taxonomyTreeString) {
		String text = "digraph G {\n";
		LFNominal[] lfNoms =  lf.noms;
		for (int i = 0; i < lfNoms.length; i++) {
			String nomVarAndSort = "\\{"+lfNoms[i].nomVar+":"+lfNoms[i].sort+"\\}" ;

			String features = "" ;
			for (int j=0;j<lfNoms[i].feats.length;j++) {
				features += lfNoms[i].feats[j].feat+":"+lfNoms[i].feats[j].value  + "\\n" ;
				if (j < lfNoms[i].feats.length-1) 
					features += " & ";
			}

			String proposition = getPropositionString(lfNoms[i].prop) ;

			text += lfNoms[i].nomVar + " [shape=Mrecord fontsize=10 label=\"{" ;

			if (!proposition.equals("")) {
				text += proposition + "|" ;
			}
			text += nomVarAndSort ;

			if (!features.equals(""))
				text += "|"+features ;

			text += "}\"]"+";\n" ;

			String relations = "" ;
			for (int j=0;j<lfNoms[i].rels.length;j++) {
				LFRelation rel = lfNoms[i].rels[j] ;
				text += rel.head + " -> " + rel.dep + "[label=\"" + rel.mode + "\"];\n" ;
				if (rel.coIndexedDep) {
					text += rel.dep + " -> " + rel.head +";\n"; 
				}
			}

		}
		text += "\n}";
		return text ;
	}
*/
	
	/*
	 * deprecated: work only on String input and output!
	
	public Set<OWLIndividual> getInstances(String _concept) {
		log("Getting all instances of concept: " + _concept);
		this.m_reasoner.getKB().classify();
		this.m_reasoner.getKB().realize();
		
		OWLClass _class = m_OWLmanager.getOWLDataFactory().getOWLClass(URI.create(getFullName(_concept)));
		
		return this.m_reasoner.getIndividuals(_class, false);
	}
	*/
	
	/**
	 * This method returns all known instances of a given concept.
	 * 
	 * @param _concept  the short name of the concept
	 * @return a Set<Instance> of all known instances of _concept (short names!)
	 */
	public Set<ReasonerInstance> getInstances(ReasonerConcept _concept) {
		log("Getting all instances of concept: " + _concept.getName());
		this.m_reasoner.getKB().classify();
		this.m_reasoner.getKB().realize();
		
		OWLClass _class = m_OWLmanager.getOWLDataFactory().getOWLClass(URI.create(_concept.getFullName()));
		Set<OWLIndividual> individuals = m_reasoner.getIndividuals(_class, false);

		Set<ReasonerInstance> returnSet = new TreeSet<ReasonerInstance>();
		for (OWLIndividual individual : individuals) {
			returnSet.add(m_ontoMemberFactory.createInstance(individual.getURI().toString()));
		}
		return returnSet;
	}


	/**
	 * This method returns all instances of the ontology.
	 * 
	 * @return a Set<Instance> of all known instances
	 */
	public Set<ReasonerInstance> getAllInstances() {
		log("Getting all instances");
		this.m_reasoner.getKB().classify();
		this.m_reasoner.getKB().realize();
		
		Set<OWLIndividual> individuals = m_reasoner.getIndividuals();

		Set<ReasonerInstance> returnSet = new TreeSet<ReasonerInstance>();
		for (OWLIndividual individual : individuals) {
			returnSet.add(m_ontoMemberFactory.createInstance(individual.getURI().toString()));
		}
		return returnSet;
	}

	
	/**
	 * This method checks whether a given instance instantiates a
	 * given concept.
	 * 
	 * @param _instance the short name of the instance
	 * @param _concept the short name of the concept
	 * @return bool
	 */
	public boolean isInstanceOf(ReasonerInstance _instance, ReasonerConcept _concept) {
		log("is " + _instance.getName() + " an instance of " + _concept.getName());
		this.m_reasoner.getKB().classify();
		this.m_reasoner.getKB().realize();
		
		OWLIndividual _ind = m_OWLmanager.getOWLDataFactory().getOWLIndividual(URI.create(_instance.getFullName()));
		OWLClass _class = m_OWLmanager.getOWLDataFactory().getOWLClass(URI.create(_concept.getFullName()));
		
		return m_reasoner.isInstanceOf(_ind, _class);
	}

	/**
	 * This method checks whether a given instance exists.
	 * 
	 * @param _instance the short name of the instance
	 * @return bool
	 */
	public boolean instanceExists(ReasonerInstance _instance) {
		this.m_reasoner.getKB().classify();
		this.m_reasoner.getKB().realize();
		
		OWLIndividual _ind = m_OWLmanager.getOWLDataFactory().getOWLIndividual(URI.create(_instance.getFullName()));

		return m_reasoner.isDefined(_ind);
	}

	/**
	 * This method checks whether a given concept is a
	 * superconcept of a second given concept.
	 * 
	 * @param _concept1  the short name of the first concept
	 * @param _concept2  the short name of the second concept
	 * @return bool
	 */
	public boolean isSuperConcept(ReasonerConcept _concept1, ReasonerConcept _concept2) {
		log("is " + _concept1.getName() + " a superconcept of " + _concept2.getName());
		this.m_reasoner.getKB().classify();
		this.m_reasoner.getKB().realize();
		
		OWLClass _class1 = m_OWLmanager.getOWLDataFactory().getOWLClass(URI.create(_concept1.getFullName()));
		OWLClass _class2 = m_OWLmanager.getOWLDataFactory().getOWLClass(URI.create(_concept2.getFullName()));
		
		return m_reasoner.isSubClassOf(_class2, _class1);
	}

	/**
	 * This method checks whether a given concept is a
	 * subconcept of a second given concept.
	 * 
	 * @param _concept1  the short name of the first concept
	 * @param _concept2  the short name of the second concept
	 * @return bool
	 */
	public boolean isSubConcept(ReasonerConcept _concept1, ReasonerConcept _concept2) {
		log("is " + _concept1.getName() + " a subconcept of " + _concept2.getName());
		this.m_reasoner.getKB().classify();
		this.m_reasoner.getKB().realize();
		
		OWLClass _class1 = m_OWLmanager.getOWLDataFactory().getOWLClass(URI.create(_concept1.getFullName()));
		OWLClass _class2 = m_OWLmanager.getOWLDataFactory().getOWLClass(URI.create(_concept2.getFullName()));
		
		return m_reasoner.isSubClassOf(_class1, _class2);
	}

	/**
	 * This method checks whether two given concepts are equivalent. 
	 * 
	 * @param _concept1  the short name of the first concept
	 * @param _concept2  the short name of the second concept
	 * @return bool
	 */
	public boolean areConceptsEquivalent(ReasonerConcept _concept1, ReasonerConcept _concept2) {
		log("are " + _concept1.getName() + " and " + _concept2.getName() + " equivalent concepts?");
		this.m_reasoner.getKB().classify();
		this.m_reasoner.getKB().realize();
		
		OWLClass _class1 = m_OWLmanager.getOWLDataFactory().getOWLClass(URI.create(_concept1.getFullName()));
		OWLClass _class2 = m_OWLmanager.getOWLDataFactory().getOWLClass(URI.create(_concept2.getFullName()));
		
		return m_reasoner.isEquivalentClass(_class1, _class2);
	}
	
	/**
	 * This method checks whether a given concept is a
	 * basic level concept.
	 * 
	 * @param _concept  the short name of the concept
	 * @return bool
	 */
	public boolean isBasicLevelConcept(ReasonerConcept _concept) {
		log("is " + _concept.getName() + " a basic level concept?");
		this.m_reasoner.getKB().classify();
		this.m_reasoner.getKB().realize();
		
		Set<ReasonerConcept> directSuperCons = getDirectSuperConcepts(_concept);
		return directSuperCons.contains(m_ontoMemberFactory.createConcept("BasicLevelCat"));
	}


	/**
	 *  This method checks whether a given concept exists in the ontology.
	 *  
	 *  @param the Concept to check
	 *  @return bool whether the concept exists (true) or not (false)
	 */
	public boolean conceptExists(ReasonerConcept _con) {
		this.m_reasoner.getKB().classify();
		this.m_reasoner.getKB().realize();
		
		OWLClass _class = m_OWLmanager.getOWLDataFactory().getOWLClass(URI.create(_con.getFullName()));
		
		boolean _return = m_reasoner.isDefined(_class);
		log("does " + _con.getName() + " exist? The answer is: "+_return);
				
		return _return;
	}

	
	/**
	 * Adds an instance of a given concept to the ontology.
	 * 
	 * @param _instance  The instance to be created (short name)
	 * @param _concept  The concept (short name) to which the instance belongs
	 */
	public void addInstance(ReasonerInstance _instance, ReasonerConcept _concept) {
		// I do not know whether this really is the correct way to do it!!!
		
		ATermAppl classTerm = ATermUtils.makeTermAppl(_concept.getFullName());
		ATermAppl individualTerm = ATermUtils.makeTermAppl(_instance.getFullName());
		m_reasoner.getKB().addIndividual(individualTerm);
		m_reasoner.getKB().addType(individualTerm, classTerm);
		log("Added new instance " + individualTerm.getName() + " of concept " + classTerm.getName());
	}

	/**
	 * This method is supposed to assert that an instance
	 * belongs NOT to a given class.
	 * But is does not do that --- right now...
	 * So, handle with care! THIS DOES NOT WORK!!!
	 * 
	 * @param _instance  The instance to be created (short name)
	 * @param _concept  The concept (short name) to which the instance DOES NOT belong
	 */
	public void negateInstance(ReasonerInstance _instance, ReasonerConcept _concept) {
		// I do not know whether this really is the correct way to do it!!!
		
		ATermAppl classTerm = ATermUtils.makeTermAppl(_concept.getFullName());
		classTerm = ATermUtils.negate(classTerm);
		ATermList assertions = ATermUtils.makeList(classTerm);
		
		for (ReasonerConcept con : this.getAllConcepts(_instance)) {
			assertions = assertions.insert(ATermUtils.makeTermAppl(con.getFullName()));
		}		
		
		ATermAppl andTerm = ATermUtils.makeAnd(assertions);
		ATermAppl individualTerm = ATermUtils.makeTermAppl(_instance.getFullName());
		m_reasoner.getKB().addIndividual(individualTerm);
		m_reasoner.getKB().addType(individualTerm, andTerm);
		log("Added new instance " + individualTerm.getName() + " of concept " + andTerm.getName());
	}
	
	
	public boolean deleteRelation(ReasonerRelation _relation) {
		log("deleting a relation not possible in Pellet!");
		return false;
	}

	/**
	 * Deletes a given instance from the ontology, including all relations it has!
	 * 
	 * @param _instance  The instance to be deleted (short name)
	 */
	public boolean deleteInstance(ReasonerInstance _instance) {
		// I do not know whether this really is the correct way to do it!!!
		ATermAppl individualTerm = ATermUtils.makeTermAppl(_instance.getFullName());

		Set<ReasonerInstance> relIns = getRelatedInstances(_instance);
		
		log("going to query getProperties");
		for (ReasonerInstance relIn : relIns) {
			ATermAppl relInTerm= ATermUtils.makeTermAppl(relIn.getFullName());
			List<ATermAppl> props =  m_reasoner.getKB().getProperties(individualTerm, relInTerm);
			
			for (ATermAppl prop : props) {
				log("deleting property "+prop+"("+individualTerm+","+relInTerm+")");
				m_reasoner.getKB().removePropertyValue(prop, individualTerm, relInTerm);
			}
			
			// and now the inverse properties
			props =  m_reasoner.getKB().getProperties(relInTerm, individualTerm);
			
			for (ATermAppl prop : props) {
				log("deleting property "+prop+"("+relInTerm+","+individualTerm+")");
				m_reasoner.getKB().removePropertyValue(prop, relInTerm, individualTerm);
			}
		}
		//ATermAppl ind1 = ATermUtils.makeTermAppl(getFullName(_rel.getArg1().getName()));
		//ATermAppl ind2 = ATermUtils.makeTermAppl(getFullName(_rel.getArg2().getName()));
		//ATermAppl role = ATermUtils.makeTermAppl(getFullName(_rel.getRelationName())); 
		//m_reasoner.getKB().addPropertyValue(role, ind1, ind2);
		
		//m_reasoner.getKB().addR
		m_reasoner.getKB().removeIndividual(individualTerm);
		log("Deleted instance " + individualTerm.getName() + ".");
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();
		System.err.println("Pellet does not support proper deletion!");
		return false;
	}	
	
	/**
	 * Asserts a relation between two instances.
	 * e.g. relation(instance1, instance2)
	 * 
	 * @param _rel   the given relation
	 */
	public void assertRelation(ReasonerRelation _rel) {
		// First attempt to do this...!!!
		
		ATermAppl ind1 = ATermUtils.makeTermAppl(_rel.getArg1().getFullName());
		ATermAppl ind2 = ATermUtils.makeTermAppl(_rel.getArg2().getFullName());
		ATermAppl role = ATermUtils.makeTermAppl(_rel.getFullRelationName()); 

		m_reasoner.getKB().addPropertyValue(role, ind1, ind2);
		
		//m_reasoner.getKB().classify();
		//m_reasoner.getKB().realize();
			
		log("Added new relation " + role.getName() + "(" + ind1.getName() + "," + ind2.getName() + ")");
	}
	
	/**
	 * Returns all subconcepts of a given concept.
	 * 
	 * @param _concept The conept's short name
	 * @return  an unorderd Set<Concept> of all the concept's subconcepts
	 */
	public Set<ReasonerConcept> getAllSubConcepts(ReasonerConcept _concept) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		ATermAppl concept = ATermUtils.makeTermAppl(_concept.getFullName());
		Set<ReasonerConcept> returnSet = new TreeSet<ReasonerConcept>();
		
		for (Set<ATermAppl> typeSet : m_reasoner.getKB().getSubClasses(concept,false)) {
			for (ATermAppl type : typeSet) {
				returnSet.add(m_ontoMemberFactory.createConcept(type.getName()));
			}
		}
		return returnSet;
	}

	/**
	 * Returns the direct subconcepts of a given concept.
	 * 
	 * @param _concept The conept's short name
	 * @return  an unorderd Set<Concept> of the concept's direct subconcepts
	 */
	public Set<ReasonerConcept> getDirectSubConcepts(ReasonerConcept _concept) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		ATermAppl concept = ATermUtils.makeTermAppl(_concept.getFullName());
		Set<ReasonerConcept> returnSet = new TreeSet<ReasonerConcept>();
		
		for (Set<ATermAppl> typeSet : m_reasoner.getKB().getSubClasses(concept,true)) {
			for (ATermAppl type : typeSet) {
				returnSet.add(m_ontoMemberFactory.createConcept(type.getName()));
			}
		}
		return returnSet;
	}

	/**
	 * Returns all superconcepts of a given concept.
	 * 
	 * @param _concept The conept's short name
	 * @return  an unorderd Set<Concept> of all the concept's superconcepts
	 */
	public Set<ReasonerConcept> getAllSuperConcepts(ReasonerConcept _concept) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		ATermAppl concept = ATermUtils.makeTermAppl(_concept.getFullName());
		Set<ReasonerConcept> returnSet = new TreeSet<ReasonerConcept>();
		
		for (Set<ATermAppl> typeSet : m_reasoner.getKB().getSuperClasses(concept,false)) {
			for (ATermAppl type : typeSet) {
				returnSet.add(m_ontoMemberFactory.createConcept(type.getName()));
			}
		}
		return returnSet;
	}

	/**
	 * Returns the direct superconcepts of a given concept.
	 * 
	 * @param _concept The conept's short name
	 * @return  an unorderd Set<Concept> of the concept's direct superconcepts
	 */
	public Set<ReasonerConcept> getDirectSuperConcepts(ReasonerConcept _concept) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		ATermAppl concept = ATermUtils.makeTermAppl(_concept.getFullName());
		Set<ReasonerConcept> returnSet = new TreeSet<ReasonerConcept>();
		
		for (Set<ATermAppl> typeSet : m_reasoner.getKB().getSuperClasses(concept,true)) {
			for (ATermAppl type : typeSet) {
				returnSet.add(m_ontoMemberFactory.createConcept(type.getName()));
			}
		}
		return returnSet;
	}

	
	
	/**
	 * Returns all relations of a given instance.
	 * 
	 * @param _instance  The instance's short name
	 * @return a Set<Relation> of the relations
	 */
	public Set<ReasonerRelation> getRelations(ReasonerInstance _instance) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		ATermAppl individual = ATermUtils.makeTermAppl(_instance.getFullName());
		Set<ReasonerRelation> returnSet = new TreeSet<ReasonerRelation>();
		
		for (ReasonerInstance relInstance : this.getRelatedInstances(_instance)) {
			ATermAppl relIndividual  = ATermUtils.makeTermAppl(relInstance.getFullName());
			List<ATermAppl> _aTermList = m_reasoner.getKB().getProperties(individual, relIndividual);
			for (ATermAppl appl : _aTermList) {
				returnSet.add(m_ontoMemberFactory.createRelation(appl.getName(), _instance, relInstance));
			}
		}			
		return returnSet;
	}

	/**
	 * Returns all instances that are first args of a given relation.
	 * 
	 * @param _relation  The given relation
	 * @return a Set<Instance> of the instances
	 */
	public void getRoleFillers(ReasonerRelation _relation) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		
		OWLObjectProperty property = m_reasoner.getManager().getOWLDataFactory().getOWLObjectProperty(URI.create(_relation.getFullRelationName()));
		Set test = m_reasoner.getRanges(property);
		
		for (Object object : test) {
			System.out.println(object);
		}
				
		/*for (Instance relInstance : this.getRelatedInstances(_instance)) {
			ATermAppl relIndividual  = ATermUtils.makeTermAppl(getFullName(relInstance.getName()));
			List<ATermAppl> _aTermList = m_reasoner.getKB().getProperties(individual, relIndividual);
			for (ATermAppl appl : _aTermList) {
				returnSet.add(new SimpleRelation(getShortName(appl.getName()), _instance, relInstance));
			}
		}
		return returnSet;
		*/
	}

	
	/**
	 * TODO this method does not provide any senseful data...
	 * don't use it!
	 * Returns all possible relations of a given instance.
	 * 
	 * @param _instance The instance's short name
	 */
	public void getPossibleProperties(ReasonerInstance _instance) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		ATermAppl instance = ATermUtils.makeTermAppl(_instance.getFullName());
		Set<ReasonerConcept> returnSet = new TreeSet<ReasonerConcept>();
		
		//m_reasoner.getKB().getPossibleProperties(instance);
		
		
		for (Role role : m_reasoner.getKB().getPossibleProperties(instance)) {
			System.out.println("debug string:");
			System.out.println(role.debugString());
			System.out.println("get type");
			System.out.println(role.getType());
			System.out.println("get type name");
			System.out.println(role.getTypeName());
			System.out.println("to string");
			System.out.println(role.toString());
		}
		// return returnSet;
	}
	
	
	/**
	 * Returns all concepts a given instance instatiates.
	 * 
	 * @param _instance  The instance's short name
	 * @return an unordered Set<Concept> of all the instance's concepts
	 */
	public Set<ReasonerConcept> getAllConcepts(ReasonerInstance _instance) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		ATermAppl individual = ATermUtils.makeTermAppl(_instance.getFullName());
		Set<ReasonerConcept> returnSet = new TreeSet<ReasonerConcept>();

		for (Set<ATermAppl> typeSet : m_reasoner.getKB().getTypes(individual)) {
			for (ATermAppl type : typeSet) {
				log (type.getName());
				if (!type.getName().equals("_TOP_")) returnSet.add(m_ontoMemberFactory.createConcept(type.getName()));
			}
		}
		return returnSet;
	}
	
	
	/**
	 * Returns all direct concepts of a given instance.
	 * 
	 * @param _instance  The instance's short name
	 * @return  an unorderd Set<Concept> of all the instance's direct concepts
	 */
	public Set<ReasonerConcept> getDirectConcepts(ReasonerInstance _instance) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		ATermAppl individual = ATermUtils.makeTermAppl(_instance.getFullName());
		Set<ReasonerConcept> returnSet = new TreeSet<ReasonerConcept>();

		for (Set<ATermAppl> typeSet : m_reasoner.getKB().getTypes(individual,true)) {
			for (ATermAppl type : typeSet) {
				returnSet.add(m_ontoMemberFactory.createConcept(type.getName()));
			}
		}
		return returnSet;
	}
	
	
	/**
	 * Returns the most specific concepts a given instance instantiates.
	 * 
	 * @param _instance  The instance's short name
	 * @return  an unordered Set<Concept> of all the instance's most specific concepts
	 */
	public Set<ReasonerConcept> getMostSpecificConcepts(ReasonerInstance _instance) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		OWLIndividual individual = m_OWLmanager.getOWLDataFactory().getOWLIndividual(URI.create(_instance.getFullName()));
		Set<ReasonerConcept> returnSet = new TreeSet<ReasonerConcept>();
		
		// get all equivalence sets of the direct types
		Set<Set<OWLClass>> setOfEquiSets =  m_reasoner.getTypes(individual,true);
		
		// get a set of all direct types
		Set<String> allDirectTypesHashSet = new HashSet<String>();
		for (Set<OWLClass> memberSetOfSetOfEquiSets : setOfEquiSets) {
			for (OWLClass _class_: memberSetOfSetOfEquiSets) {
				allDirectTypesHashSet.add(_class_.getURI().toString());
			}
		}
		
		// unsupported in Pellet 1.5.1
		// create a concept converter that transforms ATerms to OWLDescriptions
		// ConceptConverter konverter = new ConceptConverter(m_OWLontology, m_OWLmanager.getOWLDataFactory());
		
		// create a target container for the most specific concepts
		Set<String> mostSpecificConceptSet = new HashSet<String>();
		// assume that all direct types are most specific types
		mostSpecificConceptSet.addAll(allDirectTypesHashSet);
		
		
		// determine which equivalence sets are in a taxonomical relation
		for (Set<OWLClass> eqSet1 : setOfEquiSets) {
			// make the equivalence set of tree set
			TreeSet<String> equiTreeSet1 = new TreeSet<String>();
			for (OWLClass _class_ : eqSet1) {
				equiTreeSet1.add(_class_.getURI().toString());
			}
			
			for (Set<OWLClass> eqSet2: setOfEquiSets) {
				TreeSet<String> equiTreeSet2 = new TreeSet<String>();
				for (OWLClass _class2_ : eqSet2) {
					equiTreeSet2.add(_class2_.getURI().toString());
				}
				
				// unsupported in Pellet 1.5.1
				// OWLDescription owlClass1 = (OWLDescription) converter.convert(ATermUtils.makeTermAppl(equiTreeSet1.first()));
				// OWLDescription owlClass2 = (OWLDescription) converter.convert(ATermUtils.makeTermAppl(equiTreeSet2.first()));

				OWLClass owlClass1 = m_OWLmanager.getOWLDataFactory().getOWLClass(URI.create(equiTreeSet1.first()));
				OWLClass owlClass2 = m_OWLmanager.getOWLDataFactory().getOWLClass(URI.create(equiTreeSet2.first()));
				
				boolean is1SubClassof2 = m_reasoner.isSubClassOf(owlClass1, owlClass2);
				boolean is2SubClassof1 = m_reasoner.isSubClassOf(owlClass2, owlClass1);
				if (is1SubClassof2 && !is2SubClassof1) {
					for (String owlclass : equiTreeSet2) {
						mostSpecificConceptSet.remove(owlclass);
					}
				}
				//}
			}
		}
		
		for (String specificClass : mostSpecificConceptSet) {
			returnSet.add(m_ontoMemberFactory.createConcept(specificClass));
		}
		return returnSet;
	}
	
	
	/**
	 * Returns all basic level concepts of a given instance.
	 * 
	 * @param _instance  The instance's short name
	 * @return  all basic level Concepts of the instance
	 */
	public Set<ReasonerConcept> getBasicLevelConcepts(ReasonerInstance _instance) {
		Set<ReasonerConcept> _returnSet = new TreeSet<ReasonerConcept>();
		// check if the given instance has a BasicLevelCat concept at all
		if (getAllConcepts(_instance).contains(m_ontoMemberFactory.createConcept("BasicLevelCat"))) {

			Set<ReasonerConcept> allCons = getAllConcepts(_instance);
			for (ReasonerConcept con : allCons) {
				// if we find a basic level concept, we're done
				if (isBasicLevelConcept(con)) _returnSet.add(con);
			}

			// OLD CODE:
			// tries to find the most specific basic level con...
			// now go through the rest of the concepts
//			Queue<Concept> conQ = new LinkedList<Concept>();
//			conQ.addAll(mostSpecCon);
//
//			while(!conQ.isEmpty()) {
//				Concept _currCon = conQ.poll();
//				if (isBasicLevelConcept(_currCon)) return _currCon;
//				else conQ.addAll(getDirectSuperConcepts(_currCon));
//			}
			
		} // if not, we don't have a result -> return null!
		return _returnSet;
	}
	
	
	/**
	 * Returns all predecessors of a given instance. I.e. this method
	 * returns a Set<Instance> of all instances in the ontology that are
	 * first arguments to relations that have the given instance as second argument.
	 *
	 * @param _instance  The instance's short name
	 * @return a Set<Instance> of the predecessor instances
	 */
	public Set<ReasonerInstance> getPredecessors(ReasonerInstance _instance) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		Node instanceNode = m_reasoner.getKB().getABox().getNode(ATermUtils.makeTermAppl(_instance.getFullName()));
		Set<ReasonerInstance> returnSet = new HashSet<ReasonerInstance>();
		for (Object currNode : instanceNode.getPredecessors()) {
			returnSet.add(m_ontoMemberFactory.createInstance(((Node) currNode).toString()));
		}
		return returnSet;
	}


	/**
	 * Returns all successors of a given instance. I.e. this method
	 * returns a Set<Instance> of all instances in the ontology that are
	 * second arguments to relations that have the given instance as first argument.
	 * 
	 * @param _instance  The instance's short name
	 * @return  a Set<Instance> of the successor instances
	 */
	public Set<ReasonerInstance> getSuccessors(ReasonerInstance _instance) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		Node instanceNode = m_reasoner.getKB().getABox().getNode(ATermUtils.makeTermAppl(_instance.getFullName()));
		Set<ReasonerInstance> returnSet = new HashSet<ReasonerInstance>();
		for (Object currNode : ((Individual) instanceNode).getSuccessors()) {
			returnSet.add(m_ontoMemberFactory.createInstance(((Node) currNode).toString()));
		}
		return returnSet;
	}
	

	/**
	 * Returns all related instances of a given instance. I.e. this method
	 * returns a Set<Instance> that contains all predecessors and all successors
	 * of the given instance.
	 * 
	 * @param _instance  The instance's short name
	 * @return a Set<Instance> of all related instances
	 */
	public Set<ReasonerInstance> getRelatedInstances(ReasonerInstance _instance) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		Set<ReasonerInstance> returnSet = new HashSet<ReasonerInstance> ();
		returnSet.addAll(this.getSuccessors(_instance));
		returnSet.addAll(this.getPredecessors(_instance));
		return returnSet;
	}

	/**
	 * Returns all instances that are related to a given instance by a given relation. 
	 * 
	 * @param _instance  The instance
	 * @param _relation  The relation (only the relation label is used! Instance info is ignored)
	 * @return a Set<Instance> of all related instances
	 */
	public Set<ReasonerInstance> getRelatedInstances(ReasonerInstance _instance, ReasonerRelation _relation) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		OWLIndividual individual = m_OWLmanager.getOWLDataFactory().getOWLIndividual(URI.create(_instance.getFullName()));
		OWLObjectPropertyExpression property = m_OWLmanager.getOWLDataFactory().getOWLObjectProperty(URI.create(_relation.getFullRelationName()));
		TreeSet<ReasonerInstance> returnSet = new TreeSet<ReasonerInstance>();
		for (OWLIndividual _ind : m_reasoner.getRelatedIndividuals(individual,property)) {
			returnSet.add(m_ontoMemberFactory.createInstance(_ind.toString()));
		}
		return returnSet;
	}
	
	public Set<ReasonerInstance> getImmediateRelatedInstances(ReasonerInstance _instance, ReasonerRelation _relation) {
		throw new RuntimeException("getImmediateRelatedInstances not supported by Pellet!");
	}
	
	public Set<ReasonerInstance> getInverseRelatedInstances(ReasonerInstance _instance, ReasonerRelation _relation) {
		throw new RuntimeException("getInverseRelatedInstances not supported by Pellet!");
	}

	/**
	 * Returns all instances that are related to a given instance by a given relation. 
	 * 
	 * @param _instance  The instance
	 * @param _relation  The short name of the relation
	 * @return a Set<Instance> of all related instances
	 */
	public Set<ReasonerInstance> getRelatedInstances(ReasonerInstance _instance, String _relation) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		OWLIndividual individual = m_OWLmanager.getOWLDataFactory().getOWLIndividual(URI.create(_instance.getFullName()));
		OWLObjectPropertyExpression property = m_OWLmanager.getOWLDataFactory().getOWLObjectProperty(URI.create(_relation));
		TreeSet<ReasonerInstance> returnSet = new TreeSet<ReasonerInstance>();
		for (OWLIndividual _ind : m_reasoner.getRelatedIndividuals(individual,property)) {
			returnSet.add(m_ontoMemberFactory.createInstance(_ind.toString()));
		}
		return returnSet;
	}
	
	public Set<String> getNames(ReasonerInstance _instance) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();

		TreeSet<String> _returnSet = new TreeSet<String>();
		
		OWLIndividual individual = m_OWLmanager.getOWLDataFactory().getOWLIndividual(URI.create(_instance.getFullName()));
		OWLDataProperty property = m_OWLmanager.getOWLDataFactory().getOWLDataProperty(URI.create("http://www.dfki.de/cosy/officeenv.owl#name"));
		OWLConstant owlName = m_reasoner.getRelatedValue(individual,property);
		if (owlName!=null) _returnSet.add(owlName.getLiteral());

		return _returnSet;
	}

	/**
	 * Returns all related instances of a given instance. I.e. this method
	 * returns a Set<Instance> that contains all predecessors and all successors
	 * of the given instance.
	 * 
	 * @param _instance  The instance's short name
	 * @return a Set<Instance> of all related instances
	 */
/*	public Set<Relation> getRelations(Instance _instance) {
		m_reasoner.getKB().realize();
		m_reasoner.getKB().classify();
		
		Node instanceNode = m_reasoner.getKB().getABox().getNode(ATermUtils.makeTermAppl(getFullName(_instance.getName())));
		m_reasoner.

		Set<Instance> returnSet = new HashSet<Instance> ();
		returnSet.addAll(this.getSuccessors(_instance));
		returnSet.addAll(this.getPredecessors(_instance));
		return returnSet;
	}
*/
	
	/**
	 * A helper method, primarily for debugging purposes.
	 * It prints the current state of the ABox to std out.
	 */
	public void printABox() {
		log("printABox()");
		Iterator n = m_reasoner.getKB().getABox().getNodes().iterator();
		while( n.hasNext() ) {
			Node node = (Node) n.next();
			if( !node.isRoot() || node instanceof Literal )
				continue;
			printNode( (Individual) node, new HashSet<Individual>(), "   " );
		}
	}
	
	/*
	public Set getAllRelatedInstances(String _instance) {
		return 
			m_reasoner.getKB().getABox().getNode(ATermUtils.makeTermAppl(getFullName(_instance))).getPredecessors().
			addAll(m_reasoner.getKB().getABox().getNode(ATermUtils.makeTermAppl(getFullName(_instance))).get
					)
	}
	*/ 
	
	/**
	 * HELPER METHOD TAKEN FROM org.mindswap.pellet.ABox
	 * Copyright (c) 2003 Ron Alford, Mike Grove, Bijan Parsia, Evren Sirin
	 * under the MIT License
	 * 
	 * Print the node in the completion tree.
	 * 
	 * @param node
	 * @param printed
	 * @param indent
	 */
	private void printNode(Individual node, Set<Individual> printed, String indent) {
		boolean printOnlyName = (node.isNominal() && !printed.isEmpty());

		if( printed.contains( node ) ) {
			System.out.println( " " + node.getNameStr() );
			return;
		}
		else
			printed.add( node );

		if( node.isMerged() ) {
			System.out.println( node.getNameStr() + " -> " + node.getSame().getNameStr() + " "
					+ node.getMergeDependency( true ) );
			return;
		}
		else if( node.isPruned() )
			throw new InternalReasonerException( "Pruned node: " + node );

		System.out.println("NODE GET NAME STR: " + node.getNameStr() + " --- NODE GET PREDECESSORS: " + node.getPredecessors() + 
				" --- NODE GET SUCCESSORS: " + node.getSuccessors());
		//System.out.println( node.debugString() + " " + node.getDifferents() );

		if( printOnlyName )
			return;

		indent += "  ";
		Iterator i = node.getSuccessors().iterator();
		while( i.hasNext() ) {
			Node succ = (Node) i.next();
			EdgeList edges = node.getEdgesTo( succ );

			System.out.print( indent + "[" );
			for( int e = 0; e < edges.size(); e++ ) {
				if( e > 0 )
					System.out.print( ", " );
				System.out.print( edges.edgeAt( e ).getRole() );
			}
			System.out.print( "] " );
			if( succ instanceof Individual )
				printNode( (Individual) succ, printed, indent );
			else
				System.out.println( " (Literal) " + succ.getName() + " " + succ.getTypes() );
		}
	}

	
	/**
	 * A helper method, primarily for debugging purposes.
	 * It prints the current state of the TBox to std out.
	 */
	public void printTBox() {
		Set<ATermAppl> allClasses = m_reasoner.getKB().getTBox().getAllClasses();
		for (ATermAppl appl : allClasses) {
			System.out.println(appl);
		}
	}

	/**
	 * A helper method, primarily for debugging purposes.
	 * It prints the current state of the RBox to std out.
	 */
	public void printRBox() {
		m_reasoner.getKB().getRBox().getTaxonomy().print();
	}


	/**
	 * This method sets the details for logging:
	 * on / off
	 * the PrintStream to log to
	 * 
	 * @param _logging    true for on, false for off
	 * @param _outstream  where to log to
	 */
	public void setLogging(boolean _logging, PrintStream _outstream) {
		m_logging = _logging;
		m_logStream = _outstream;
	}
	
	public void setLogging(boolean _logging) {
		m_logging = _logging;
	}
	
	/**
     * Private helper method for logging a String message.
     * 
     * @param _text  the text to log
     */
    private void log(String _text) {
    	if (m_logging) m_logStream.println("[PelletWrapper] " + _text);
    }

	public void addName(ReasonerInstance _ins, String name) {
		// TODO Auto-generated method stub
		
	}

	public void addProperty(ReasonerInstance _ins, String _property, Object _value) {
		// TODO Auto-generated method stub
		
	}

	public Set<Object> getPropertyValues(ReasonerInstance _ins, String _prop) {
		// TODO Auto-generated method stub
		return null;
	}

	public Set<ReasonerInstance> getInstancesByName(String _givenName) {
		// TODO Auto-generated method stub
		return null;
	}

	public boolean areInstancesRelated(ReasonerInstance _ins1, ReasonerInstance _ins2, ReasonerRelation _rel) {
		// TODO Auto-generated method stub
		return false;
	}

	public Set<Integer> getNumberTags(ReasonerInstance _ins) {
		// TODO Auto-generated method stub
		return null;
	}

	public Set<ReasonerInstance> getInstancesByNumberTag(Integer _number) {
		// TODO Auto-generated method stub
		return null;
	}

	public void addNumberTag(ReasonerInstance _ins, Integer _number) {
		// TODO Auto-generated method stub
		
	}

	public boolean relationExists(ReasonerRelation _rel) {
		// TODO Auto-generated method stub
		return false;
	}

	public Set<ReasonerInstance> getInstances(Set<ReasonerConcept> _cons) {
		// TODO Auto-generated method stub
		return null;
	}


}