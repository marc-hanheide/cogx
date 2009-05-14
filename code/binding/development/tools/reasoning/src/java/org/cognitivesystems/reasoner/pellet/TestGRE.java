package org.cognitivesystems.reasoner.pellet;

import java.util.LinkedList;
import java.util.TreeSet;

import org.cognitivesystems.reasoner.base.ReasonerConcept;
import org.cognitivesystems.reasoner.base.ReasonerInstance;
import org.cognitivesystems.reasoner.base.ReasonerConcept;
import org.cognitivesystems.reasoner.base.ReasonerInstance;
import org.cognitivesystems.reasoner.base.OntologyMemberFactory;
import org.cognitivesystems.reasoner.base.ReasonerException;
import org.cognitivesystems.reasoner.base.ReasonerRelation;

public class TestGRE {
	
	public PelletWrapper myPelletWrapper;

	public TestGRE() {
		System.out.println("GRE examples");
		try {
			String _ontologyFile = "/project/cl/cosy/svn.cosy/code/subarchitectures/coma.sa/ontologies/officeenv_wn_ins.owl";
			String _ns = "http://www.dfki.de/cosy/officeenv.owl";
			myPelletWrapper = new PelletWrapper(_ontologyFile, _ns, "#");
		}
		catch (ReasonerException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	private enum Attribute { TYPE, OWNERSHIP, NAME, LOCATION };
	
	public LinkedList<String> makeRefEx(ReasonerInstance _intdRef, TreeSet<ReasonerInstance> _contrastSet, LinkedList<Attribute> _prefAtt) {
		LinkedList<String> refEx = new LinkedList<String>();
		
		//sanity:
		if (_contrastSet.contains(_intdRef)) {
			_contrastSet.remove(_intdRef);
		}
		
		// get the properties of the intended referent:
		TreeSet<ReasonerRelation> _intdRefRels = (TreeSet<ReasonerRelation>) myPelletWrapper.getRelations(_intdRef);
		
		TreeSet<ReasonerInstance> ruledOut;
		
		for (Attribute currAtt : _prefAtt) {
			if (_contrastSet.isEmpty()) break;
			System.out.println("checking current attriubute: "+currAtt);
			
			switch (currAtt) {
			case TYPE:
				ReasonerConcept vCon = findBestValueCon(_intdRef, (ReasonerConcept) ((TreeSet<ReasonerConcept>) myPelletWrapper.getBasicLevelConcepts(_intdRef)).first());
				//System.out.println("rulesOut("+V.getName() + "," + _contrastSet+") =>");
				//System.out.println(rulesOutCon(V,_contrastSet));
				ruledOut = rulesOutCon(vCon,_contrastSet);
				if (ruledOut!=null && (!ruledOut.isEmpty())) {
					refEx.add("TYPE="+vCon.getName());
					_contrastSet.removeAll(ruledOut);
					refEx.add("TYPE GENERATED!");
				}
				break;
			case OWNERSHIP:
				TreeSet<ReasonerInstance> vOwners = findValuesOwner(_intdRef, _intdRefRels);
				
				for (ReasonerInstance v : vOwners) {
					ruledOut = rulesOutOwner(v,_contrastSet);
					if (ruledOut!=null && (!ruledOut.isEmpty())) {
						refEx.add("OWNER="+v.getName());
						_contrastSet.removeAll(ruledOut);
						break;
					}
				}

				break;
			case NAME:
				TreeSet<ReasonerInstance> vNames = findValuesName(_intdRef, _intdRefRels);
				
				for (ReasonerInstance v : vNames) {
					ruledOut = rulesOutName(v,_contrastSet);
					if (ruledOut!=null && (!ruledOut.isEmpty())) {
						refEx.add("NAME="+v.getName());
						_contrastSet.removeAll(ruledOut);
						break;
					}
				}
				
				
				break;
			case LOCATION:
				
				break;

			default:
				break;
			}
		}
		if (_contrastSet.isEmpty()) {
			if (refEx.contains("TYPE GENERATED!")) {
				refEx.remove("TYPE GENERATED!");
				return refEx;
			}
			else {
				refEx.add("TYPE="+((TreeSet<ReasonerConcept>)myPelletWrapper.getBasicLevelConcepts(_intdRef)).first().getName()); 
				return refEx;
			}
		}
		refEx.add("NO REFEX GENERATED, CONTRAST SET NON EMPTY: ");
		refEx.add(_contrastSet.toString());
		return refEx;
	}
	
	
	public ReasonerConcept findBestValueCon(ReasonerInstance _intdRef, ReasonerConcept _inititalValue) {
		System.out.println("called findBestValueCon: "+_intdRef.getName()+" "+_inititalValue.getName());
		ReasonerConcept retVal;
		if (userKnows(_intdRef.getName(), _inititalValue.getName())) {
			retVal = _inititalValue;
		} else retVal = null;
		
		// rest omitted!
		
		return retVal;
	}

	public TreeSet<ReasonerInstance> findValuesName(ReasonerInstance _intdRef, TreeSet<ReasonerRelation> _relations) {
		System.out.println("called findValuesName");
		TreeSet<ReasonerInstance> retVals = new TreeSet<ReasonerInstance>();
		System.out.println(_relations);
		
		for (ReasonerRelation relation : _relations) {
			System.out.println(relation.getRelationName());			
			if (relation.getRelationName().equals("hasName")) {
				ReasonerInstance potentialName = relation.getArg2();
				if (userKnows(_intdRef.getName(), potentialName.getName())) {
					retVals.add(potentialName);
				} 
			}
		}
		return retVals;
	}
	
	public TreeSet<ReasonerInstance> findValuesOwner(ReasonerInstance _intdRef, TreeSet<ReasonerRelation> _relations) {
		System.out.println("called findValuesOwner");
		TreeSet<ReasonerInstance> retVals = new TreeSet<ReasonerInstance>();
		System.out.println(_relations);
		
		for (ReasonerRelation relation : _relations) {
			System.out.println(relation.getRelationName());			
			if (relation.getRelationName().equals("ownedBy")) {
				ReasonerInstance potentialOwner = (ReasonerInstance) relation.getArg2();
				for (ReasonerRelation _rel : myPelletWrapper.getRelations(potentialOwner)) {
					if (_rel.getRelationName().equals("hasName")) {
						if (userKnows(_intdRef.getName(), _rel.getArg2().getName())) {
							retVals.add(potentialOwner);
						} 
						
					}
				}
				
			}
		}
		return retVals;
	}

	
	public boolean userKnows(String _ins, String _val) {
		System.out.println("called user knows: " + _ins + " " +_val);
		return true;
	}
	
	public TreeSet<ReasonerInstance> rulesOutCon(ReasonerConcept _concept, TreeSet<ReasonerInstance> _contrastSet) {
		System.out.println("called rulesOutCon: "+_concept.getName()+" "+_contrastSet);
		TreeSet<ReasonerInstance> retSet = new TreeSet<ReasonerInstance>();
		if (_concept==null) return null;
		else {
			for (ReasonerInstance instance : _contrastSet) {
				//System.out.println("pellet->getInstances("+_concept+") => " + myPelletWrapper.getInstances(_concept));
				if (!(myPelletWrapper.getInstances(_concept).contains(instance))) {
					// user knows omitted here
					//if (userKnows(instance.getName(), _concept.getName())) {
						retSet.add(instance);
					//}
				}
			}
		}
		return retSet;
	}
	
	public TreeSet<ReasonerInstance> rulesOutName(ReasonerInstance _name, TreeSet<ReasonerInstance> _contrastSet) {
		System.out.println("called rulesOutName");
		TreeSet<ReasonerInstance> retSet = new TreeSet<ReasonerInstance>();
		if (_name==null) return null;
		else {
			for (ReasonerInstance instance : _contrastSet) {
				if (!myPelletWrapper.getRelatedInstances(instance).contains(_name)) {
					// user knows omitted here
					//if (userKnows(instance.getName(), _concept.getName())) {
						retSet.add((ReasonerInstance) instance);
					//}
				}
			}
		}
		return retSet;
	}

	public TreeSet<ReasonerInstance> rulesOutOwner(ReasonerInstance _owner, TreeSet<ReasonerInstance> _contrastSet) {
		System.out.println("called rulesOutName");
		TreeSet<ReasonerInstance> retSet = new TreeSet<ReasonerInstance>();
		if (_owner==null) return null;
		else {
			for (ReasonerInstance instance : _contrastSet) {
				if (!myPelletWrapper.getRelatedInstances(instance).contains(_owner)) {
					// user knows omitted here
					//if (userKnows(instance.getName(), _concept.getName())) {

					for (ReasonerRelation _rel : myPelletWrapper.getRelations(_owner)) {
						if (_rel.getRelationName().equals("hasName")) {
							retSet.add((ReasonerInstance)_rel.getArg2());
						}
					}
				}
			}
		}
		return retSet;
	}

	public static void main(String[] args) {
		TestGRE myGRE = new TestGRE();
		OntologyMemberFactory oeMemberMaker = new OntologyMemberFactory("http://www.dfki.de/cosy/officeenv.owl", "#");
		OntologyMemberFactory wnMemberMaker = new OntologyMemberFactory("http://www.dfki.de/cosy/wordnet", "#");

		System.out.println("DEBUG:");
		System.out.println(myGRE.myPelletWrapper.getInstances(oeMemberMaker.createConcept("PhysicalEntity")));
		
		LinkedList<Attribute> prefAtt = new LinkedList<Attribute>();
		prefAtt.add(TestGRE.Attribute.TYPE);
		prefAtt.add(TestGRE.Attribute.OWNERSHIP);
		prefAtt.add(TestGRE.Attribute.NAME);
		for (int i = 1; i < 13; i++) {
			System.out.println("------------"+i+"----------------");
			LinkedList<String> expression = myGRE.makeRefEx(oeMemberMaker.createInstance("area"+i), 
					(TreeSet<ReasonerInstance>) myGRE.myPelletWrapper.getInstances(wnMemberMaker.createConcept("PhysicalEntity")), 
					prefAtt);
			System.out.println("THE EXPRESSION IS:");
			if (expression.isEmpty()) {
				System.out.println("empty");
				continue;
			}
			for (String string: expression) {
				if (string!=null) System.out.println(string);
				else System.out.println("NULL");
			}
		}
		
		
		
		/*System.out.println("GRE examples");
		PelletWrapper myPelletWrapper;
		try {
			String _ontologyFile = "/project/cl/cosy/svn.cosy/code/subarchitectures/coma.sa/ontologies/officeenv_wn_ins.owl";
			String _ns = "http://www.dfki.de/cosy/officeenv.owl";
			myPelletWrapper = new PelletWrapper(_ontologyFile, _ns);

		System.out.println("loaded the ontology, printing the taxonomy:");
		System.out.println(myPelletWrapper.getTaxonomyTree(false));

		for (Instance _currIns : myPelletWrapper.getAllInstances()) {
			System.out.println(_currIns.getName());
		}
		
		for (Instance _currIns : myPelletWrapper.getInstances(new Concept("PhysicalEntity",Concept.Namespace.WORDNET))) {
			System.out.println(_currIns.getName());
		}

		
		} catch (ReasonerException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}*/
	}
	
}
