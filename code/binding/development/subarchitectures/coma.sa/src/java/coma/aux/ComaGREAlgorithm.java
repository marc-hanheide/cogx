package coma.aux;

import java.util.LinkedList;
import java.util.Set;
import java.util.TreeSet;


import org.cognitivesystems.comsys.processing.GREAttributeTemplateFiller.GREAttribute;
import org.cognitivesystems.comsys.processing.GREAttributeTemplateFiller;
import org.cognitivesystems.reasoner.base.OntologyMemberFactory;
import org.cognitivesystems.reasoner.base.ReasonerConcept;
import org.cognitivesystems.reasoner.base.ReasonerInstance;
import org.cognitivesystems.reasoner.base.ReasonerInterface;

import coma.components.ComaReasoner;

public class ComaGREAlgorithm {
	
	private ComaReasoner m_parentReasoner;
	private LinkedList<GREAttribute> m_prefAtt;
	static private int m_globalStartIndex = 42;

	public ComaGREAlgorithm(ComaReasoner _parentReasoner) {
		m_prefAtt = new LinkedList<GREAttribute>();
		m_prefAtt.add(GREAttribute.TYPE);
		m_prefAtt.add(GREAttribute.TOPOLOGICAL_INCLUSION);
		m_prefAtt.add(GREAttribute.OWNERSHIP);
		m_prefAtt.add(GREAttribute.NAME);
		m_prefAtt.add(GREAttribute.NUMBER_TAG);
		m_parentReasoner = _parentReasoner;
	}

	private ReasonerInterface getReasoner() {
		return m_parentReasoner.getReasoner();
	}
	
	private TreeSet<ReasonerInstance> createContextSet(ReasonerInstance _intdRef, ReasonerInstance _origin) {
		log("creating context set");
		// this method implements the creation of
		// a GRE context in large-scale space!
		TreeSet<ReasonerInstance> _context = new TreeSet<ReasonerInstance>();
		
		// 1st of all check whether int ref is contained in origin
		TreeSet<ReasonerInstance> _allOriChildren = new TreeSet<ReasonerInstance>(); 
		_allOriChildren.addAll((TreeSet<ReasonerInstance>) getReasoner().getInverseRelatedInstances(_origin, OntologyMemberFactory.createRelation("oe", ":", "topoIncluded", null, null)));
//		log("all ori children has size " + _allOriChildren.size());
		if (_allOriChildren.contains(_intdRef)) {
			_context.clear();
			_context.addAll(_allOriChildren);
			_context.add(_origin);
//			log("all ori children contains int ref -> context is now all ori children.");
		} 
		
		// starting from the origin, incrementally moving up one step at a time
		// then check whether the intended referent is a dependent node
		// if so, return all dependent nodes as the context set

		// start with the immediate containers of the origin
		TreeSet<ReasonerInstance> _topoContainers = new TreeSet<ReasonerInstance>(); 
		_topoContainers.addAll((TreeSet<ReasonerInstance>) getReasoner().getImmediateRelatedInstances(_origin, 
				OntologyMemberFactory.createRelation("oe", ":", "topoIncluded", null, null)));
//		log("topoContainers has size " + _topoContainers.size());
		// this might be problematic in multiple inheritance / lattice cases
		// need some experiments first though
		while (_context.isEmpty()) {
//			log("context still empty");
			// temporally store the immediate containers as potential new origins
			// (when incrementally enlarging the context)
			
			// now I have to build the union of the daughter nodes of all current containers
			// and then check whether they contain the intended referent
			// if so, then the context is found ;-)
			TreeSet<ReasonerInstance> _allContainerChildren = new TreeSet<ReasonerInstance>();
			TreeSet<ReasonerInstance> _moreGeneralContainers = new TreeSet<ReasonerInstance>(); 
			for (ReasonerInstance instance : _topoContainers) {
				_allContainerChildren.addAll(
						(TreeSet<ReasonerInstance>) getReasoner().
						getInverseRelatedInstances(instance, 
								OntologyMemberFactory.createRelation(
										"oe", ":", "topoIncluded", null, null))
				);
//				log("all container children has size " + _allContainerChildren.size());
				_moreGeneralContainers.addAll(
						(TreeSet<ReasonerInstance>) getReasoner().
						getImmediateRelatedInstances(instance, 
								OntologyMemberFactory.createRelation(
										"oe", ":", "topoIncluded", null, null)));
			}
			if (_allContainerChildren.contains(_intdRef)) {
				_context.clear();
				_context.addAll(_allContainerChildren);
				_context.addAll(_topoContainers);
//				log("all container children contains int ref -> context is now all container children + all containers.");
			} 
			
//			TreeSet<ReasonerInstance> _moreGeneralContainers = new TreeSet<ReasonerInstance>(); 
//			log("got more general containers. it is a set of size " + _moreGeneralContainers.size());
//			// go thru the currently known topological containers
//			for (ReasonerInstance instance : _topoContainers) {
//				log("still cycling thru all containers. current container: " + instance.getName());
//				// get all children of the current container
//				TreeSet<ReasonerInstance> _allChildren = (TreeSet<ReasonerInstance>) getReasoner().getInverseRelatedInstances(instance, OntologyMemberFactory.createRelation("oe", ":", "topoIncluded", null, null));
//				// if the current container "dominates" the intended referent, we should 
//				// add all its children to the produced context set
//				if (_allChildren.contains(_intdRef)) _context.addAll(_allChildren);
//				// just in case we do not have a context, we should take
//				// into account the next level of abstraction as the containers
//				_moreGeneralContainers.addAll((TreeSet<ReasonerInstance>) getReasoner().
//						getImmediateRelatedInstances(instance, OntologyMemberFactory.
//								createRelation("oe", ":", "topoIncluded", null, null)));
//			}
			
			// for the next round: move up one level of abstraction 
			_topoContainers.clear();
			_topoContainers.addAll(_moreGeneralContainers);
		}
//		log("done creating context set -- has size:" +_context.size());
		return _context;
	}
	
	public String generateRefEx(ReasonerInstance _intdRef, ReasonerInstance _origin) {
		return "@" + generateRefEx(_intdRef, _origin, m_globalStartIndex);
	}
	
	public String generateRefEx(ReasonerInstance _intdRef, ReasonerInstance _origin, int _index) {
//		log("generateRefEx called for " + _intdRef + " and " + _origin);
		TreeSet<ReasonerInstance> _contextSet =new TreeSet<ReasonerInstance>();  
		_contextSet.addAll(createContextSet(_intdRef, _origin));
		TreeSet<ReasonerInstance> _contrastSet = new TreeSet<ReasonerInstance>(); 
		_contrastSet.addAll(_contextSet);
		
		//sanity:
		if (_contrastSet.contains(_intdRef)) {
			_contrastSet.remove(_intdRef);
		}
		
		LinkedList<GREAttribute> _attributes = new LinkedList<GREAttribute>();
		_attributes = (LinkedList<GREAttribute>) m_prefAtt.clone();
		
		return "e" + _index++ + ":" + makeRefEx(_intdRef, _origin, _contextSet, _contrastSet, _attributes, false, _index);
	}

	
	private String makeRefEx( //String _refExSoFar,
			ReasonerInstance _intdRef, 
			ReasonerInstance _origin, 
			Set<ReasonerInstance> _contextSet,
			Set<ReasonerInstance> _contrastSet,
			LinkedList<GREAttribute> _remainingAttributes,
			boolean _typeIncluded,
			int _index) {
//		log("makeRefEx called");
		
		GREAttributeTemplateFiller _GRETemplateFiller = new GREAttributeTemplateFiller();

		TreeSet<ReasonerInstance> ruledOut = new TreeSet<ReasonerInstance>();
		
		// base case for recursion
		if (_contrastSet.isEmpty()) {
			log ("contrast set is emtpy!");
			if (_typeIncluded) return " ^ <Unique>true";
			else {
				return " ^ " + _GRETemplateFiller.fillTemplate(GREAttribute.TYPE,findBestValueCon(_intdRef, _contrastSet).getName()) +
					" ^ <Unique>true";
			}
		}

		if (_remainingAttributes.isEmpty()) {
			log("no more attributes to check. contrast set has size: " + _contrastSet.size() + ". still exiting.");
			if (_typeIncluded) return " ^ <Unique>false";
			else {
				return " ^ " + _GRETemplateFiller.fillTemplate(GREAttribute.TYPE,findBestValueCon(_intdRef, _contrastSet).getName()) +
					" ^ <Unique>false";
			}			
		}
		
		GREAttribute currAtt = _remainingAttributes.removeFirst();

		switch (currAtt) {
		case TYPE:
			log("TYPE");
			ReasonerConcept _bestCon = findBestValueCon(_intdRef, _contrastSet);
			if (_bestCon==null) {
				log("could not find best con value -- break");				
				break;
			}
			ruledOut.clear(); 
			ruledOut.addAll((TreeSet<ReasonerInstance>) rulesOutCon(_bestCon,_contrastSet));
			if (ruledOut!=null && (!ruledOut.isEmpty())) {
				_contrastSet.removeAll(ruledOut);
				if (_index==m_globalStartIndex+1) {
					return "entity(" + _GRETemplateFiller.fillTemplate(currAtt, _bestCon.getName()) + 
					makeRefEx(_intdRef, _origin, _contextSet, _contrastSet, _remainingAttributes, true, _index) +
					")";
				} else {
					return "entity ^ " + _GRETemplateFiller.fillTemplate(currAtt, _bestCon.getName()) + 
					makeRefEx(_intdRef, _origin, _contextSet, _contrastSet, _remainingAttributes, true, _index);
				}
			}
			break; // exit the switch block

		case OWNERSHIP:
			log("OWNER");
			Object[] _bestOwner = findBestValueOwnerName(_intdRef, _contrastSet);
			
//			for (ReasonerInstance vOwner : getReasoner().getInverseRelatedInstances(_intdRef, 
//					OntologyMemberFactory.createRelation("oe", ":", "owns", null, null))) {
//				log("current vOwner = " +vOwner);
				// a person (owner) might be known under many names
				// so first we have to find the "best" known name
				// eg dependent on user-knows...
//				String _bestName = findBestValueOwnerName(vOwner, _contrastSet);
				if (_bestOwner==null) {
					log("best onwer is null...");
//					continue;
					break;
				}
				// if we have a good name, we can check whether that ownership
				// relation rules out any members of the contrast set
				ruledOut.clear(); 
				ruledOut.addAll((TreeSet<ReasonerInstance>) rulesOutOwner((ReasonerInstance) _bestOwner[1],_contrastSet));
				if (ruledOut!=null && (!ruledOut.isEmpty())) {
					_contrastSet.removeAll(ruledOut);
//					return " ^ " + _GRETemplateFiller.fillTemplate(currAtt, (String) _bestOwner[0]) +
//						makeRefEx(_intdRef, _origin, _contextSet, _contrastSet, _remainingAttributes, _typeIncluded, 
//								"on to the next round from OWNER");
					return " ^ " +_GRETemplateFiller.fillTemplate(
							currAtt, 
							generateRefEx((ReasonerInstance) _bestOwner[1], _origin, _index++)) + 
//							_contextSet, _contrastSet, _remainingAttributes, false, "new recursion level from TOPO")) + ")" +
							makeRefEx(_intdRef, _origin, _contextSet, _contrastSet, _remainingAttributes, _typeIncluded, _index++);
				}
//			}
			break; // exit the switch block

		case NAME:
			log("NAME");
			String _bestName = findBestValueName(_intdRef, _contrastSet);
			if (_bestName==null) break;
			// if we have a good name, we can check whether that name
			// property rules out any members of the contrast set
			ruledOut.clear();
			ruledOut.addAll((TreeSet<ReasonerInstance>) rulesOutName(_bestName,_contrastSet));
			if (ruledOut!=null && (!ruledOut.isEmpty())) {
				_contrastSet.removeAll(ruledOut);
				return " ^ " + _GRETemplateFiller.fillTemplate(currAtt, _bestName) +
					makeRefEx(_intdRef, _origin, _contextSet, _contrastSet, _remainingAttributes, _typeIncluded
							, _index);
			}
			break; // exit the switch block

		case NUMBER_TAG:
			log("NUMBER_TAG");
			Integer _bestNumber = findBestValueNumber(_intdRef, _contrastSet);
			if (_bestNumber==null) break;
			// if we have a good name, we can check whether that name
			// property rules out any members of the contrast set
			ruledOut.clear();
			ruledOut.addAll((TreeSet<ReasonerInstance>) rulesOutNumber(_bestNumber,_contrastSet));
			if (ruledOut!=null && (!ruledOut.isEmpty())) {
				_contrastSet.removeAll(ruledOut);
				return " ^ " + _GRETemplateFiller.fillTemplate(currAtt, _bestNumber.toString()) +
					makeRefEx(_intdRef, _origin, _contextSet, _contrastSet, _remainingAttributes, _typeIncluded, _index);
			}
			break; // exit the switch block

		case TOPOLOGICAL_INCLUSION:
			log("TOPOLOGICAL_INCLUSION");
			// here I have to be careful to only take into account 
			// "human spatial concepts"

			// get the immediate container
			// check if that one is a "human concept", if not, I'say, move one up
			// and then generate a ref ex for that one
			ReasonerInstance _bestContainer = findBestValueTopoContainer(_intdRef, _contrastSet, _contextSet);
			if (_bestContainer==null) {
				log("there is no best container -> breaking");
				break;
			}
			ruledOut.clear();
			ruledOut.addAll((TreeSet<ReasonerInstance>) rulesOutTopoContainer(_bestContainer,_contrastSet));
			if (ruledOut!=null && (!ruledOut.isEmpty())) {
				// ok, check if we can actually generate a prepositional phrase
				// that makes sense to humans
				// 1) check if it's an "on" or "in" relation
				// 2a) fill in the template with the correct preposition
				// 2b) if none of the 2 can be inferred, we cannot produce a good RefEx
				
				boolean _isOnRelation = getReasoner().areInstancesRelated(_intdRef, _bestContainer, 
						OntologyMemberFactory.createRelation("oe", ":", "on", null, null));
				boolean _isInRelation = getReasoner().areInstancesRelated(_intdRef, _bestContainer, 
						OntologyMemberFactory.createRelation("oe", ":", "in", null, null));
				
				if (!_isInRelation && !_isOnRelation) {
					// cannot produce meaningful RefEx using this attribute
					// because I do not have a "human spatial proposition"
					log("don't know a 'human spatial preposition.... breaking");
					break; // exit the switch block
				}
				String _topPrepositionName = (_isOnRelation ? "on" : "in");
				
				_contrastSet.removeAll(ruledOut);
				return " ^ " +_GRETemplateFiller.fillTemplate(
						currAtt, 
						generateRefEx(_bestContainer, _origin, _index++) + 
						";" + _topPrepositionName) + 
//						_contextSet, _contrastSet, _remainingAttributes, false, "new recursion level from TOPO")) + ")" +
						makeRefEx(_intdRef, _origin, _contextSet, _contrastSet, _remainingAttributes, _typeIncluded, _index);
			} else { log("ruled out empty... breaking"); }
			break; // exit the switch block
		default:
			log("got an attribute that I cannot handle... DEFAULT = will go to next iteration round.");
			return makeRefEx(_intdRef, _origin, _contextSet, _contrastSet, _remainingAttributes, _typeIncluded, _index);
		}

		log("end makeRefEx reached with nothing new to add :-( going to next iteration round.");
		return makeRefEx(_intdRef, _origin, _contextSet, _contrastSet, _remainingAttributes, _typeIncluded, _index);
	}
	
	
	/**
	 * done.
	 * 
	 * @param _intdRef
	 * @param _contrastSet
	 * @return
	 */
	public ReasonerConcept findBestValueCon(ReasonerInstance _intdRef, Set<ReasonerInstance> _contrastSet) { //, ReasonerConcept _inititalValue) {
//		log("find best value con called!");
		ReasonerConcept retVal = null;
		TreeSet<ReasonerInstance> _potentiallyRuledOut = new TreeSet<ReasonerInstance>();
		TreeSet<ReasonerConcept> _bestCons = new TreeSet<ReasonerConcept>();
		
		if (getReasoner().getBasicLevelConcepts(_intdRef).size()>0) {
			_bestCons.addAll(getReasoner().getBasicLevelConcepts(_intdRef));
		} else {
			_bestCons.addAll(getReasoner().getMostSpecificConcepts(_intdRef));
		}
		
		for (ReasonerConcept vCon : _bestCons) {
//		for (ReasonerConcept vCon : getReasoner().getAllConcepts(_intdRef)) {
//			log("current basic level concept = " + vCon);
			if (userKnows(_intdRef.getName(), vCon.getName())) {
				if (rulesOutCon(vCon, _contrastSet).size() >= _potentiallyRuledOut.size()) {
					retVal = vCon;
					_potentiallyRuledOut.clear();
					_potentiallyRuledOut.addAll((TreeSet<ReasonerInstance>) rulesOutCon(vCon, _contrastSet));
				}
			}
		}
		log("best basic level concept = " + retVal);
		// rest omitted!
		return retVal;
	}
	
	/**
	 * done.
	 * 
	 * @param _intdRef
	 * @param _contrastSet
	 * @return
	 */
	public Object[] findBestValueOwnerName(ReasonerInstance _intdRef, Set<ReasonerInstance> _contrastSet) {
		Object[] _returnArray = new Object[2];
		// an entitiy might have several owners
		// and an owner in turn might have several names
		// and also several people can have the same name
		// TODO can I have coordination like "Henrik's and Hendrik's office?"
		// TODO answer is: currently no; comsys does not handle coordination properly (at all?)
		
		// a person (owner) might be known under many names
		// so first we have to find the "best" known name
		// eg dependent on user-knows...
		String retOwnerName = null;
		ReasonerInstance retInstance = null;
		TreeSet<ReasonerInstance> _potentiallyRuledOut = new TreeSet<ReasonerInstance>();

		for (ReasonerInstance vOwner : getReasoner().getInverseRelatedInstances(_intdRef, 
				OntologyMemberFactory.createRelation("oe", ":", "owns", null, null))) {
			for (String _vName : getReasoner().getNames(vOwner)) {
//				log("current owner: " + vOwner + " -- current name of the owner: " + _vName);

				// the name must uniquely identify a person in the context!!!
				if (getReasoner().getInstancesByName(_vName).size()>1) continue;
				// ok, we now have a unique name
				
				// let's check whether the user knows the owner under that name
				// (implicit assumption: if owner is unknown then this will also reliably fail *g*)
				if (!userKnows(vOwner.getName(), _vName)) continue;

				// now check whether the user knows that he is the owner
				if (!userKnows(vOwner.getName(), _intdRef.getName())) continue;
				
				if (rulesOutOwner(vOwner, _contrastSet).size() > _potentiallyRuledOut.size()) {
					retOwnerName = _vName;
					retInstance = vOwner;
					_potentiallyRuledOut.clear();
					_potentiallyRuledOut.addAll((TreeSet<ReasonerInstance>) rulesOutOwner(vOwner, _contrastSet));
				}
			}
			
		}
		if (retOwnerName!=null && retInstance != null) {
			_returnArray[0] = retOwnerName;
			_returnArray[1] = retInstance;
		}
		else {
			_returnArray = null;
		}
		
		return _returnArray;
	}

	
	

	/**
	 * done.
	 * 
	 * @param _intdRef
	 * @param _contrastSet
	 * @return
	 */
	public String findBestValueName(ReasonerInstance _intdRef, Set<ReasonerInstance> _contrastSet) {
		String retName = null;
		TreeSet<ReasonerInstance> _potentiallyRuledOut = new TreeSet<ReasonerInstance>();

//		log(_intdRef + " has this many names: " + getReasoner().getNames(_intdRef).size());
		
		for (String _vName : getReasoner().getNames(_intdRef)) {
			// let's check whether the user knows the intd ref under that name
			if (!userKnows(_intdRef.getName(), _vName)) continue;

//			log("current name: " + _vName);
			
			if (rulesOutName(_vName, _contrastSet).size() > _potentiallyRuledOut.size()) {
				retName = _vName;
				_potentiallyRuledOut.clear();
				_potentiallyRuledOut.addAll((TreeSet<ReasonerInstance>) rulesOutName(_vName, _contrastSet));
			}

		}
		return retName;
	}
	

	/**
	 * done.
	 * 
	 * @param _intdRef
	 * @param _contrastSet
	 * @return
	 */
	public Integer findBestValueNumber(ReasonerInstance _intdRef, Set<ReasonerInstance> _contrastSet) {
		Integer retNumber = null;
		TreeSet<ReasonerInstance> _potentiallyRuledOut = new TreeSet<ReasonerInstance>();

//		log(_intdRef + " has this many numbers: " + getReasoner().getNumberTags(_intdRef).size());
		
		for (Integer _vNumTag : getReasoner().getNumberTags(_intdRef)) {
			// let's check whether the user knows the intd ref under that name
			if (!userKnows(_intdRef.getName(), _vNumTag.toString())) continue;

//			log("current number: " + _vNumTag);
			
			if (rulesOutNumber(_vNumTag, _contrastSet).size() > _potentiallyRuledOut.size()) {
				retNumber = _vNumTag;
				_potentiallyRuledOut.clear();
				_potentiallyRuledOut.addAll((TreeSet<ReasonerInstance>) rulesOutNumber(_vNumTag, _contrastSet));
			}

		}
		return retNumber;
	}
	
	
	public ReasonerInstance findBestValueTopoContainer(ReasonerInstance _intdRef, Set<ReasonerInstance> _contrastSet, Set<ReasonerInstance> _contextSet) { 
		// get the immediate container
		// check if that one is a "human concept", if not, I'say, move one up
		// and then generate a ref ex for that one
		ReasonerInstance _bestTopoContainer = null;
		TreeSet<ReasonerInstance> _potentiallyRuledOut = new TreeSet<ReasonerInstance>();
		LinkedList<ReasonerInstance> _currentTopoLevelQueue= new LinkedList<ReasonerInstance>(); 
		_currentTopoLevelQueue.add(_intdRef);
		
		while (_bestTopoContainer == null && (!_currentTopoLevelQueue.isEmpty())) {
			ReasonerInstance _currentTopoLevel = _currentTopoLevelQueue.removeFirst();
			for (ReasonerInstance _vContainer : getReasoner().getImmediateRelatedInstances(_currentTopoLevel,
					OntologyMemberFactory.createRelation("oe", ":", "topoIncluded", null, null))) {
				// let's check whether we are actually still in the context!
				if (!_contextSet.contains(_vContainer)) continue;
				
				// let's check whether the user knows that intd ref is included there
				if (!userKnows(_intdRef.getName(), _vContainer.getName())) continue;

				// let's check whether the container has a "human spatial concept"
				if (getReasoner().getBasicLevelConcepts(_vContainer).size()<1) continue;

				if (rulesOutTopoContainer(_vContainer, _contrastSet).size() > _potentiallyRuledOut.size()) {
					_bestTopoContainer = _vContainer;
					_potentiallyRuledOut.clear();
					_potentiallyRuledOut.addAll((TreeSet<ReasonerInstance>) rulesOutTopoContainer(_vContainer, _contrastSet));
				}
				_currentTopoLevelQueue.add(_vContainer);
			}
			// now we have checked everything inside the current topological layer 
			if (_bestTopoContainer!=null) break; // get out of the loop!
		}
		return _bestTopoContainer;		
	}

	
	/**
	 * TODO make a sophisticated user model
	 * 
	 * @param _ins
	 * @param _val
	 * @return
	 */
	public boolean userKnows(String _ins, String _val) {
		System.out.println("called user knows: " + _ins + " " +_val);
		return true;
	}
	
	
	/**
	 * done.
	 * 
	 * @param _concept
	 * @param _contrastSet
	 * @return
	 */
	public Set<ReasonerInstance> rulesOutCon(ReasonerConcept _concept, Set<ReasonerInstance> _contrastSet) {
//		log("rules out concept called...");
		TreeSet<ReasonerInstance> retSet = new TreeSet<ReasonerInstance>();
		if (_concept==null) return null;
		else {
			retSet.addAll((TreeSet) _contrastSet);
//			log("ret set (copy of contrast set) has size" + retSet.size());
			retSet.removeAll(getReasoner().getInstances(_concept));
//			log("ret set has size " + retSet.size());
		}
		return retSet;
		// TODO user knows omitted!
	}
	
	
	/**
	 * done.
	 * 
	 * @param _name
	 * @param _contrastSet
	 * @return
	 */
	public Set<ReasonerInstance> rulesOutName(String _name, Set<ReasonerInstance> _contrastSet) {
		TreeSet<ReasonerInstance> retSet = new TreeSet<ReasonerInstance>();
		if (_name==null) return null;
		else {
			retSet.addAll((TreeSet) _contrastSet);
			retSet.removeAll(getReasoner().getInstancesByName(_name));
		}
		return retSet;
		// TODO user knows omitted!
	}

	/**
	 * done.
	 * 
	 * @param _name
	 * @param _contrastSet
	 * @return
	 */
	public Set<ReasonerInstance> rulesOutNumber(Integer _number, Set<ReasonerInstance> _contrastSet) {
		TreeSet<ReasonerInstance> retSet = new TreeSet<ReasonerInstance>();
		if (_number==null) return null;
		else {
			retSet.addAll((TreeSet) _contrastSet);
			retSet.removeAll(getReasoner().getInstancesByNumberTag(_number));
		}
		return retSet;
		// TODO user knows omitted!
	}

	
	/**
	 * done.
	 * 
	 * @param _owner
	 * @param _contrastSet
	 * @return
	 */
	public Set<ReasonerInstance> rulesOutOwner(ReasonerInstance _owner, Set<ReasonerInstance> _contrastSet) {
		TreeSet<ReasonerInstance> retSet = new TreeSet<ReasonerInstance>();
		if (_owner==null) return null;
		else {
			retSet.addAll((TreeSet) _contrastSet);
			retSet.removeAll(getReasoner().getRelatedInstances(_owner, 
					OntologyMemberFactory.createRelation("oe", ":", "owns", null, null)));
		}
		return retSet;
		// TODO user knows omitted!
	}

	/**
	 * done.
	 * 
	 * @param _owner
	 * @param _contrastSet
	 * @return
	 */
	public Set<ReasonerInstance> rulesOutTopoContainer(ReasonerInstance _container, Set<ReasonerInstance> _contrastSet) {
		TreeSet<ReasonerInstance> retSet = new TreeSet<ReasonerInstance>();
		if (_container==null) return null;
		else {
			retSet.addAll((TreeSet) _contrastSet);
			retSet.removeAll(getReasoner().getRelatedInstances(_container, 
					OntologyMemberFactory.createRelation("oe", ":", "topoContains", null, null)));
		}
		return retSet;
		// TODO user knows omitted!
	}
	
	public void log(String _logMsg) {
		m_parentReasoner.log(_logMsg);
	}
}
