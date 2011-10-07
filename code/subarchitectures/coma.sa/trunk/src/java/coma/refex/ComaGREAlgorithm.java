package coma.refex;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Properties;
import java.util.Queue;
import java.util.Set;
import java.util.TreeSet;

//import coma.gui.ABoxGraph;
import coma.refex.GREAttributeTemplateFiller.GREAttribute;
import comadata.ComaReasonerInterfacePrx;

public class ComaGREAlgorithm {

	private ComaReasonerInterfacePrx m_parentReasoner;
	private LinkedList<GREAttribute> m_prefAtt;
	private int m_startIndex = 42;
	private boolean m_logging = false;
	// private ABoxGraph graph;
	private String anchor_modelA;
	private String anchor_modelR;
	private String anchor_initial;
	private String anchor_modelG;

	private String activeModel;

	private Properties parameters;
	
	public final static String NAME_PROPERTY = "dora:hasName"; 
	public final static String NUMBER_TAG_PROPERTY = "dora:hasNumber"; 

	// private String topologicalParentRelation = "dora:topoIncluded";
	// private String topologicalDescendantRelation = "dora:topoIncludesTrans";
	// private String onRelation = "dora:on";
	// private String ownRelation = "dora:own";

	public ComaGREAlgorithm(ComaReasonerInterfacePrx _parentReasoner, String path) {
		m_prefAtt = new LinkedList<GREAttribute>();
		m_prefAtt.add(GREAttribute.TYPE);
		// m_prefAtt.add(GREAttribute.TOPOLOGICAL_IN);
		// m_prefAtt.add(GREAttribute.TOPOLOGICAL_ON);
		m_prefAtt.add(GREAttribute.TOPOLOGICAL_INCLUSION);
		// m_prefAtt.add(GREAttribute.OWNERSHIP);
		// m_prefAtt.add(GREAttribute.NAME);
		// m_prefAtt.add(GREAttribute.NUMBER_TAG);
		m_parentReasoner = _parentReasoner;

		FileInputStream input;
		try {
			input = new FileInputStream(path);
			parameters = new Properties();
			parameters.load(input);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			return;
		} catch (IOException e) {
			e.printStackTrace();
			return;
		}
		// graph = new ABoxGraph(_parentReasoner, parameters);
	}
	
	public ComaGREAlgorithm(ComaReasonerInterfacePrx _parentReasoner) {
		m_prefAtt = new LinkedList<GREAttribute>();
		m_prefAtt.add(GREAttribute.TYPE);
		// m_prefAtt.add(GREAttribute.TOPOLOGICAL_IN);
		// m_prefAtt.add(GREAttribute.TOPOLOGICAL_ON);
		m_prefAtt.add(GREAttribute.TOPOLOGICAL_INCLUSION);
		// m_prefAtt.add(GREAttribute.OWNERSHIP);
		// m_prefAtt.add(GREAttribute.NAME);
		// m_prefAtt.add(GREAttribute.NUMBER_TAG);
		m_parentReasoner = _parentReasoner;

		parameters = new Properties();
		parameters.put("topologicalParentRelation", "dora:topoIncluded");
		parameters.put("topologicalDescendantRelation", "dora:topoIncludesTrans");
		parameters.put("topConcept", "dora:Scene");
		parameters.put("on", "dora:on");
		parameters.put ("in", "dora:in");
		//http\://www.w3.org/1999/02/22-rdf-syntax-ns = rdf
		//http\://www.w3.org/2001/XMLSchema = xsd 
		//http\://www.w3.org/2000/01/rdf-schema = rdfs
		//http\://www.w3.org/2002/07/owl = owl
		//http\://dora.cogx.eu = dora
		//http\://cogx.eu/anchorexp.owl = anchor
	}

	public void setGlobalAnchor(String _a) {
		this.anchor_modelG = _a;
	}

	public void setInitialAnchor(String _a) {
		this.anchor_initial = _a;
		this.anchor_modelR = _a;
		this.anchor_modelA = _a;
	}

	public void resetAnchor() {
		this.anchor_modelR = this.anchor_initial;
	}

	public String generateRefExA(String _intdRef) throws Exception {
		if (anchor_modelA == null)
			throw new Exception("set the anchor first!");
		log("GRE with model A, a=" + anchor_modelA);
		this.activeModel = "A";
		return "A@" + generateRefEx(_intdRef, anchor_modelA, m_startIndex++);
	}

	public String generateRefExR(String _intdRef) throws Exception {
		if (anchor_modelR == null)
			throw new Exception("set the anchor first!");
		this.activeModel = "R";
		return "R@" + generateRefEx(_intdRef, anchor_modelR, m_startIndex++);
	}

	public String generateRefExG(String _intdRef) throws Exception {
		log("GRE with model G, a=" + anchor_modelG);
		this.activeModel = "G";
		if (anchor_modelG == null)
			throw new Exception("set the global anchor first!");
		return "G@" + generateRefEx(_intdRef, anchor_modelG, m_startIndex++);
	}
	
	public String generateRefExGCannedTextString(String _intdRef) throws Exception {
		log("GRE with model G, a=" + anchor_modelG);
		this.activeModel = "G";
		if (anchor_modelG == null)
			throw new Exception("set the global anchor first!");
		 
		String refEx = generateRefExCannedText(_intdRef, anchor_modelG);
		if (refEx.contains(" ^ <Unique>true")) {
			refEx.replace(" ^ <Unique>true", "");
			refEx = "the " + refEx;
		} else if (refEx.contains(" ^ <Unique>false")) {
			refEx.replace(" ^ <Unique>false", "");
			refEx = "a " + refEx;
		}
		return refEx;
	}


	public String generateRefEx(String _intdRef, String _origin) {
		setGlobalAnchor(_origin);
		try {
			return generateRefExG(_intdRef);
		} catch (Exception e) {
			System.err.println(e.getMessage());
			e.printStackTrace();
		}
		return "FAIL";
	}
	
	public String generateRefExCannedText(String _intdRef, String _origin) {
		// log("generateRefEx called for " + _intdRef + " and " + _origin);
//		_intdRef = handelURI(_intdRef);
//		_origin = handelURI(_origin);
		TreeSet<String> _contextSet = new TreeSet<String>();
		_contextSet.addAll(createContextSet(_intdRef, _origin));
		TreeSet<String> _contrastSet = new TreeSet<String>();
		_contrastSet.addAll(_contextSet);

		// sanity:
		if (_contrastSet.contains(_intdRef)) {
			_contrastSet.remove(_intdRef);
		}

		LinkedList<GREAttribute> _attributes = new LinkedList<GREAttribute>();
		_attributes = (LinkedList<GREAttribute>) m_prefAtt.clone();

		return "TODO"; // makeRefExCannedText(_intdRef, _origin, _contextSet, _contrastSet,	_attributes, false);
	}


	public String generateRefEx(String _intdRef, String _origin, int _index) {
		// log("generateRefEx called for " + _intdRef + " and " + _origin);
//		_intdRef = handelURI(_intdRef);
//		_origin = handelURI(_origin);
		TreeSet<String> _contextSet = new TreeSet<String>();
		_contextSet.addAll(createContextSet(_intdRef, _origin));
		TreeSet<String> _contrastSet = new TreeSet<String>();
		_contrastSet.addAll(_contextSet);

		// sanity:
		if (_contrastSet.contains(_intdRef)) {
			_contrastSet.remove(_intdRef);
		}

		LinkedList<GREAttribute> _attributes = new LinkedList<GREAttribute>();
		_attributes = (LinkedList<GREAttribute>) m_prefAtt.clone();

		return "e"
				+ m_startIndex++
				+ ":"
				+ makeRefEx(_intdRef, _origin, _contextSet, _contrastSet,
						_attributes, false, _index);
	}

	/*
	private String makeRefExCannedText(
			// String _refExSoFar,
			String _intdRef, String _origin, Set<String> _contextSet,
			Set<String> _contrastSet,
			LinkedList<GREAttribute> _remainingAttributes,
			boolean _typeIncluded) {
		log("makeRefExCannedText called with r = " + _intdRef + " a = " + _origin
				+ " context = " + stringsetToString(_contextSet)
				+ " contrast = " + stringsetToString(_contrastSet));

		log("!!! setting anchor to intRef " + _intdRef);
		anchor_modelA = _intdRef;
		anchor_modelR = _intdRef;

		String dependentREanchor = _intdRef;

		GREAttributeTemplateFiller _GRETemplateFiller = new GREAttributeTemplateFiller();

		TreeSet<String> ruledOut = new TreeSet<String>();

		// base case for recursion
		if (_contrastSet.isEmpty()) {
			log("contrast set is emtpy!");
			if (_typeIncluded)
				return " ^ <Unique>true";
			else {
				return " ^ "
						+ _GRETemplateFiller.fillTemplate(GREAttribute.TYPE,
								findBestValueCon(_intdRef, _contrastSet))
						+ " ^ <Unique>true";
			}
		}

		if (_remainingAttributes.isEmpty()) {
			log("no more attributes to check. contrast set has size: "
					+ _contrastSet.size() + ". contrast set = "
					+ stringsetToString(_contrastSet) + "... still exiting.");
			if (_typeIncluded)
				return " ^ <Unique>false";
			else {
				return " ^ "
						+ _GRETemplateFiller.fillTemplate(GREAttribute.TYPE,
								findBestValueCon(_intdRef, _contrastSet))
						+ " ^ <Unique>false";
			}
		}

		GREAttribute currAtt = _remainingAttributes.removeFirst();

		switch (currAtt) {
		case TYPE:
			log("TYPE");
			String _bestCon = findBestValueCon(_intdRef, _contrastSet);
			if (_bestCon == null) {
				log("could not find best con value -- break");
				break;
			}
			ruledOut.clear();
			ruledOut.addAll((TreeSet<String>) rulesOutCon(_bestCon,
					_contrastSet));
			if (ruledOut != null && (!ruledOut.isEmpty())) {
				_contrastSet.removeAll(ruledOut);

				return _GRETemplateFiller
				.fillTemplate(currAtt, _bestCon)
				+ makeRefExCannedText(_intdRef, _origin, _contextSet,
						_contrastSet, _remainingAttributes, true);

			}
			break; // exit the switch block

		case TOPOLOGICAL_INCLUSION:
			log("TOPOLOGICAL_INCLUSION");
			// here I have to be careful to only take into account
			// "human spatial concepts"

			// get the immediate container
			// check if that one is a "human concept", if not, I'd say, move one
			// up
			// and then generate a ref ex for that one
			String[] _bestRelation = findBestValueRelation(_intdRef,
					_contrastSet, _contextSet);
			if (_bestRelation == null) {
				log("there is no best relation-> breaking");
				break;
			} else {
				log("best topological_inclusion relation: " + _bestRelation[0]
						+ " / " + _bestRelation[1]);
			}
			ruledOut.clear();
			ruledOut.addAll((TreeSet<String>) rulesOutRelation(
					_bestRelation[0], _bestRelation[1], _contrastSet));
			if (ruledOut != null && (!ruledOut.isEmpty())) {
				// ok, check if we can actually generate a prepositional phrase
				// that makes sense to humans
				// 1) check if it's an "on" or "in" relation
				// 2a) fill in the template with the correct preposition
				// 2b) if none of the 2 can be inferred, we cannot produce a
				// good RefEx

				// boolean _isOnRelation = m_parentReasoner.areInstancesRelated(
				// _intdRef, _bestContainer, parameters.getProperty("on"));
				// boolean _isInRelation = getReasoner()
				// .areInstancesRelated(
				// _intdRef, _bestContainer, parameters.getProperty("in"));

				// if (!_isInRelation && !_isOnRelation) {
				// cannot produce meaningful RefEx using this attribute
				// because I do not have a "human spatial proposition"
				// log("don't know a 'human spatial preposition.... breaking");
				// break; // exit the switch block
				// }
				String _topPrepositionName = _bestRelation[1];

				// String _topPrepositionName = (_isOnRelation ? parameters
				// .getProperty("on") : parameters
				// .getProperty("topologicalParentRelation"));

				_contrastSet.removeAll(ruledOut);
				return _GRETemplateFiller.fillTemplate(currAtt,
								generateRefExCannedText(_bestRelation[0], _origin)
										+ ";" + _topPrepositionName) +
						// _contextSet, _contrastSet, _remainingAttributes,
						// false, "new recursion level from TOPO")) + ")" +
						makeRefExCannedText(_intdRef, _origin, _contextSet, _contrastSet,
								_remainingAttributes, _typeIncluded);
			} else {
				log("ruled out empty... breaking");
			}
			break; // exit the switch block

		case TOPOLOGICAL_IN:
			log("TOPOLOGICAL_IN");
			// here I have to be careful to only take into account
			// "human spatial concepts"

			// get the immediate container
			// check if that one is a "human concept", if not, I'say, move one
			// up
			// and then generate a ref ex for that one
			String _bestInContainer = findBestValueInContainer(_intdRef,
					_contrastSet, _contextSet);
			if (_bestInContainer == null) {
				log("there is no best container -> breaking");
				break;
			}
			ruledOut.clear();
			ruledOut.addAll((TreeSet<String>) rulesOutInContainer(
					_bestInContainer, _contrastSet));
			if (ruledOut != null && (!ruledOut.isEmpty())) {
				// ok, check if we can actually generate a prepositional phrase
				// that makes sense to humans
				// 1) check if it's an "on" or "in" relation
				// 2a) fill in the template with the correct preposition
				// 2b) if none of the 2 can be inferred, we cannot produce a
				// good RefEx

				String _topPrepositionName = (parameters.getProperty("in"));

				_contrastSet.removeAll(ruledOut);
				return " ^ "
						+ _GRETemplateFiller.fillTemplate(currAtt,
								generateRefEx(_bestInContainer, _origin,
										_index++)
										+ ";" + _topPrepositionName) +
						// _contextSet, _contrastSet, _remainingAttributes,
						// false, "new recursion level from TOPO")) + ")" +
						makeRefEx(_intdRef, _origin, _contextSet, _contrastSet,
								_remainingAttributes, _typeIncluded, _index);
			} else {
				log("ruled out empty... breaking");
			}
			break; // exit the switch block

		case TOPOLOGICAL_ON:
			log("TOPOLOGICAL_ON");
			// here I have to be careful to only take into account
			// "human spatial concepts"

			// get the immediate container
			// check if that one is a "human concept", if not, I'say, move one
			// up
			// and then generate a ref ex for that one
			String _bestOnContainer = findBestValueOnContainer(_intdRef,
					_contrastSet, _contextSet);
			if (_bestOnContainer == null) {
				log("there is no best container -> breaking");
				break;
			}
			ruledOut.clear();
			ruledOut.addAll((TreeSet<String>) rulesOutOnContainer(
					_bestOnContainer, _contrastSet));
			if (ruledOut != null && (!ruledOut.isEmpty())) {
				// ok, check if we can actually generate a prepositional phrase
				// that makes sense to humans
				// 1) check if it's an "on" or "in" relation
				// 2a) fill in the template with the correct preposition
				// 2b) if none of the 2 can be inferred, we cannot produce a
				// good RefEx

				String _topPrepositionName = (parameters.getProperty("on"));

				_contrastSet.removeAll(ruledOut);
				return " ^ "
						+ _GRETemplateFiller.fillTemplate(currAtt,
								generateRefEx(_bestOnContainer, _origin,
										_index++)
										+ ";" + _topPrepositionName) +
						// _contextSet, _contrastSet, _remainingAttributes,
						// false, "new recursion level from TOPO")) + ")" +
						makeRefEx(_intdRef, _origin, _contextSet, _contrastSet,
								_remainingAttributes, _typeIncluded, _index);
			} else {
				log("ruled out empty... breaking");
			}
			break; // exit the switch block

		default:
			log("got an attribute that I cannot handle... DEFAULT = will go to next iteration round.");
			return makeRefEx(_intdRef, _origin, _contextSet, _contrastSet,
					_remainingAttributes, _typeIncluded, _index);
		}

		log("end makeRefEx reached with nothing new to add :-( going to next iteration round.");
		return makeRefEx(_intdRef, _origin, _contextSet, _contrastSet,
				_remainingAttributes, _typeIncluded, _index);
	}	
	*/
	
	private String makeRefEx(
			// String _refExSoFar,
			String _intdRef, String _origin, Set<String> _contextSet,
			Set<String> _contrastSet,
			LinkedList<GREAttribute> _remainingAttributes,
			boolean _typeIncluded, int _index) {
		log("makeRefEx called with r = " + _intdRef + " a = " + _origin
				+ " context = " + stringsetToString(_contextSet)
				+ " contrast = " + stringsetToString(_contrastSet));

		log("!!! setting anchor to intRef " + _intdRef);
		anchor_modelA = _intdRef;
		anchor_modelR = _intdRef;

		String dependentREanchor = _intdRef;

		GREAttributeTemplateFiller _GRETemplateFiller = new GREAttributeTemplateFiller();

		TreeSet<String> ruledOut = new TreeSet<String>();

		// base case for recursion
		if (_contrastSet.isEmpty()) {
			log("contrast set is emtpy!");
			if (_typeIncluded)
				return " ^ <Unique>true";
			else {
				return " ^ "
						+ _GRETemplateFiller.fillTemplate(GREAttribute.TYPE,
								findBestValueCon(_intdRef, _contrastSet))
						+ " ^ <Unique>true";
			}
		}

		if (_remainingAttributes.isEmpty()) {
			log("no more attributes to check. contrast set has size: "
					+ _contrastSet.size() + ". contrast set = "
					+ stringsetToString(_contrastSet) + "... still exiting.");
			if (_typeIncluded)
				return " ^ <Unique>false";
			else {
				return " ^ "
						+ _GRETemplateFiller.fillTemplate(GREAttribute.TYPE,
								findBestValueCon(_intdRef, _contrastSet))
						+ " ^ <Unique>false";
			}
		}

		GREAttribute currAtt = _remainingAttributes.removeFirst();

		switch (currAtt) {
		case TYPE:
			log("TYPE");
			String _bestCon = findBestValueCon(_intdRef, _contrastSet);
			if (_bestCon == null) {
				log("could not find best con value -- break");
				break;
			}
			ruledOut.clear();
			ruledOut.addAll((TreeSet<String>) rulesOutCon(_bestCon,
					_contrastSet));
			if (ruledOut != null && (!ruledOut.isEmpty())) {
				_contrastSet.removeAll(ruledOut);
				if (_index == m_startIndex + 1) {
					return "entity("
							+ _GRETemplateFiller
									.fillTemplate(currAtt, _bestCon)
							+ makeRefEx(_intdRef, _origin, _contextSet,
									_contrastSet, _remainingAttributes, true,
									_index) + ")";
				} else {
					return "entity ^ "
							+ _GRETemplateFiller
									.fillTemplate(currAtt, _bestCon)
							+ makeRefEx(_intdRef, _origin, _contextSet,
									_contrastSet, _remainingAttributes, true,
									_index);
				}
			}
			break; // exit the switch block

		case OWNERSHIP:
			log("OWNER");
			Object[] _bestOwner = findBestValueOwnerName(_intdRef, _contrastSet);

			// for (ReasonerInstance vOwner :
			// getReasoner().getInverseRelatedInstances(_intdRef,
			// OntologyMemberFactory.createRelation("oe", ":", "owns", null,
			// null))) {
			// log("current vOwner = " +vOwner);
			// a person (owner) might be known under many names
			// so first we have to find the "best" known name
			// eg dependent on user-knows...
			// String _bestName = findBestValueOwnerName(vOwner, _contrastSet);
			if (_bestOwner == null) {
				log("best onwer is null...");
				// continue;
				break;
			}
			// if we have a good name, we can check whether that ownership
			// relation rules out any members of the contrast set
			ruledOut.clear();
			ruledOut.addAll((TreeSet<String>) rulesOutOwner(
					(String) _bestOwner[1], _contrastSet));
			if (ruledOut != null && (!ruledOut.isEmpty())) {
				_contrastSet.removeAll(ruledOut);
				// return " ^ " + _GRETemplateFiller.fillTemplate(currAtt,
				// (String) _bestOwner[0]) +
				// makeRefEx(_intdRef, _origin, _contextSet, _contrastSet,
				// _remainingAttributes, _typeIncluded,
				// "on to the next round from OWNER");
				return " ^ "
						+ _GRETemplateFiller.fillTemplate(currAtt,
								generateRefEx((String) _bestOwner[1], _origin,
										_index++)) +
						// _contextSet, _contrastSet, _remainingAttributes,
						// false, "new recursion level from TOPO")) + ")" +
						makeRefEx(_intdRef, _origin, _contextSet, _contrastSet,
								_remainingAttributes, _typeIncluded, _index++);
			}
			// }
			break; // exit the switch block

		case NAME:
			log("NAME");
			String _bestName = findBestValueName(_intdRef, _contrastSet);
			if (_bestName == null)
				break;
			// if we have a good name, we can check whether that name
			// property rules out any members of the contrast set
			ruledOut.clear();
			ruledOut.addAll((TreeSet<String>) rulesOutName(_bestName,
					_contrastSet));
			if (ruledOut != null && (!ruledOut.isEmpty())) {
				_contrastSet.removeAll(ruledOut);
				return " ^ "
						+ _GRETemplateFiller.fillTemplate(currAtt, _bestName)
						+ makeRefEx(_intdRef, _origin, _contextSet,
								_contrastSet, _remainingAttributes,
								_typeIncluded, _index);
			}
			break; // exit the switch block

		case NUMBER_TAG:
			log("NUMBER_TAG");
			String _bestNumber = findBestValueNumber(_intdRef, _contrastSet);
			if (_bestNumber == null)
				break;
			// if we have a good name, we can check whether that name
			// property rules out any members of the contrast set
			ruledOut.clear();
			ruledOut.addAll((TreeSet<String>) rulesOutNumber(_bestNumber,
					_contrastSet));
			if (ruledOut != null && (!ruledOut.isEmpty())) {
				_contrastSet.removeAll(ruledOut);
				return " ^ "
						+ _GRETemplateFiller.fillTemplate(currAtt, _bestNumber
								.toString())
						+ makeRefEx(_intdRef, _origin, _contextSet,
								_contrastSet, _remainingAttributes,
								_typeIncluded, _index);
			}
			break; // exit the switch block

		case TOPOLOGICAL_INCLUSION:
			log("TOPOLOGICAL_INCLUSION");
			// here I have to be careful to only take into account
			// "human spatial concepts"

			// get the immediate container
			// check if that one is a "human concept", if not, I'd say, move one
			// up
			// and then generate a ref ex for that one
			String[] _bestRelation = findBestValueRelation(_intdRef,
					_contrastSet, _contextSet);
			if (_bestRelation == null) {
				log("there is no best relation-> breaking");
				break;
			} else {
				log("@@@@@@@@@@@@@@@@@@@@@@ BEST RELATION: " + _bestRelation[0]
						+ " / " + _bestRelation[1]);
			}
			ruledOut.clear();
			ruledOut.addAll((TreeSet<String>) rulesOutRelation(
					_bestRelation[0], _bestRelation[1], _contrastSet));
			if (ruledOut != null && (!ruledOut.isEmpty())) {
				// ok, check if we can actually generate a prepositional phrase
				// that makes sense to humans
				// 1) check if it's an "on" or "in" relation
				// 2a) fill in the template with the correct preposition
				// 2b) if none of the 2 can be inferred, we cannot produce a
				// good RefEx

				// boolean _isOnRelation = m_parentReasoner.areInstancesRelated(
				// _intdRef, _bestContainer, parameters.getProperty("on"));
				// boolean _isInRelation = getReasoner()
				// .areInstancesRelated(
				// _intdRef, _bestContainer, parameters.getProperty("in"));

				// if (!_isInRelation && !_isOnRelation) {
				// cannot produce meaningful RefEx using this attribute
				// because I do not have a "human spatial proposition"
				// log("don't know a 'human spatial preposition.... breaking");
				// break; // exit the switch block
				// }
				String _topPrepositionName = _bestRelation[1];

				// String _topPrepositionName = (_isOnRelation ? parameters
				// .getProperty("on") : parameters
				// .getProperty("topologicalParentRelation"));

				_contrastSet.removeAll(ruledOut);
				return " ^ "
						+ _GRETemplateFiller.fillTemplate(currAtt,
								generateRefEx(_bestRelation[0], _origin,
										_index++)
										+ ";" + _topPrepositionName) +
						// _contextSet, _contrastSet, _remainingAttributes,
						// false, "new recursion level from TOPO")) + ")" +
						makeRefEx(_intdRef, _origin, _contextSet, _contrastSet,
								_remainingAttributes, _typeIncluded, _index);
			} else {
				log("ruled out empty... breaking");
			}
			break; // exit the switch block

		case TOPOLOGICAL_IN:
			log("TOPOLOGICAL_IN");
			// here I have to be careful to only take into account
			// "human spatial concepts"

			// get the immediate container
			// check if that one is a "human concept", if not, I'say, move one
			// up
			// and then generate a ref ex for that one
			String _bestInContainer = findBestValueInContainer(_intdRef,
					_contrastSet, _contextSet);
			if (_bestInContainer == null) {
				log("there is no best container -> breaking");
				break;
			}
			ruledOut.clear();
			ruledOut.addAll((TreeSet<String>) rulesOutInContainer(
					_bestInContainer, _contrastSet));
			if (ruledOut != null && (!ruledOut.isEmpty())) {
				// ok, check if we can actually generate a prepositional phrase
				// that makes sense to humans
				// 1) check if it's an "on" or "in" relation
				// 2a) fill in the template with the correct preposition
				// 2b) if none of the 2 can be inferred, we cannot produce a
				// good RefEx

				String _topPrepositionName = (parameters.getProperty("in"));

				_contrastSet.removeAll(ruledOut);
				return " ^ "
						+ _GRETemplateFiller.fillTemplate(currAtt,
								generateRefEx(_bestInContainer, _origin,
										_index++)
										+ ";" + _topPrepositionName) +
						// _contextSet, _contrastSet, _remainingAttributes,
						// false, "new recursion level from TOPO")) + ")" +
						makeRefEx(_intdRef, _origin, _contextSet, _contrastSet,
								_remainingAttributes, _typeIncluded, _index);
			} else {
				log("ruled out empty... breaking");
			}
			break; // exit the switch block

		case TOPOLOGICAL_ON:
			log("TOPOLOGICAL_ON");
			// here I have to be careful to only take into account
			// "human spatial concepts"

			// get the immediate container
			// check if that one is a "human concept", if not, I'say, move one
			// up
			// and then generate a ref ex for that one
			String _bestOnContainer = findBestValueOnContainer(_intdRef,
					_contrastSet, _contextSet);
			if (_bestOnContainer == null) {
				log("there is no best container -> breaking");
				break;
			}
			ruledOut.clear();
			ruledOut.addAll((TreeSet<String>) rulesOutOnContainer(
					_bestOnContainer, _contrastSet));
			if (ruledOut != null && (!ruledOut.isEmpty())) {
				// ok, check if we can actually generate a prepositional phrase
				// that makes sense to humans
				// 1) check if it's an "on" or "in" relation
				// 2a) fill in the template with the correct preposition
				// 2b) if none of the 2 can be inferred, we cannot produce a
				// good RefEx

				String _topPrepositionName = (parameters.getProperty("on"));

				_contrastSet.removeAll(ruledOut);
				return " ^ "
						+ _GRETemplateFiller.fillTemplate(currAtt,
								generateRefEx(_bestOnContainer, _origin,
										_index++)
										+ ";" + _topPrepositionName) +
						// _contextSet, _contrastSet, _remainingAttributes,
						// false, "new recursion level from TOPO")) + ")" +
						makeRefEx(_intdRef, _origin, _contextSet, _contrastSet,
								_remainingAttributes, _typeIncluded, _index);
			} else {
				log("ruled out empty... breaking");
			}
			break; // exit the switch block

		default:
			log("got an attribute that I cannot handle... DEFAULT = will go to next iteration round.");
			return makeRefEx(_intdRef, _origin, _contextSet, _contrastSet,
					_remainingAttributes, _typeIncluded, _index);
		}

		log("end makeRefEx reached with nothing new to add :-( going to next iteration round.");
		return makeRefEx(_intdRef, _origin, _contextSet, _contrastSet,
				_remainingAttributes, _typeIncluded, _index);
	}

	private TreeSet<String> createContextSet(String _r, String _a) {
		// init context
		TreeSet<String> context = new TreeSet<String>();
		context.add(_a);
		context.addAll(Arrays.asList(getReasoner().getRelatedInstancesByRelation(_a,
				parameters.getProperty("topologicalDescendantRelation"))));

		// some logging
		log("initiate context determination with r=" + _r + " and a=" + _a);
		log("initial context: " + stringsetToString(context));

		// check if r is in the initial context, then we're done
		if (context.contains(_r)) {
			log("initial context contains r, I am done!");
			return context;
		} else { // if not, do topological abstraction
			// create a queue of topological abstraction layers
			Queue<String> q = new LinkedList<String>();
			q.add(_a);
			log("Q init with: " + stringQToString(q));

			while (!q.isEmpty()) {
				log("current Q: " + stringQToString(q));
				String node = q.poll();
				// log("current node: " + node);
				Set<String> parents = new TreeSet<String>(Arrays.asList(getReasoner().
						getRelatedInstancesByRelation(node,
								parameters.getProperty("topologicalParentRelation"))));
				log("current node's parents: " + stringsetToString(parents));
				for (String parent : parents) {
					log("current parent: " + parent);
					q.add(parent);
					context
							.addAll(Arrays.asList(getReasoner()
									.getRelatedInstancesByRelation(parent,
											parameters.getProperty("topologicalDescendantRelation"))));
					context.add(parent);
					log("current context: " + stringsetToString(context));
				}
				if (context.contains(_r)) {
					log("context contains r, I am done!");
					log("final context: " + stringsetToString(context));
					return context;
				} else {
					log("context does not contain r");
				}
			}
		}
		log("failure / returned context: " + context);
		return context;
	}

	/**
	 * done.
	 * 
	 * @param _intdRef
	 * @param _contrastSet
	 * @return
	 */
	public String findBestValueCon(String _intdRef, Set<String> _contrastSet) { // ,
		// ReasonerConcept
		// _inititalValue)
		// {
		// log("find best value con called!");
		String retVal = null;
		TreeSet<String> _potentiallyRuledOut = new TreeSet<String>();
		TreeSet<String> _bestCons = new TreeSet<String>();

		log("inside findBestValueCon");
		if (false) { // getReasoner().getBasicLevelConcepts(_intdRef).size() >
			// 0) {
			log("basic lvl con was non empty!");
			_bestCons.addAll(Arrays.asList(getReasoner().getBasicLevelConcepts(
					_intdRef)));
		} else {
			log("basic lvl con was emtpy, let's check for most specific con instead");
			_bestCons.addAll(Arrays.asList(getReasoner().getMostSpecificConcepts(
					_intdRef)));
		}
		log(stringsetToString(_bestCons));

		for (String vCon : _bestCons) {
			// for (ReasonerConcept vCon :
			// getReasoner().getAllConcepts(_intdRef)) {
			// log("current basic level concept = " + vCon);
			if (userKnows(_intdRef, vCon)) {
				if (rulesOutCon(vCon, _contrastSet).size() >= _potentiallyRuledOut
						.size()) {
					retVal = vCon;
					_potentiallyRuledOut.clear();
					_potentiallyRuledOut.addAll((TreeSet<String>) rulesOutCon(
							vCon, _contrastSet));
				}
			}
		}
		log("best concept = " + retVal);
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
	public Object[] findBestValueOwnerName(String _intdRef,
			Set<String> _contrastSet) {
		Object[] _returnArray = new Object[2];
		// an entitiy might have several owners
		// and an owner in turn might have several names
		// and also several people can have the same name
		// TODO can I have coordination like "Henrik's and Hendrik's office?"
		// TODO answer is: currently no; comsys does not handle coordination
		// properly (at all?)

		// a person (owner) might be known under many names
		// so first we have to find the "best" known name
		// eg dependent on user-knows...
		String retOwnerName = null;
		String retInstance = null;
		TreeSet<String> _potentiallyRuledOut = new TreeSet<String>();

		for (String vOwner : Arrays.asList(getReasoner()
				.getInverseRelatedInstancesByRelation(_intdRef,
						parameters.getProperty("own")))) {
			for (String _vName : getReasoner().getPropertyValues(vOwner, NAME_PROPERTY)) {
				// log("current owner: " + vOwner +
				// " -- current name of the owner: " + _vName); 

				// the name must uniquely identify a person in the context!!!
				if (Arrays.asList(getReasoner().getInstancesByPropertyValue(NAME_PROPERTY, _vName)).size() > 1)
					continue;
				// ok, we now have a unique name

				// let's check whether the user knows the owner under that name
				// (implicit assumption: if owner is unknown then this will also
				// reliably fail *g*)
				if (!userKnows(vOwner, _vName))
					continue;

				// now check whether the user knows that he is the owner
				if (!userKnows(vOwner, _intdRef))
					continue;

				if (rulesOutOwner(vOwner, _contrastSet).size() > _potentiallyRuledOut
						.size()) {
					retOwnerName = _vName;
					retInstance = vOwner;
					_potentiallyRuledOut.clear();
					_potentiallyRuledOut
							.addAll((TreeSet<String>) rulesOutOwner(vOwner,
									_contrastSet));
				}
			}

		}
		if (retOwnerName != null && retInstance != null) {
			_returnArray[0] = retOwnerName;
			_returnArray[1] = retInstance;
		} else {
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
	public String findBestValueName(String _intdRef, Set<String> _contrastSet) {
		String retName = null;
		TreeSet<String> _potentiallyRuledOut = new TreeSet<String>();

		// log(_intdRef + " has this many names: " +
		// getReasoner().getNames(_intdRef).size());

		for (String _vName : Arrays.asList(getReasoner().getPropertyValues(_intdRef, NAME_PROPERTY))) {
			// let's check whether the user knows the intd ref under that name
			if (!userKnows(_intdRef, _vName))
				continue;

			// log("current name: " + _vName);

			if (rulesOutName(_vName, _contrastSet).size() > _potentiallyRuledOut
					.size()) {
				retName = _vName;
				_potentiallyRuledOut.clear();
				_potentiallyRuledOut.addAll((TreeSet<String>) rulesOutName(
						_vName, _contrastSet));
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
	public String findBestValueNumber(String _intdRef, Set<String> _contrastSet) {
		String retNumber = null;
		TreeSet<String> _potentiallyRuledOut = new TreeSet<String>();

		// log(_intdRef + " has this many numbers: " +
		// getReasoner().getNumberTags(_intdRef).size());

		for (String _vNumTag : getReasoner().getPropertyValues(_intdRef, NUMBER_TAG_PROPERTY)) {
			// let's check whether the user knows the intd ref under that name
			if (!userKnows(_intdRef, _vNumTag.toString()))
				continue;

			// log("current number: " + _vNumTag);

			if (rulesOutNumber(_vNumTag, _contrastSet).size() > _potentiallyRuledOut
					.size()) {
				retNumber = _vNumTag;
				_potentiallyRuledOut.clear();
				_potentiallyRuledOut.addAll((TreeSet<String>) rulesOutNumber(
						_vNumTag, _contrastSet));
			}

		}
		return retNumber;
	}

	public String[] findBestValueRelation(String _intdRef,
			Set<String> _contrastSet, Set<String> _contextSet) {

		// check for all instances that are related to the intended referent
		// by a topological inclusion relation
		// no matter if it is transitive or not
		// -- the discriminatory power of the respective relation will
		// also be taken into account

		// the return value is an array that contain the instance and
		// corresponding best relation
		String[] _bestRelation = new String[2];

		TreeSet<String> _potentiallyRuledOut = new TreeSet<String>();

		Set<String> allRelatedInstances = new TreeSet<String>(Arrays.asList(getReasoner()
				.getRelatedInstances(_intdRef)));

		for (String _currRelIns : allRelatedInstances) {
			// let's check whether we are actually still in the context!
			if (!_contextSet.contains(_currRelIns))
				continue;

			Set<String> _allCurrRels = new TreeSet<String>(Arrays.asList(getReasoner().getRelationsBetweenInstances(
					_intdRef, _currRelIns)));
			for (String _currRel : _allCurrRels) {
				// let's make sure we don't get downwards loops!
				if (!getReasoner().isSubRelation(_currRel,
						parameters.getProperty("topologicalParentRelation")))
					continue;

				// let's check whether the user knows that intd ref is related
				// to the current instance
				if (!userKnows(_intdRef, _currRel, _currRelIns))
					continue;

				if (rulesOutRelation(_currRelIns, _currRel, _contrastSet)
						.size() > _potentiallyRuledOut.size()) {
					_bestRelation[0] = _currRelIns;
					_bestRelation[1] = _currRel;
					_potentiallyRuledOut.clear();
					_potentiallyRuledOut
							.addAll((TreeSet<String>) rulesOutRelation(
									_currRelIns, _currRel, _contrastSet));
					log("found a new bestRelation: " + _bestRelation[0] + " "
							+ _bestRelation[1] + " and it rules out: "
							+ stringsetToString(_potentiallyRuledOut));
				}
			}

		}

		return _bestRelation;
	}

	public String findBestValueTopoContainer(String _intdRef,
			Set<String> _contrastSet, Set<String> _contextSet) {
		// get the immediate container
		// check if that one is a "human concept", if not, I'say, move one up
		// and then generate a ref ex for that one
		String _bestTopoContainer = null;
		TreeSet<String> _potentiallyRuledOut = new TreeSet<String>();
		LinkedList<String> _currentTopoLevelQueue = new LinkedList<String>();
		_currentTopoLevelQueue.add(_intdRef);
		log("findBestValue TopoContainer called");

		while (_bestTopoContainer == null
				&& (!_currentTopoLevelQueue.isEmpty())) {
			String _currentTopoLevel = _currentTopoLevelQueue.removeFirst();
			for (String _vContainer : Arrays.asList(getReasoner()
					.getRelatedInstancesByRelation(_currentTopoLevel,
							parameters.getProperty("topologicalParentRelation")))) {
				log("current topoContainer is: " + _vContainer);
				// let's check whether we are actually still in the context!
				if (!_contextSet.contains(_vContainer))
					continue;

				// let's check whether the user knows that intd ref is included
				// there
				if (!userKnows(_intdRef, _vContainer))
					continue;

				// let's check whether the container has a
				// "human spatial concept"
				// if (getReasoner().getBasicLevelConcepts(_vContainer).size() <
				// 1)
				// continue;

				if (rulesOutTopoContainer(_vContainer, _contrastSet).size() > _potentiallyRuledOut
						.size()) {
					_bestTopoContainer = _vContainer;
					_potentiallyRuledOut.clear();
					_potentiallyRuledOut
							.addAll((TreeSet<String>) rulesOutTopoContainer(
									_vContainer, _contrastSet));
					log("found a new bestTopoContainer: " + _bestTopoContainer
							+ " and it rules out: "
							+ stringsetToString(_potentiallyRuledOut));
				}
				_currentTopoLevelQueue.add(_vContainer);
			}
			// now we have checked everything inside the current topological
			// layer
			if (_bestTopoContainer != null)
				break; // get out of the loop!
		}
		log("returning best topo container: " + _bestTopoContainer);
		return _bestTopoContainer;
	}

	public String findBestValueInContainer(String _intdRef,
			Set<String> _contrastSet, Set<String> _contextSet) {
		// get the immediate container
		// check if that one is a "human concept", if not, I'say, move one up
		// and then generate a ref ex for that one
		String _bestTopoContainer = null;
		TreeSet<String> _potentiallyRuledOut = new TreeSet<String>();
		LinkedList<String> _currentTopoLevelQueue = new LinkedList<String>();
		_currentTopoLevelQueue.add(_intdRef);
		log("findBestValue IN Container called");

		while (_bestTopoContainer == null
				&& (!_currentTopoLevelQueue.isEmpty())) {
			String _currentTopoLevel = _currentTopoLevelQueue.removeFirst();
			for (String _vContainer : Arrays.asList(getReasoner()
					.getImmediateRelatedInstancesByRelation(_currentTopoLevel,
							parameters.getProperty("in")))) {
				// let's check whether we are actually still in the context!
				if (!_contextSet.contains(_vContainer))
					continue;

				// let's check whether the user knows that intd ref is included
				// there
				if (!userKnows(_intdRef, _vContainer))
					continue;

				// let's check whether the container has a
				// "human spatial concept"
				// if (getReasoner().getBasicLevelConcepts(_vContainer).size() <
				// 1)
				// continue;

				if (rulesOutInContainer(_vContainer, _contrastSet).size() > _potentiallyRuledOut
						.size()) {
					_bestTopoContainer = _vContainer;
					_potentiallyRuledOut.clear();
					_potentiallyRuledOut
							.addAll((TreeSet<String>) rulesOutInContainer(
									_vContainer, _contrastSet));
				}
				_currentTopoLevelQueue.add(_vContainer);
			}
			// now we have checked everything inside the current topological
			// layer
			if (_bestTopoContainer != null)
				break; // get out of the loop!
		}
		log("returning best IN container: " + _bestTopoContainer);
		return _bestTopoContainer;
	}

	public String findBestValueOnContainer(String _intdRef,
			Set<String> _contrastSet, Set<String> _contextSet) {
		// get the immediate container
		// check if that one is a "human concept", if not, I'say, move one up
		// and then generate a ref ex for that one
		String _bestTopoContainer = null;
		TreeSet<String> _potentiallyRuledOut = new TreeSet<String>();
		LinkedList<String> _currentTopoLevelQueue = new LinkedList<String>();
		_currentTopoLevelQueue.add(_intdRef);
		log("findBestValue ON Container called");

		while (_bestTopoContainer == null
				&& (!_currentTopoLevelQueue.isEmpty())) {
			String _currentTopoLevel = _currentTopoLevelQueue.removeFirst();
			for (String _vContainer : Arrays.asList(getReasoner()
					.getImmediateRelatedInstancesByRelation(_currentTopoLevel,
							parameters.getProperty("on")))) {
				// let's check whether we are actually still in the context!
				if (!_contextSet.contains(_vContainer))
					continue;

				// let's check whether the user knows that intd ref is included
				// there
				if (!userKnows(_intdRef, _vContainer))
					continue;

				// let's check whether the container has a
				// "human spatial concept"
				// if (getReasoner().getBasicLevelConcepts(_vContainer).size() <
				// 1)
				// continue;

				if (rulesOutOnContainer(_vContainer, _contrastSet).size() > _potentiallyRuledOut
						.size()) {
					_bestTopoContainer = _vContainer;
					_potentiallyRuledOut.clear();
					_potentiallyRuledOut
							.addAll((TreeSet<String>) rulesOutOnContainer(
									_vContainer, _contrastSet));
				}
				_currentTopoLevelQueue.add(_vContainer);
			}
			// now we have checked everything inside the current topological
			// layer
			if (_bestTopoContainer != null)
				break; // get out of the loop!
		}
		log("returning best ON container: " + _bestTopoContainer);
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
		log("called user knows: " + _ins + " " + _val
				+ " -- which is not implemented and always returns true");
		return true;
	}

	/**
	 * TODO make a sophisticated user model
	 * 
	 * @param _ins1
	 * @param _rel
	 * @param _ins2
	 * @return true
	 */
	public boolean userKnows(String _ins1, String _rel, String _ins2) {
		log("called user knows: " + _ins1 + " " + _rel + " " + _ins2
				+ " -- which is not implemented and always returns true");
		return true;
	}

	/**
	 * done.
	 * 
	 * @param _concept
	 * @param _contrastSet
	 * @return
	 */
	public Set<String> rulesOutCon(String _concept, Set<String> _contrastSet) {
		log("rules out concept called... current contrast set: " + _contrastSet
				+ "; given concept: " + _concept);
		TreeSet<String> retSet = new TreeSet<String>();
		if (_concept == null)
			return null;
		else {
			retSet.addAll((TreeSet) _contrastSet);
			log("ret set (copy of contrast set): " + stringsetToString(retSet));
			Set<String> toberemovedIns = new TreeSet<String>(Arrays.asList(getReasoner().getAllInstances(
					_concept)));
			log("instances of " + _concept + " are: "
					+ stringsetToString(toberemovedIns));
			retSet.removeAll(toberemovedIns);
			// log("ret set has size " + retSet.size());
		}
		log("returning ret set= " + stringsetToString(retSet));
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
	public Set<String> rulesOutName(String _name, Set<String> _contrastSet) {
		TreeSet<String> retSet = new TreeSet<String>();
		if (_name == null)
			return null;
		else {
			retSet.addAll((TreeSet) _contrastSet);
			retSet
					.removeAll(Arrays.asList(getReasoner()
							.getInstancesByPropertyValue(NAME_PROPERTY, _name)));
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
	public Set<String> rulesOutNumber(String _number, Set<String> _contrastSet) {
		TreeSet<String> retSet = new TreeSet<String>();
		if (_number == null)
			return null;
		else {
			retSet.addAll((TreeSet) _contrastSet);
			retSet.removeAll(Arrays.asList(getReasoner().getInstancesByPropertyValue(
					NUMBER_TAG_PROPERTY, _number)));
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
	public Set<String> rulesOutOwner(String _owner, Set<String> _contrastSet) {
		TreeSet<String> retSet = new TreeSet<String>();
		if (_owner == null)
			return null;
		else {
			retSet.addAll((TreeSet) _contrastSet);
			retSet.removeAll(Arrays.asList(getReasoner().getRelatedInstancesByRelation(
					_owner, parameters.getProperty("own"))));
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
	public Set<String> rulesOutTopoContainer(String _container,
			Set<String> _contrastSet) {
		TreeSet<String> retSet = new TreeSet<String>();
		if (_container == null)
			return null;
		else {
			retSet.addAll((TreeSet) _contrastSet);
			retSet.removeAll(Arrays.asList(getReasoner().getRelatedInstancesByRelation(
					_container,
					parameters.getProperty("topologicalDescendantRelation"))));
			log("rulesOutTopoContainer(" + _container + ") yields: " + retSet);
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
	public Set<String> rulesOutRelation(String _objIns, String _rel,
			Set<String> _contrastSet) {
		TreeSet<String> retSet = new TreeSet<String>();
		if ((_objIns == null) || (_rel == null))
			return null;
		else {
			for (String _contrastIns : _contrastSet) {
				if (!getReasoner().areInstancesRelated(_contrastIns, _rel, _objIns)) {
					retSet.add(_contrastIns);
				}
			}
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
	public Set<String> rulesOutInContainer(String _container,
			Set<String> _contrastSet) {
		TreeSet<String> retSet = new TreeSet<String>();
		if (_container == null)
			return null;
		else {
			retSet.addAll((TreeSet) _contrastSet);
			retSet.removeAll(Arrays.asList(getReasoner().getRelatedInstancesByRelation(
					_container,
					parameters.getProperty("topologicalDescendantRelation"))));
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
	public Set<String> rulesOutOnContainer(String _container,
			Set<String> _contrastSet) {
		TreeSet<String> retSet = new TreeSet<String>();
		if (_container == null)
			return null;
		else {
			retSet.addAll((TreeSet) _contrastSet);
			retSet.removeAll(Arrays.asList(getReasoner()
					.getInverseRelatedInstancesByRelation(_container,
							parameters.getProperty("on"))));
		}
		return retSet;
		// TODO user knows omitted!
	}

	public final ComaReasonerInterfacePrx getReasoner() {
		return m_parentReasoner;
	}

	public void log(String _msg) {
		if (m_logging)
			System.out.println("[GRE: " + _msg + "]");
		// m_logger.info(_msg);
	}

	public void setLogging(boolean _logging) {
		this.m_logging = _logging;
	}

	// private String topologicalParentRelation = "dora:topoIncluded";
	// private String topologicalDescendantRelation = "dora:topoIncludesTrans";
	// private String onRelation = "dora:on";
	// private String ownRelation = "dora:own";

	private String stringsetToString(Set<String> _stringset) {
		String returnString = "";

		for (String string : _stringset) {
			if (returnString.equals(""))
				returnString += (string);
			else
				returnString += (", " + string);
		}

		return returnString;
	}

	private String stringQToString(Queue<String> _stringset) {
		String returnString = "";

		for (String string : _stringset) {
			if (returnString.equals(""))
				returnString += (string);
			else
				returnString += (", " + string);
		}

		return returnString;
	}

	/*
	private Set<String> handelURI(Set<String> inSet) {
		// log("handelURI Set<String> called.");
		TreeSet<String> retSet = new TreeSet<String>();
		for (String uri : inSet) {
			// log("current URI = " + uri);
			String[] ret = uri.split("#");
			if (ret.length == 2) {
				// log("uri was split in 2: " + ret[0] + " + " + ret[1]);
				// log("adding the following to the return set: " +
				// parameters.getProperty(ret[0]) + ":" + ret[1]);
				retSet.add(parameters.getProperty(ret[0]) + ":" + ret[1]);
			} else {
				// log("uri was NOT split in 2: " + ret[0]);
				// log("adding the following to the return set: " + ret[0]);
				retSet.add(ret[0]);
			}
		}
		// log("exiting handelURI Set<String>.");
		return retSet;
	}

	private String handelURI(String uri) {
		// log("handelURI String called.");
		String[] ret = uri.split("#");
		if (ret.length == 2) {
			// log("uri was split in 2: " + ret[0] + " + " + ret[1]);
			// log("adding the following to the return set: " +
			// parameters.getProperty(ret[0]) + ":" + ret[1]);
			return parameters.getProperty(ret[0]) + ":" + ret[1];
		} else {
			// log("uri was NOT split in 2: " + ret[0]);
			// log("adding the following to the return set: " + ret[0]);
			return ret[0];
		}

	}
	*/

	//public void showGraph(int h, int v) {
	//	graph.update();
	//	graph.showABox(h, v);
	//}
	/*
	private TreeSet<String> oldcreateContextSet(String _intdRef, String _origin) {
		log(Integer.valueOf(getReasoner().getInstances("owl:Thing").size())
				.toString());
		Set<String> relatedInstances = handelURI(getReasoner()
				.getRelatedInstances(_origin));
		log("origin: " + _origin);
		for (String string : relatedInstances) {
			log(string);
		}
		log("---");
		Set<String> topoIncluders = handelURI(getReasoner()
				.getRelatedInstances(_origin,
						parameters.getProperty("topologicalParentRelation")));
		log("--- +++");
		for (String string : topoIncluders) {
			log("--" + string);
		}
		log("creating context set");
		// this method implements the creation of
		// a GRE context in large-scale space!
		TreeSet<String> _context = new TreeSet<String>();

		// 1st of all check whether the intended referent is contained in origin
		TreeSet<String> _allOriChildren = new TreeSet<String>();
		_allOriChildren.addAll(handelURI((TreeSet<String>) getReasoner()
				.getInverseRelatedInstances(_origin,
						parameters.getProperty("topologicalParentRelation"))));
		log("all ori children has size " + _allOriChildren.size());
		if (_allOriChildren.contains(_intdRef)) {
			_context.clear();
			_context.addAll(_allOriChildren);
			_context.add(_origin);
			log("all ori children contains int ref -> context is now all ori children.");
		}

		// starting from the origin, incrementally moving up one step at a time
		// then check whether the intended referent is a dependent node
		// if so, return all dependent nodes as the context set

		// start with the immediate containers of the origin
		TreeSet<String> _topoContainers = new TreeSet<String>();
		_topoContainers.addAll(handelURI((TreeSet<String>) getReasoner()
				.getImmediateRelatedInstances(_origin,
						parameters.getProperty("topologicalParentRelation"))));
		log("topoContainers has size " + _topoContainers.size());
		// this might be problematic in multiple inheritance / lattice cases
		// need some experiments first though
		while (_context.isEmpty()) {
			log("context still empty");
			// temporally store the immediate containers as potential new
			// origins
			// (when incrementally enlarging the context)

			// now I have to build the union of the daughter nodes of all
			// current containers
			// and then check whether they contain the intended referent
			// if so, then the context is found ;-)
			TreeSet<String> _allContainerChildren = new TreeSet<String>();
			TreeSet<String> _moreGeneralContainers = new TreeSet<String>();
			for (String instance : _topoContainers) {
				_allContainerChildren
						.addAll(((TreeSet<String>) getReasoner()
								.getInverseRelatedInstances(
										instance,
										parameters
												.getProperty("topologicalParentRelation"))));
				log("all container children has size "
						+ _allContainerChildren.size());
				_moreGeneralContainers
						.addAll(handelURI((TreeSet<String>) getReasoner()
								.getImmediateRelatedInstances(
										instance,
										parameters
												.getProperty("topologicalParentRelation"))));
			}
			if (_allContainerChildren.contains(_intdRef)) {
				_context.clear();
				_context.addAll(_allContainerChildren);
				_context.addAll(_topoContainers);
				log("all container children contains int ref -> context is now all container children + all containers.");
			}

			// TreeSet<ReasonerInstance> _moreGeneralContainers = new
			// TreeSet<ReasonerInstance>();
			log("got more general containers. it is a set of size "
					+ _moreGeneralContainers.size());
			// // go thru the currently known topological containers
			// for (ReasonerInstance instance : _topoContainers) {
			// log("still cycling thru all containers. current container: " +
			// instance.getName());
			// // get all children of the current container
			// TreeSet<ReasonerInstance> _allChildren =
			// (TreeSet<ReasonerInstance>)
			// getReasoner().getInverseRelatedInstances(instance,
			// OntologyMemberFactory.createRelation("oe", ":", "topoIncluded",
			// null, null));
			// // if the current container "dominates" the intended referent, we
			// should
			// // add all its children to the produced context set
			// if (_allChildren.contains(_intdRef))
			// _context.addAll(_allChildren);
			// // just in case we do not have a context, we should take
			// // into account the next level of abstraction as the containers
			// _moreGeneralContainers.addAll((TreeSet<ReasonerInstance>)
			// getReasoner().
			// getImmediateRelatedInstances(instance, OntologyMemberFactory.
			// createRelation("oe", ":", "topoIncluded", null, null)));
			// }

			// for the next round: move up one level of abstraction
			_topoContainers.clear();
			_topoContainers.addAll(_moreGeneralContainers);
		}
		log("done creating context set -- has size:" + _context.size());
		return _context;
	}
	*/

	//public void export(String path) {
	//	DotExport export = new DotExport(m_parentReasoner,new File(path),parameters);
	//	export.export();
	//}

}
