package coma.aux;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

import org.cognitivesystems.reasoner.base.OntologyMemberFactory;
import org.cognitivesystems.reasoner.base.ReasonerInstance;
import org.cognitivesystems.reasoner.base.ReasonerInterface;
import org.cognitivesystems.reasoner.crowl.CrowlWrapper;

import planning.autogen.Action;
import planning.autogen.PlanningStatus;

import com.hp.hpl.jena.query.QuerySolution;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.rdf.model.Resource;

import BindingData.BindingProxy;
import BindingFeatures.AreaID;
import BindingFeatures.Concept;
import BindingFeatures.Name;
import BindingFeatures.RelationLabel;
import BindingFeatures.TemporalFrame;
import BindingFeaturesCommon.TemporalFrameType;
import ComaData.GenerateComaProxies;
import binding.common.BindingComponentException;
import binding.util.BindingUtils;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.OperationMode;
import cast.cdl.TriBool;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.data.CASTData;

import coma.components.ComaReasoner;

public class ComaReferenceResolver {

	private ComaReasoner m_parentComponent;
	private WorkingMemoryChange m_actionWMC;

	public ComaReferenceResolver(ComaReasoner _parentReasoner, WorkingMemoryChange _actionWMC) {
		m_parentComponent = _parentReasoner;
		m_actionWMC = _actionWMC;
		log("new ReferenceResolver initialized for action " + _actionWMC.m_address.m_id);
	}
	
	private ReasonerInterface getReasoner() {
		return m_parentComponent.getReasoner();
	}
//	private String getBindingSA() {
//		return m_parentReasoner.getBindingSA();
//	}
	private void log(String _logMsg) {
		m_parentComponent.log("ReferenceResolver: " + _logMsg);
	}
	
	
	public void resolveAgainstProxy(WorkingMemoryAddress _proxyAddress) {
		log("resolveAgainstProxy("+_proxyAddress+") called");
		String _bsaID = _proxyAddress.m_subarchitecture;
		
		ReasonerInstance _discourseAnchor = OntologyMemberFactory.createInstance(
				m_parentComponent.getOntologyNS(), m_parentComponent.getOntologySep(),
				"world");
		boolean _anchorProperlySet = false;
		
		// resolve discourse anchor
		// if no anchor is specified, the robot's current location is assumed
		BindingProxy _robotSelfProxy = null;
		try {
			CASTData<BindingProxy>[] _allProxies =  
				m_parentComponent.getWorkingMemoryEntries(_bsaID, BindingProxy.class);
			
			for (CASTData<BindingProxy> data : _allProxies) {
				BindingProxy _currProxy = (BindingProxy) data.getData();
				
				// this needs to be changed if there is a consistent representation of SELF
				ArrayList<Concept> _allConFts = BindingUtils.getBindingFeatures(
						m_parentComponent, _bsaID, _currProxy, Concept.class);
				
				for (Concept _currConcept : _allConFts) {
					if (_currConcept.m_concept.equalsIgnoreCase("robot")) {
						_robotSelfProxy = _currProxy;
						break;
					}
				}
				
				if (_robotSelfProxy!=null) break;
			}
			
			// ok we now have the robot's self proxy
			// need to access its union to get to its position relatee
			ArrayList<BindingProxy> _allRobotRels = BindingUtils.getRelationsFromUnion(
					m_parentComponent, _bsaID, 
					BindingUtils.getUnion(m_parentComponent, _bsaID, _robotSelfProxy));
			
			for (BindingProxy _currRel : _allRobotRels) {
				ArrayList<RelationLabel> _allRelLabels = BindingUtils.getBindingFeatures(
						m_parentComponent, _bsaID, _currRel, RelationLabel.class);
				for (RelationLabel _currLabel : _allRelLabels) {
					if (_currLabel.m_label.equalsIgnoreCase("position")) {
						ArrayList<CASTData<BindingProxy>> _allPositionRelatees = BindingUtils.getProxiesViaToPort(
								m_parentComponent, _bsaID, _currRel);
						for (CASTData<BindingProxy> data : _allPositionRelatees) {
							BindingProxy _currRelatee = data.getData();
							
							// now check all possible features that identify a proxy that
							// has a coma instance counterpart
							
							// AreaID
							ArrayList<AreaID> _allAreaIDs = BindingUtils.getBindingFeatures(
									m_parentComponent, _bsaID, _currRelatee, AreaID.class);
							for (AreaID _currAreaID : _allAreaIDs) {
								String _areaInsName = "area" + _currAreaID;
								_anchorProperlySet = true;
								break;
							}
							
							
							if (_anchorProperlySet) break;
						}
						if (_anchorProperlySet) break;
					}
					if (_anchorProperlySet) break;
				}
				if (_anchorProperlySet) break;
			}
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			log("Cannot resolve dialogue anchor. Resolving against " + _discourseAnchor + " instead...");
			return;
		}
		// end: attempt to resovle discourse anchor
		log("discourse anchor = " + _discourseAnchor.getFullName());
		
		
		// now resolve referent
		
		TreeMap<String, String> _proxyIDToVarMapping = new TreeMap<String, String>(); 
		LinkedList<BindingProxy> _proxiesToBeChecked = new LinkedList<BindingProxy>();
		String _query = "";
		
		// read in head proxy
		// get concept=head-concept
		BindingProxy _headProxy = null;
		try {
			_headProxy = BindingUtils.getProxy(m_parentComponent, _bsaID, _proxyAddress.m_id);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			log("cannot resolve non existing proxy. Returning...");
			return;
		}
		
		int _varI = 0;
		String _currVar = "?x"+ _varI++;
		
		_proxiesToBeChecked.add(_headProxy);
		_proxyIDToVarMapping.put(_proxyAddress.m_id, _currVar);
		
		while (!_proxiesToBeChecked.isEmpty()) {
			BindingProxy _currProxy = _proxiesToBeChecked.removeFirst();

			// BLOCK FOR CHECKING CONCEPT FEATURE
			ArrayList<Concept> _currConcepts = null;
			try {
				_currConcepts = BindingUtils.getBindingFeatures(m_parentComponent, 
						_bsaID, _currProxy, Concept.class);
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				log("cannot resolve using non-existing concept feature.");
				if (_query.equals("")) {
					log ("cannot resolve if the referent's concept feature cannot be accessed! returning...");
					return;
				} else {
					log("cannot access the current proxy's concept. Ignoring...");
					continue;
				}
			}
			if (_currConcepts!=null && !_currConcepts.isEmpty()) { 
				for (Concept concept : _currConcepts) {
					// make sure we do not impose restrictions over
					// concepts that the ontology does not know about!
					if (getReasoner().conceptExists(OntologyMemberFactory.
							createConcept(m_parentComponent.getOntologyNS(),
									m_parentComponent.getOntologySep(), concept.m_concept)		
					)) {
						_query += _currVar + " rdf:type " + 
						(OntologyMemberFactory.
						createConcept(m_parentComponent.getOntologyNS(),
								m_parentComponent.getOntologySep(), concept.m_concept)).
								getFullName() 
								 + " . ";
						log("partial query so far: " + _query);
					} else {
						log("concept does not exist: " + concept.m_concept);
					}
				}
			} else if (_query.equals("")) {
				log("cannot resolve if the referent does not have any concept features! returning...");
				return;
			}
			
			// BLOCK FOR CHECKING OTHER FEATURES:
			// 1) NAME
			ArrayList<Name> _currNames = null;
			try {
				_currNames = BindingUtils.getBindingFeatures(m_parentComponent, 
						_bsaID, _currProxy, Name.class);
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				log("SA Process exception caught -- still trying to continue!");
				continue;
			}			
			if (_currNames!=null && !_currNames.isEmpty()) { 
				for (Name _name : _currNames) {
					// TODO: make this a disjunction instead of a conjunction!
					// TODO: ie it is sufficient if one of the names matches...
					_query += _currVar + " oe:name " + 
						"'" + _name.m_name + "' .";
					log("partial query so far: " + _query);
				}
			} 
						
			
			
			// CHECK RELATIONS ->  recursive call
			
			// load all relations + related proxies
			// get rel-labels-i++ rel-prox-concept-i++
			// and recursively follow all related proxies
			ArrayList<BindingProxy> _relations = null;
			try {
				_relations = BindingUtils.getRelations(m_parentComponent, _bsaID, _headProxy);
			} catch (BindingComponentException e) {
				e.printStackTrace();
				continue;
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				continue;
			}
			
			for (BindingProxy _relProxy : _relations) {
				try {
					TemporalFrame _frame =  BindingUtils.getBindingFeature(m_parentComponent, _bsaID, _relProxy,  TemporalFrame.class);
					if (_frame.m_temporalFrame.equals(TemporalFrameType.TYPICAL)) {
						log("encountered a TYPICAL temporal frame! skipping this relation...");
						continue;
					}
				} catch (SubarchitectureProcessException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
					log("I had a problem getting the temporal frame... that is not a problem... going on...");
				}
						
				try {
					ArrayList<CASTData<BindingProxy>> _fromProxiesData = BindingUtils.getProxiesViaFromPort(m_parentComponent, _bsaID, _relProxy);
					for (CASTData<BindingProxy> _fromProxyD : _fromProxiesData) {
						ArrayList<CASTData<BindingProxy>> _toProxiesData = BindingUtils.getProxiesViaToPort(m_parentComponent, _bsaID, _relProxy);
						for (CASTData<BindingProxy> _toProxyD : _toProxiesData) {
							ArrayList<RelationLabel> _currLabels = BindingUtils.getBindingFeatures(m_parentComponent, 
									_bsaID, _relProxy, RelationLabel.class);							
							for (RelationLabel _currLabel : _currLabels) {
								String _relLabel = _currLabel.m_label;
								if (_relLabel.equalsIgnoreCase("owner")) _relLabel="owns";
								if (_relLabel.equalsIgnoreCase("position")) _relLabel="topoIncluded";
								
								if (getReasoner().relationExists(OntologyMemberFactory.
										createRelation(m_parentComponent.getOntologyNS(),
												m_parentComponent.getOntologySep(), 
												_relLabel, null, null))) {
									log("_fromProxyD has type " + _fromProxyD.getType());
									// determine variable for subject
									if (_proxyIDToVarMapping.containsKey(_fromProxyD.getID())) {
										_query += _proxyIDToVarMapping.get(_fromProxyD.getID());								
									} else {
										_currVar = "?x"+ _varI++;
										_proxiesToBeChecked.add(_fromProxyD.getData());
										_proxyIDToVarMapping.put(_fromProxyD.getID(), _currVar);
										
										_query += _proxyIDToVarMapping.get(_fromProxyD.getID());									
									}
									
									// determine relation
									_query += " " + 
										(OntologyMemberFactory.
											createRelation(m_parentComponent.getOntologyNS(),
													m_parentComponent.getOntologySep(), 
													_relLabel, null, null)).getFullRelationName() +
													" ";
									
									// determine variable for subject
									if (_proxyIDToVarMapping.containsKey(_toProxyD.getID())) {
										_query += _proxyIDToVarMapping.get(_toProxyD.getID());								
									} else {
										_currVar = "?x"+ _varI++;
										_proxiesToBeChecked.add(_toProxyD.getData());
										_proxyIDToVarMapping.put(_toProxyD.getID(), _currVar);
										
										_query += _proxyIDToVarMapping.get(_toProxyD.getID());									
									}			
									
									_query += " . ";
									
									log("partial query after relations: " + _query);
									
								} else {
									log("relation does not exist: " + _relLabel);
								}
								
							}							
						}
					}
					// if a proxy is known:
					// just add its variable and the relation statement
					// if a proxy is unknown:
					// create new variable, add proxy to variable mapping,
					// add proxy to the queue of to visit proxies
					// inside that queue: create rdf:type concept restriction
					// and walk thru its relations...
					// :repeat: ;-)
					
				} catch (SubarchitectureProcessException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		} // end while proxy queue is non empty loop
		
		// finalize query
		String _queryPrefix = "SELECT ";
		for (String _var : _proxyIDToVarMapping.values()) {
			_queryPrefix+=_var + " ";
		}
		_query = _queryPrefix + " WHERE { " + _query + " }";
		
		log(_query);		

		// execute query
		ResultSet _results = ((CrowlWrapper) getReasoner()).executeSPARQLQuery(_query);

		ArrayList<WorkingMemoryPointer> _instanceWMPList = new ArrayList<WorkingMemoryPointer>();
		boolean _success = false;

		while (_results.hasNext()) {
			QuerySolution _qs = (QuerySolution) _results.next();
			log("current binding:");
			for (String _var : _proxyIDToVarMapping.values()) {
				
				Resource _currAnswerResource = _qs.getResource(_var);
				log(_var + " is bound to " + _currAnswerResource.toString());

				WorkingMemoryPointer _currInsWMP = null;
				try {
					_currInsWMP = m_parentComponent.getUptodateInstanceWMP(
							OntologyMemberFactory.createInstance(
									"oe", ":", 
									_currAnswerResource.getLocalName()));
				} catch (SubarchitectureProcessException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				_instanceWMPList.add(_currInsWMP);
			}
		}
		if (_instanceWMPList.isEmpty()) _success=false;
		else _success=true;
		
		if (_success) {
			// OK, now I should generate them proxies
			// TODO represent all involved entities and their relations				
			WorkingMemoryPointer[] _instanceWMPArray  = new WorkingMemoryPointer[_instanceWMPList.size()]; 
			int i=0;
			for (WorkingMemoryPointer pointer : _instanceWMPList) {
				_instanceWMPArray[i] = pointer;
				i++;
			}
				
			WorkingMemoryPointer[] _conceptWMPArray = new WorkingMemoryPointer[0];
			WorkingMemoryPointer[] _relationWMPArray = new WorkingMemoryPointer[0];
			String[] _defCons = new String[0];
			
			try {
				GenerateComaProxies _newProxyGenTask = new GenerateComaProxies(
						_instanceWMPArray, _conceptWMPArray, _relationWMPArray, _defCons);

				String _currID = m_parentComponent.newDataID();
				m_parentComponent.addToWorkingMemory(_currID, _newProxyGenTask);
				m_parentComponent.addChangeFilter(
						ChangeFilterFactory.createAddressFilter(_currID, 
								m_parentComponent.getSubarchitectureID(),
								WorkingMemoryOperation.DELETE),
								new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(
									WorkingMemoryChange _wmc) {
								proxiesAdded(_wmc);
							}});
				_success = true;
			} catch (SubarchitectureProcessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				_success=false;
			}
		}
		
		if (!_success) {
			Action _myAction;
			try {
				_myAction = (Action) m_parentComponent.getWorkingMemoryEntry(m_actionWMC.m_address).getData();
				_myAction.m_succeeded=TriBool.triFalse;
				m_parentComponent.overwriteWorkingMemory(m_actionWMC.m_address, 
						_myAction, OperationMode.BLOCKING);
			} catch (SubarchitectureProcessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
	
	private void proxiesAdded(WorkingMemoryChange _wmc) {
		log("all proxies created -- reporting back");
		Action _myAction;
		try {
			_myAction = (Action) m_parentComponent.getWorkingMemoryEntry(m_actionWMC.m_address).getData();
			_myAction.m_succeeded=TriBool.triTrue;
			_myAction.m_status=PlanningStatus.COMPLETE;
			m_parentComponent.overwriteWorkingMemory(m_actionWMC.m_address, 
					_myAction, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

}
