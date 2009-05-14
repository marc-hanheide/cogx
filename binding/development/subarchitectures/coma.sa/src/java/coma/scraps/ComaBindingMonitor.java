package coma.scraps;

import java.util.HashSet;
import java.util.Properties;

import org.cognitivesystems.reasoner.base.ReasonerConcept;
import org.cognitivesystems.reasoner.base.ReasonerInstance;
import org.cognitivesystems.reasoner.base.OntologyMemberFactory;

import ComaData.ComaConcept;
import ComaData.ComaInstance;
import ComaData.ComaReasonerFunction;
import ComaData.ComaReasonerFunctionType;
import ComaData.ComaRelation;
import ComaData.TriBoolResult;
import coma.aux.ComaHelper;
import coma.aux.InstanceUnknownException;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.OperationMode;
import cast.cdl.TriBool;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import cast.core.data.CASTData;
import BindingData.BindingProxy;
import BindingData.FeaturePointer;
import BindingFeatures.Concept;
import BindingFeatures.DebugString;
import BindingFeaturesCommon.TruthValue;
import binding.abstr.AbstractMonitor;

public class ComaBindingMonitor extends AbstractMonitor {

	private enum ConceptQueryRestrictor {
		ALL,
		DIRECT,
		MOSTSPECIFIC,
		BASICLEVEL
	}
	
	private String m_ontologyNamespace;
	private String m_ontologySep;


	private HashSet<String> m_interestingProxies; 

	public ComaBindingMonitor(String _id) {
		super(_id);
		// Do I really need to set the ontology here?
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
	}

	

	/** 
    The <i>start</i> method registers change filters with the working
	 memory, checking for added or overwritten packed logical forms. 

	 @see #handleWorkingMemoryChange(WorkingMemoryChange _wmc)
	 @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	@Override
	public void start() {
		super.start();
		try {
			// First, register change filters for all features I am
			// interested in!
			// They will all trigger the same method, which marks the
			// corresponding proxy as "interesting".
			// 1) REGISTER CONCEPT FEATURE
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Concept.class, 
					WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(
						WorkingMemoryChange _wmc) {
					log("Spotted a new Concept feature on binding WM!");
					handleNewRelevantFeature(_wmc);
				}
			});
			// TODO 2) REGISTER AREAID FEATURE! 

			// now register a change filter for all proxies
			// in the invoked method, we check if the proxy is "interesting"
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Concept.class, 
					WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(
						WorkingMemoryChange _wmc) {
					log("Someone has added a new proxy to the binder...");
					handleNewProxy(_wmc);
				}
			});

		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		} // end try..catch
	} // end method  



	private void handleNewRelevantFeature(WorkingMemoryChange _wmc) {

		try {
			// get the WME-ID of the function
			String dataId  = _wmc.m_address.m_id;
			String subArchId = _wmc.m_address.m_subarchitecture;

			// get the actual feature from working memory
			CASTData data = getWorkingMemoryEntry(dataId, subArchId);
			String ftrType = data.getType();

			// we have to have one clause for each relevant feature (see above)
			// 1) CONCEPT FEATURE
			if (ftrType.equals(CASTUtils.typeName(Concept.class))) {
				Concept conceptFtr = (Concept) data.getData();
				if (existsOnWorkingMemory(_wmc.m_address)) {
					m_interestingProxies.add(conceptFtr.m_parent.m_immediateProxyID);
				}
			}
			// TODO add clause for AreaID feature!


			// we do not have to do any further processing,
			// the rest will be handled when we get the "complete" proxy
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	private void handleNewProxy(WorkingMemoryChange _wmc) {
		try {
			// get the WME-ID of the proxy
			String dataId  = _wmc.m_address.m_id;
			String subArchId = _wmc.m_address.m_subarchitecture;

			// we only have to care about "interesting proxies"...
			// once we're here, we remove the proxy ID from the list
			// in order to keep it small
			if (!m_interestingProxies.remove(dataId)) {
				// this proxy is not interesting: exiting...
				return;
			}
			// ok, now that we're here, this means
			// that we are dealing with an interesting proxy

			// get the actual proxy from working memory
			CASTData data = getWorkingMemoryEntry(dataId, subArchId);
			BindingProxy proxy = (BindingProxy) data.getData();

			Concept proxyConcept = null;
			Object proxyAreaID = null; 
			// TODO refactor Object to actual datatype (also occurrences below!)
			for (FeaturePointer _ftpointer : proxy.m_proxyFeatures) {
				// one clause for each relevant feature
				// 1) CONCEPT
				if (_ftpointer.m_type.equals(CASTUtils.typeName(Concept.class))) {
					proxyConcept = (Concept) getWorkingMemoryEntry(_ftpointer.m_address, subArchId).getData();
				}
				// TODO 2) AREAID
			}

			// now we should have all information we need to
			// get back to the conceptual map

			// if AreaID is specified, ie !=null, we know that it is an area...
			if (proxyAreaID!=null) {
				// check whether we already know about the current Area
				// -> check if there is a ComaInstance on coma wm that has
				// as name "areaN" where N=proxyAreaID
				WorkingMemoryPointer instanceWMP = getInstanceWME(OntologyMemberFactory.createInstance(m_ontologyNamespace, m_ontologySep,"area"+proxyAreaID));
				if (instanceWMP!=null) {
					// if yes, we should probably check if nav.sa has presented
					// a new concept (area classification, that is)
					// TODO do something here...
				} else {
					// if not, we add it to the conceptual map
					// -> create a ComaInstance with the given area ID and concept
					// --> create a ComaReasonerFunction to add this instance to the Abox.
					addInstanceConceptAssertionToComa(OntologyMemberFactory.createInstance(m_ontologyNamespace, m_ontologySep,"area"+proxyAreaID), 
							OntologyMemberFactory.createConcept("",  "", proxyConcept.m_concept));
				}

				// in any case, we should then add our own proxy!
				// TODO how to deal with inferred knowledge from the reasoner,
				// say when we have a new most specific concept available?
				// should we write that back to coma WM?
				// short answer: when we create our own proxy, 
				// we check back with the reasoner...
				try {
					getConceptsForCreatingProxy(OntologyMemberFactory.createInstance("", "", "area"+proxyAreaID), ConceptQueryRestrictor.MOSTSPECIFIC, _wmc.m_address);
				} catch (InstanceUnknownException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}


			}


		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}




	/**
	 * This method adds a ComaInstance WME and a ComaConcept WME to Coma WM.
	 * It also writes a ComaReasonerFunction of type AddInstance that points
	 * to the instance and concept WMEs.
	 * It waits for the ComaReasoner to process the function and then
	 * deletes the function and the result TriBool WME!
	 * 
	 * @param _instance
	 * @param _concept
	 */
	private void addInstanceConceptAssertionToComa(ReasonerInstance _instance, ReasonerConcept _concept) {

		try {
			WorkingMemoryPointer _arg1ptr = getInstanceWME(_instance);
			if (_arg1ptr==null) {
				String _insDataID = newDataID();
				addToWorkingMemory(_insDataID, new ComaInstance(_instance.getNamespace(),_instance.getSep(),_instance.getName(),new String[0], new String[0]), OperationMode.BLOCKING);
				_arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaInstance.class),
						new WorkingMemoryAddress(_insDataID, m_subarchitectureID));
			}

			WorkingMemoryPointer _arg2ptr = getConceptWME(_concept);
			if (_arg2ptr==null) {
				String _conDataID = newDataID();
				addToWorkingMemory(_conDataID, new ComaConcept(_concept.getNamespace(),_concept.getSep(),_concept.getName()), OperationMode.BLOCKING);
				_arg2ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
						new WorkingMemoryAddress(_conDataID, m_subarchitectureID));
			}

			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.AddInstance,_arg1ptr,
					_arg2ptr,new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));
			String _reqID = newDataID();

			// add change filter for my WME before(!) actually putting it on WM
			addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, 
					WorkingMemoryOperation.OVERWRITE), 
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					// log(CASTUtils.toString(_wmc));
					handleAddAssertionFunctionChanged(_wmc);
				}
			});
			addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);

		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/**
	 * TODO adapt this text!
	 * This method will query the ComaReasoner for all concepts of a given instance.
	 * 
	 * It first gets the WME for the instance in question. If the coma WM
	 * does not contain such an instance, an exception is thrown.
	 * If, however, a suitable instance is found, this method puts a new
	 * ComaReasonerFunction of type GetAllConcepts onto the coma WM,
	 * and registers an appropriate filter for the result callback.
	 *
	 * @param _ins
	 * @param _quantification
	 * @param _otherProxyAdress stores the ID of the nav.sa proxy
	 * @throws InstanceUnknownException
	 */
	private void getConceptsForCreatingProxy(ReasonerInstance _ins, ConceptQueryRestrictor _quantification, final WorkingMemoryAddress _otherProxyAddress) throws InstanceUnknownException{

		try {
			WorkingMemoryPointer _insptr = getInstanceWME(_ins);
			if (_insptr==null) {
				throw new InstanceUnknownException(_ins.getFullName());
			}
			ComaReasonerFunction _req = new ComaReasonerFunction(
					(_quantification.equals(ConceptQueryRestrictor.ALL) ? ComaReasonerFunctionType.GetAllConcepts
							: (_quantification.equals(ConceptQueryRestrictor.DIRECT) ? ComaReasonerFunctionType.GetAllDirectConcepts
									: (_quantification.equals(ConceptQueryRestrictor.MOSTSPECIFIC) ? ComaReasonerFunctionType.GetMostSpecificConcepts
											: (_quantification.equals(ConceptQueryRestrictor.BASICLEVEL) ? ComaReasonerFunctionType.GetBasicLevelConcepts
													: ComaReasonerFunctionType.GetAllConcepts)))), // default fall-back...
													_insptr,
													new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
													new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
													new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));

			// add change filter before the actual WME!
			String _reqID = newDataID();
			addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, WorkingMemoryOperation.OVERWRITE), 
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					// log(CASTUtils.toString(_wmc));
					handleCallbackForCreatingProxy(_wmc, _otherProxyAddress);
				}
			});
			addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);

		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}


	/**
	 * This method tests whether a given instance already exists on WM.
	 * If so, it returns the corresponding WME.
	 * Otherwise, it returns null.
	 *
	 * @param _ins
	 * @return
	 */
	private WorkingMemoryPointer getInstanceWME(ReasonerInstance _ins) {
		try {
			for (CASTData<?> _currDataIns : getWorkingMemoryEntries(m_subarchitectureID, ComaInstance.class)) {
				if (((((ComaInstance) _currDataIns.getData()).m_namespace).equals(_ins.getNamespace())) 
					&& ((((ComaInstance) _currDataIns.getData()).m_sep).equals(_ins.getSep()))
					&& ((((ComaInstance) _currDataIns.getData()).m_name).equals(_ins.getName()))) {
					return new WorkingMemoryPointer(_currDataIns.getType(),
							new WorkingMemoryAddress(_currDataIns.getID(), m_subarchitectureID));
				}
			}
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}
			
    
	/**
	 * This method tests whether a given concept already exists on WM.
	 * If so, it returns the corresponding WME.
	 * Otherwise, it returns null.
	 *
	 * @param _con
	 * @return
	 */
	private WorkingMemoryPointer getConceptWME(ReasonerConcept _con) {
		try {
			for (CASTData<?> _currDataCon : getWorkingMemoryEntries(m_subarchitectureID, ComaConcept.class)) {
				if (((((ComaConcept) _currDataCon.getData()).m_namespace).equals(_con.getNamespace())) 
						&& ((((ComaConcept) _currDataCon.getData()).m_sep).equals(_con.getSep()))
						&& ((((ComaConcept) _currDataCon.getData()).m_name).equals(_con.getName()))) {
					return new WorkingMemoryPointer(_currDataCon.getType(),
							new WorkingMemoryAddress(_currDataCon.getID(), m_subarchitectureID));
				}
			}
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	
	private void handleCallbackForCreatingProxy(WorkingMemoryChange _wmc, WorkingMemoryAddress _otherProxyAddress) {
		try {
			// get the WME-ID of the function
			String dataId  = _wmc.m_address.m_id;
			String subArchId = _wmc.m_address.m_subarchitecture;

			// get the actual function from working memory
			CASTData data;
			data = getWorkingMemoryEntry(dataId, subArchId);
			ComaReasonerFunction _comaRsnrFn = (ComaReasonerFunction) data.getData();

			ComaInstance fnArgIns;

			fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
			ComaConcept[] mostSpecCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());

			deleteFromWorkingMemory(_wmc.m_address.m_id);

			if (!_comaRsnrFn.m_add_info_ptr.m_address.m_id.equals("")) {
				//	log("deleting add_info at address "+_comaRsnrFn.m_add_info_ptr.m_address.m_id);
				deleteFromWorkingMemory(_comaRsnrFn.m_add_info_ptr.m_address.m_id); 
			}
			if (!_comaRsnrFn.m_resultptr.m_address.m_id.equals("")) {
				//	log("deleting result at address "+_comaRsnrFn.m_resultptr.m_address.m_id);
				deleteFromWorkingMemory(_comaRsnrFn.m_resultptr.m_address.m_id); 
			}


			// NOW CREATE OUR OWN PROXY:
			log("startNewBasicProxy!");
			startNewBasicProxy();
			boolean _bindingNecessary = false;

//			for (String _currConString : instance.m_mostSpecificConcepts) {
			for (ComaConcept _concept : mostSpecCons) {
				Concept _concpt = new Concept();
				_concpt.m_concept=_concept.m_name.toLowerCase();
				addFeatureToCurrentProxy(_concpt);
			}
//			addFeatureToCurrentProxy(new DebugString("coma_instance_id:" + instance.m_shortname, false, ""));
			DebugString _dbgStr = new DebugString();
			_dbgStr.m_debugString="coma_instance_id:" + fnArgIns.m_namespace+fnArgIns.m_sep+fnArgIns.m_name;
			addOtherSourceIDToCurrentProxy(m_subarchitectureID, TruthValue.POSITIVE);
			// TODO What is the line above doing?
			final String newProxy = storeCurrentProxy();
			_bindingNecessary = true;

			if (existsOnWorkingMemory(_otherProxyAddress)) {
				addChangeFilter(ChangeFilterFactory.createAddressFilter(_otherProxyAddress, 
						WorkingMemoryOperation.DELETE), 
						new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						// log(CASTUtils.toString(_wmc));
						handleOtherProxyDeleted(_wmc, newProxy);
					}
				});
			} else {
				deleteExistingProxy(newProxy);
				_bindingNecessary = false;
			}

//			only now trigger binding of the new/changed proxies!	
			if (_bindingNecessary) {
				bindNewProxies();	
			}
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	
	public void handleOtherProxyDeleted(WorkingMemoryChange _wmc, String _myProxyID) {
//		WorkingMemoryAddress deletedWMA  = _wmc.m_address;
		try {
			deleteExistingProxy(_myProxyID);
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
		

	private void handleAddAssertionFunctionChanged(WorkingMemoryChange _wmc) {
		try {
			// get the WME-ID of the function
			String dataId  = _wmc.m_address.m_id;
			String subArchId = _wmc.m_address.m_subarchitecture;

			// get the actual function from working memory
			CASTData data;
			data = getWorkingMemoryEntry(dataId, subArchId);
			ComaReasonerFunction _comaRsnrFn = (ComaReasonerFunction) data.getData();

			TriBool _boolResult;
			ComaInstance fnArgIns;
			
			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
			log("ComaReasoner returned for AddInstance: "+ ComaHelper.triBool2String(_boolResult));
			// TODO we might want to catch cases where we get an unusual answer: TriFalse!

			deleteFromWorkingMemory(_wmc.m_address.m_id);
			if (!_comaRsnrFn.m_add_info_ptr.m_address.m_id.equals("")) {
				deleteFromWorkingMemory(_comaRsnrFn.m_add_info_ptr.m_address.m_id); 
			}
			if (!_comaRsnrFn.m_resultptr.m_address.m_id.equals("")) {
				deleteFromWorkingMemory(_comaRsnrFn.m_resultptr.m_address.m_id); 
			}
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
	
	
	
	



	@Override
	public void configure(Properties _config) {
		super.configure(_config);
		m_interestingProxies = new HashSet<String>();
		if (_config.containsKey("--onto_ns")) {
			this.m_ontologyNamespace = _config.getProperty("--onto_ns");
		}
		else {
			this.m_ontologyNamespace = "http://www.dfki.de/cosy/officeenv.owl";
		}
		if (_config.containsKey("--onto_sep")) {
			this.m_ontologySep= _config.getProperty("--onto_sep");
		}
		else {
			this.m_ontologySep = "#";
		}
		log("Using namespace: " + m_ontologyNamespace);
		log("Using separator: " + m_ontologySep);
	}




	@Override
	protected void taskAdopted(String _taskID) {
		// TODO Auto-generated method stub

	}

	@Override
	protected void taskRejected(String _taskID) {
		// TODO Auto-generated method stub

	}

/////////////////////////////////////////////////////////////////
	// THIS IS JUST A CODE FLEA MARKET!!!
	private void _handleComaReasonerFunctionChanged(WorkingMemoryChange _wmc) {
		//log("got a callback: the comaReasoner has evaluated a function!");
		
		try {
			// get the WME-ID of the function
			String dataId  = _wmc.m_address.m_id;
			String subArchId = _wmc.m_address.m_subarchitecture;
			
			// get the actual function from working memory
			CASTData data;
			data = getWorkingMemoryEntry(dataId, subArchId);
			ComaReasonerFunction _comaRsnrFn = (ComaReasonerFunction) data.getData();
			
			TriBool _boolResult;
			ComaInstance fnArgIns;
			
			// specify behavior for certain function types
			switch (_comaRsnrFn.m_functiontype.value()) {
//			case ComaReasonerFunctionType._AreConsEquivalent:
//			break;
//			case ComaReasonerFunctionType._IsConSubcon:
//			break;
//			case ComaReasonerFunctionType._IsConSupercon:
//			break;
			case ComaReasonerFunctionType._GetAllConcepts:
				fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
				ComaConcept[] allCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
				log("ComaReasoner returned a set of "+allCons.length +" concepts for GetAllConcepts for "+fnArgIns.m_name);
				for (ComaConcept _concept : allCons) {
					log(_concept.m_namespace+_concept.m_sep+_concept.m_name);
				}
				break;
			case ComaReasonerFunctionType._GetAllDirectConcepts:
				fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
				ComaConcept[] allDirectCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
				log("ComaReasoner returned a set of "+allDirectCons.length+" concepts for GetAllDirectConcepts for "+fnArgIns.m_name);
				for (ComaConcept _concept : allDirectCons) {
					log(_concept.m_namespace+_concept.m_sep+_concept.m_name);
				}
				break;
			case ComaReasonerFunctionType._GetMostSpecificConcepts:
				fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
				ComaConcept[] mostSpecCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
				log("ComaReasoner returned a set of "+mostSpecCons.length+" concepts for GetMostSpecificConcepts for "+fnArgIns.m_name);
				for (ComaConcept _concept : mostSpecCons) {
					log(_concept.m_namespace+_concept.m_sep+_concept.m_name);
				}
				break;
			case ComaReasonerFunctionType._GetAllInstances:
				ComaConcept _fnArgCon = ((ComaConcept) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
				ComaInstance[] allIns = ((ComaInstance[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
				log("ComaReasoner returned a set of "+allIns.length+" instances for GetAllInstances for "+_fnArgCon.m_name);
				for (ComaInstance _ins: allIns) {
					log(_ins.m_namespace+_ins.m_sep+_ins.m_name);
				}     			
				break;
			case ComaReasonerFunctionType._GetRelatedInstances:
				// TODO check for polymorphic uses!!!!
				ComaInstance _fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
				ComaInstance[] _relIns = ((ComaInstance[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
				if (!_comaRsnrFn.m_add_info_ptr.m_address.m_id.equals("")) {
					ComaRelation _fnArgRel = ((ComaRelation) getWorkingMemoryEntry(_comaRsnrFn.m_add_info_ptr.m_address).getData());
					log("ComaReasoner returned a set of "+_relIns.length+" instances for GetRelatedInstances for "+_fnArgIns.m_name +" and "+_fnArgRel.m_name);
				}
				else {
					log("ComaReasoner returned a set of "+_relIns.length+" instances for GetRelatedInstances for "+_fnArgIns.m_name);
				}
				for (ComaInstance _ins: _relIns) {
					log(_ins.m_namespace+_ins.m_sep+_ins.m_name);
				}     			
				break;
			case ComaReasonerFunctionType._GetBasicLevelConcepts:
				fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
				ComaConcept[] basicLvlCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
				log("ComaReasoner returned a set of "+basicLvlCons.length+" concepts for GetBasicLevelConcepts for "+fnArgIns.m_name);
				for (ComaConcept _concept : basicLvlCons) {
					log(_concept.m_namespace+_concept.m_sep+_concept.m_name);
				}
				break;
//				case ComaReasonerFunctionType._CompareCons:
//				break;
			case ComaReasonerFunctionType._GetObjectMobility:
				break;
			case ComaReasonerFunctionType._GetTypicalObjects:
				break;
			case ComaReasonerFunctionType._AddInstance:
				_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
				log("ComaReasoner returned for AddInstance: "+ ComaHelper.triBool2String(_boolResult));
				break;
			case ComaReasonerFunctionType._AddRelation:
				_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
				log("ComaReasoner returned for AddRelation: "+ ComaHelper.triBool2String(_boolResult));
				break;
			default:
				log("Unknown action type");
			break;
			}
			
			
			// hmm we can probably delete temporary entries from WM
			// log("deleting original COMA REASONER FUNCTION from WM.");
			deleteFromWorkingMemory(_wmc.m_address.m_id);
			
			/* 
        	// now that we're here we can safely delete the concept WMEs
        	log("deleting obsolete working memory entries from WM!");

        	if (!_comaRsnrFn.m_arg1ptr.m_address.m_id.equals("")) {
        		log("deleting arg1 at address "+_comaRsnrFn.m_arg1ptr.m_address.m_id);
        		deleteFromWorkingMemory(_comaRsnrFn.m_arg1ptr.m_address.m_id);
        	}
        	if (!_comaRsnrFn.m_arg2ptr.m_address.m_id.equals("")) {
        		log("deleting arg2 at address "+_comaRsnrFn.m_arg2ptr.m_address.m_id);
        		deleteFromWorkingMemory(_comaRsnrFn.m_arg2ptr.m_address.m_id);
        	}
			 */
			if (!_comaRsnrFn.m_add_info_ptr.m_address.m_id.equals("")) {
				//	log("deleting add_info at address "+_comaRsnrFn.m_add_info_ptr.m_address.m_id);
				deleteFromWorkingMemory(_comaRsnrFn.m_add_info_ptr.m_address.m_id); 
			}
			if (!_comaRsnrFn.m_resultptr.m_address.m_id.equals("")) {
				//	log("deleting result at address "+_comaRsnrFn.m_resultptr.m_address.m_id);
				deleteFromWorkingMemory(_comaRsnrFn.m_resultptr.m_address.m_id); 
			}
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
}
