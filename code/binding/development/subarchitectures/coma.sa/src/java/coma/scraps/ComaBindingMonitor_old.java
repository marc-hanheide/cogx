package coma.scraps;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Properties;

import org.cognitivesystems.reasoner.base.OntologyMemberFactory;

import ComaData.ComaConcept;
import ComaData.ComaInstance;
import ComaData.ComaReasonerFunction;
import coma.aux.ComaFunctionWriter;
import coma.aux.ComaHelper;
import coma.aux.InstanceUnknownException;
import coma.aux.ComaFunctionWriter.ConceptQueryRestrictor;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import cast.core.data.CASTData;
import BindingData.BINDING_SUBARCH_CONFIG_KEY;
import BindingFeatures.AreaID;
import BindingFeatures.Concept;
import BindingFeatures.DebugString;
import BindingFeaturesCommon.TruthValue;
import binding.abstr.AbstractMonitor;
import binding.common.BindingComponentException;

/**
 * The new ComaBindingMonitor.
 * 
 * Right now this monitor waits for AreaID feature 
 * working memory changes.
 * TODO make sure only the binding WM is taken into account!
 * 
 * When a new AreaID is added, this monitor queries the ComaReasoner
 * for properties of the corresponding Area instance.
 * These properties include the following list:
 * the known most specific concepts of the instance
 * ??? relations ??? 
 * 
 * @author Hendrik Zender (zender@dfki.de)
 * @started 2008-07-07
 * @version 2008-07-07
 *
 */
public class ComaBindingMonitor_old extends AbstractMonitor {

	String m_bindingSA;
	
	private ComaFunctionWriter m_funcWriter;
	// the following 2 shld be inverse to each other
	private HashMap<String, String> m_myProxies2myAreas;
	private HashMap<String, String> m_myAreas2myProxies;
	private String m_ontologyNamespace;
	private String m_ontologySep;

	

	/**
	 * The standrad constructor...
	 * @param _id
	 */
	public ComaBindingMonitor_old(String _id) {
		super(_id);
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
	}
	
	/**
	 * Configure and initialize the members of this component. 
	 * @param _config
	 */
	public void configure(Properties _config) {
		super.configure(_config);
        if (_config.containsKey("--onto_ns")) {
            this.m_ontologyNamespace = _config.getProperty("--onto_ns");
        }
        else {
//            this.m_ontologyNamespace = "http://www.dfki.de/cosy/officeenv.owl";
            this.m_ontologyNamespace = "oe";
        }
        if (_config.containsKey("--onto_sep")) {
            this.m_ontologySep= _config.getProperty("--onto_sep");
        }
        else {
//            this.m_ontologySep = "#";
            this.m_ontologySep = "oe";
        }

		m_funcWriter = new ComaFunctionWriter(this);
		m_myProxies2myAreas = new HashMap<String, String>();
		m_myAreas2myProxies = new HashMap<String, String>();
		m_sourceID = m_subarchitectureID;

        if (_config.containsKey(BINDING_SUBARCH_CONFIG_KEY.value)) {
        	m_bindingSA = _config.getProperty(BINDING_SUBARCH_CONFIG_KEY.value);
        } else {
        	throw new RuntimeException("You need to specify the binding SA ID using the -bsa flag!");
        }
        log("done configuring.");
	}

	/**
	 * 
	 * The <i>start</i> method registers change filters with the working
	 * memory, checking for added or overwritten packed logical forms. 
	 *
	 * @see #handleWorkingMemoryChange(WorkingMemoryChange _wmc)
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	public void start() {
		super.start();
		try {
			// First, register change filters for all features I am
			// interested in!
			// 1) REGISTER AREAID FEATURE
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(AreaID.class, 
					WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(
						WorkingMemoryChange _wmc) {
					log("Spotted a new AreaID feature on binding WM!");
					handleNewAreaID(_wmc);
				}
			});
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		} // end try..catch
		log("done registering change filters.");
	} // end method  

	
	/**
	 * This method is invoked when a callback for a newly ADDed
	 * AreaID feature is received.
	 * 
	 * @param _wmc
	 */
	private void handleNewAreaID(WorkingMemoryChange _wmc) {
		log("I got a new AreaID feature!");
		try {
			// get the WME-ID of the function
			String dataId  = _wmc.m_address.m_id;
			String subArchId = _wmc.m_address.m_subarchitecture;

			// get the actual feature from working memory
			AreaID _areaIDft = (AreaID) getWorkingMemoryEntry(dataId, subArchId).getData();
			
			// test code!
			// registering a callback for deletion of this feature
			addChangeFilter(ChangeFilterFactory.createAddressFilter(_areaIDft.m_parent.m_immediateProxyID,
					m_bindingSA, WorkingMemoryOperation.DELETE),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(
						WorkingMemoryChange _wmc) {
					log("REGISTERED PROXY GOT DELETED!!!!");
					// handleNewAreaID(_wmc);
				}
			});

			
			
			int _areaID = _areaIDft.m_id;
			String _areaProxy = _areaIDft.m_parent.m_immediateProxyID;
			if (!m_myProxies2myAreas.containsKey(_areaProxy)) {
				log("This is someone else's proxy; proceeding...");
				// well, it's probably a nav.sa proxy
				// 1st get info about this area from the reasoner
				// what I could do is: fetch info about this instance from coma WM
				// and write it out to the binder
				// then pose a coma function for the reasoner to update the 
				// most specific concepts of the instance
				// maybe there should be a coma function "update instance info"
				// that checks if some concept info for the given instance
				// is still true (and if not, removes it from the concept list);
				// and then adds all other most specific concepts that are not yet present
				
				// anyway, pose a coma function for get the most specific concepts of
				// instance("area"+_areaID)
				// register a callback for this function
				try {
					m_funcWriter.getConcepts(
							new WorkingMemoryChangeReceiver() {
					    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					    	processGetMostSpecCons(_wmc);
					    }},
					    OntologyMemberFactory.createInstance("", "", "area"+_areaID), 
					    ConceptQueryRestrictor.MOSTSPECIFIC);
				} catch (InstanceUnknownException e) {
					log("All known instances should be in COMA WM. So obviously there was an error!");
					throw new RuntimeException(e);
				}
				log("Issued function call: get most specific concepts for instance area"+_areaID);
			} else {
				log("This is my own proxy; exiting...");
				return;
			}
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private void processGetMostSpecCons(WorkingMemoryChange _wmc) {
		log("Got a callback for a get most specific concepts coma function.");
		// when this callback is received, create my proxy
		// features: AreaID
		// 			 Concept (may have more than one!
		// add to the list of my own proxies.
		// if proxy already exists: update my proxy
		// then register a callback for DELETion of the 
		// other SA's proxy; remove my proxy if other proxy
		// is deleted... maybe check if there are still others 
		// in that union...
		// heck, this is exactly the question: when to delete proxies...

		// foreplay: get the changed WME!
		CASTData<?> wme = null;
		ComaReasonerFunction _comaRsnrFn;
		try {
			wme = getWorkingMemoryEntry(_wmc.m_address);
		} catch (SubarchitectureProcessException e) {
			log("I registered a callback for this WME. Now it's gone. That's bad! Aborting!");
			throw new RuntimeException(e);
		}
		_comaRsnrFn= (ComaReasonerFunction) wme.getData();

		ComaConcept[] _fnResult;
		try {
			_fnResult = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException("Error: Couldn't read the result of the function! Aborting!");
		}
		
    	WorkingMemoryPointer _arg1ptr = _comaRsnrFn.m_arg1ptr;
    	ComaInstance _areaInstance = null;
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) {
    		try {
    			_areaInstance = (ComaInstance) getWorkingMemoryEntry(_arg1ptr.m_address).getData();
    		} catch (SubarchitectureProcessException e) {
    			e.printStackTrace();
    			throw new RuntimeException("Error: Couldn't read the original area instance! Aborting!");
    		}
    	} else {
    		log("_arg1ptr is not a ComaInstance. Some error must have occurred. Aborting!");
    		throw new RuntimeException();
    	}
		log("Got a result set of "+_fnResult.length+" most specific concepts for instance " + _areaInstance.m_name);

		// hmm we can probably delete temporary entries from WM
		m_funcWriter.cleanUpAfterFunctionEvaluated(_wmc.m_address);

		// now that we're here we can safely delete the concept WMEs
		log("not deleting concept and instance WMEs from WM! --> to do: Garbage Collection");
//		deleteFromWorkingMemory(_comaRsnrFn.m_arg1ptr.m_address.m_id,_comaRsnrFn.m_arg1ptr.m_address.m_subarchitecture);
//		deleteFromWorkingMemory(_comaRsnrFn.m_arg2ptr.m_address.m_id,_comaRsnrFn.m_arg2ptr.m_address.m_subarchitecture);

		// flag for tracking whether re-binding is needed or not
		boolean _bindingNecessary = false;
		// now manage my own proxy
		log("now take care of proxy creation / updating:");
		if (m_myAreas2myProxies.containsKey(_areaInstance.m_name)) {
			log("I already have a proxy, so let's update it!");
			// I already have a proxy, so let's update it!
			// Trust the latest results, and distrust old knowldge:
			// overwrite the concept information with the most
			// recent list of the most specific concepts.

			HashSet<String> _excludeFeatures = new HashSet<String>();
			_excludeFeatures.add(CASTUtils.typeName(Concept.class));
			_excludeFeatures.add(CASTUtils.typeName(AreaID.class));
			try {
				changeExistingProxy(m_myAreas2myProxies.get(_areaInstance.m_name), _excludeFeatures);
				log("Changing existing proxy and deleting existing concept and area ID features from the proxy.");
			} catch (BindingComponentException e) {
				throw new RuntimeException(e);
			} catch (SubarchitectureProcessException e) {
				throw new RuntimeException(e);
			}

			for (ComaConcept _currCon: _fnResult) {
				Concept _concpt = new Concept();
				_concpt.m_concept=ComaHelper.firstToLower(_currCon.m_name);
				try {					
					addFeatureToCurrentProxy(_concpt);
				} catch (BindingComponentException e) {
					throw new RuntimeException(e);
				} catch (SubarchitectureProcessException e) {
					throw new RuntimeException(e);
				}
				log("Added new concept feature: " + _concpt.m_concept);
			}
			
			if (_areaInstance.m_name.startsWith("area")) {
				AreaID _aIDft = new AreaID();
				_aIDft.m_id = Integer.parseInt(_areaInstance.m_name.replace("area", ""));
				try {
					addFeatureToCurrentProxy(_aIDft);
				} catch (BindingComponentException e) {
					throw new RuntimeException(e);
				} catch (SubarchitectureProcessException e) {
					throw new RuntimeException(e);
				}
				log("Added area ID feature to proxy: "+_aIDft.m_id);
			}

			try {
				storeCurrentProxy();
			} catch (SubarchitectureProcessException e) {
				throw new RuntimeException(e);
			}
			log("Stored current proxy.");
			_bindingNecessary = true;
		} // end "do we already have a proxy for the instance?"
		else { 
			// so we don't have a proxy for this instance, let's create one!
			log("I do not have a proxy for this area. Starting a new proxy.");
			try {
				startNewBasicProxy();
			} catch (BindingComponentException e) {
				throw new RuntimeException(e);
			} catch (SubarchitectureProcessException e) {
				throw new RuntimeException(e);
			}

			for (ComaConcept _currCon: _fnResult) {
				Concept _concpt = new Concept();
				_concpt.m_concept=ComaHelper.firstToLower(_currCon.m_name);
				try {
					addFeatureToCurrentProxy(_concpt);
				} catch (BindingComponentException e) {
					throw new RuntimeException(e);
				} catch (SubarchitectureProcessException e) {
					throw new RuntimeException(e);
				}
				log("Added new concept feature: " + _concpt.m_concept);
			}

			if (_areaInstance.m_name.startsWith("area")) {
				AreaID _aIDft = new AreaID();
				_aIDft.m_id = Integer.parseInt(_areaInstance.m_name.replace("area", ""));
				try {
					addFeatureToCurrentProxy(_aIDft);
				} catch (BindingComponentException e) {
					throw new RuntimeException(e);
				} catch (SubarchitectureProcessException e) {
					throw new RuntimeException(e);
				}
				log("Added area ID feature to proxy: "+_aIDft.m_id);
			}


			DebugString _dbgStr = new DebugString();
			_dbgStr.m_debugString="coma_instance_id:" + _areaInstance.m_name;
 			/*
			try {
				addOtherSourceIDToCurrentProxy(m_subarchitectureID, TruthValue.POSITIVE);
			} catch (BindingComponentException e) {
				throw new RuntimeException(e);
			} catch (SubarchitectureProcessException e) {
				throw new RuntimeException(e);
			}
			*/
			String newProxy;
			try {
				newProxy = storeCurrentProxy();
			} catch (SubarchitectureProcessException e) {
				throw new RuntimeException(e);
			}
			_bindingNecessary = true;

			log("performed changes -> storeCurrentProxy!");
			// now store the mappings between other proxies and our instance proxies
			m_myAreas2myProxies.put(_areaInstance.m_name, newProxy);
			m_myProxies2myAreas.put(newProxy, _areaInstance.m_name);
		}
	// only now trigger binding of the new/changed proxies!	
	if (_bindingNecessary) {
		log("we have performed changes involving proxies -> bindNewProxies()");
		try {
			bindNewProxies();
		} catch (SubarchitectureProcessException e) {
			throw new RuntimeException(e);
		}	
	}
	
	}
	
	
	@Override
	protected void taskAdopted(String _taskID) {
		// TODO Auto-generated method stub

	}

	@Override
	protected void taskRejected(String _taskID) {
		// TODO Auto-generated method stub

	}

}
