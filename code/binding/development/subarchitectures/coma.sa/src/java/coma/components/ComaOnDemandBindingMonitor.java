package coma.components;

import java.util.HashMap;
import java.util.Properties;

import motivation.util.CompetenceRegistration;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.AlreadyExistsOnWMException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;

import BindingData.BINDING_SUBARCH_CONFIG_KEY;
import BindingFeatures.AreaID;
import BindingFeatures.Concept;
import BindingFeatures.DebugString;
import BindingFeatures.Name;
import BindingFeaturesCommon.TemporalFrameType;
import ComaData.ComaConcept;
import ComaData.ComaInstance;
import ComaData.ComaRelation;
import ComaData.GenerateComaProxies;
import binding.abstr.AbstractMonitor;

public class ComaOnDemandBindingMonitor extends AbstractMonitor {

	String m_bindingSA;
	HashMap<String, String> m_WMID2proxyIDMap;
	
	/**
	 * The standard constructor...
	 * @param _id
	 */
	public ComaOnDemandBindingMonitor(String _id) {
		super(_id);
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
		m_WMID2proxyIDMap = new HashMap<String, String>();
	}
	
	/**
	 * Configure and initialize the members of this component. 
	 * @param _config
	 */
	public void configure(Properties _config) {
		super.configure(_config);
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
			addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(GenerateComaProxies.class, 
					WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(
						WorkingMemoryChange _wmc) {
					generateNewProxies(_wmc);
				}
			});
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		} // end try..catch
		log("done registering change filters.");
	} // end method  
	
	@Override
	protected void runComponent() {
		super.runComponent();
		// register coma SA's competence to provide
		// area IDs to the planner
		try {
			CompetenceRegistration.registerFeatureGenerationCompetence(this, AreaID.class);
//			CompetenceRegistration.registerFeatureGenerationCompetence(this, RelationLabel.class);
			CompetenceRegistration.registerFeatureGenerationCompetence(this, "position", TemporalFrameType.TYPICAL);
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	
	/**
	 * 
	 * NB:
	 * As a convention, the ComaRelations can also contain
	 * WorkingMemoryPointers to ComaConcept WMEs in order
	 * to make sure default locations get put on the binder!
	 * Another convention is to use a WMP to the GenerateComaProxies WME
	 * to indicate a relation with the multi-concept-default-proxy!
	 * 
	 * @param _wmc
	 */
	private void generateNewProxies(WorkingMemoryChange _wmc) {
		String dataId  = _wmc.m_address.m_id;
		String subArchId = _wmc.m_address.m_subarchitecture;

		// get the actual feature from working memory
		try {
			GenerateComaProxies _creationTask = (GenerateComaProxies) getWorkingMemoryEntry(dataId, subArchId).getData();
			
			WorkingMemoryPointer[] _instanceWMPs = _creationTask.m_instanceWMPs;
			for (WorkingMemoryPointer pointer : _instanceWMPs) {
				ComaInstance _currIns = (ComaInstance) getWorkingMemoryEntry(pointer.m_address).getData();
				String _proxyID = generateNewInstanceProxy(_currIns);
				if (_proxyID!= null) {
					log ("adding new entry to WMID2proxyIDMap: " +pointer.m_address.m_id+ " | " + _proxyID);
					m_WMID2proxyIDMap.put(pointer.m_address.m_id, _proxyID);
				}
			}
			
			WorkingMemoryPointer[] _conceptWMPs = _creationTask.m_conceptWMPs;
			for (WorkingMemoryPointer pointer : _conceptWMPs) {
				ComaConcept _currCon = (ComaConcept) getWorkingMemoryEntry(pointer.m_address).getData();
				String _proxyID = generateNewConceptProxy(_currCon);
				if (_proxyID!= null) {
					log ("adding new entry to WMID2proxyIDMap: " + pointer.m_address.m_id+ " | " + _proxyID);					
					m_WMID2proxyIDMap.put(pointer.m_address.m_id, _proxyID);
				}
			}
			
			String[] _defCons = _creationTask.m_conceptsOfDefaultProxy;
			if (_defCons.length>0) {
				String _proxyID = generateNewMultiConceptProxy(_defCons);
				if (_proxyID!= null) {
					log ("adding new entry to WMID2proxyIDMap: " + _wmc.m_address.m_id + " | " + _proxyID);
					// here we store the wm ID of the coma internalfunction instead!
					m_WMID2proxyIDMap.put(_wmc.m_address.m_id, _proxyID);
				}
			}
			log("m_WMID2proxyIDMap has size = " + m_WMID2proxyIDMap.size());

			WorkingMemoryPointer[] _relationWMPs = _creationTask.m_relationWMPs;
			for (WorkingMemoryPointer pointer : _relationWMPs) {
				ComaRelation _currRel = (ComaRelation) getWorkingMemoryEntry(pointer.m_address).getData();
				String _proxyID = generateNewRelationProxy(_currRel);
//				if (_proxyID!= null) m_WMP2proxyIDMap.put(pointer, _proxyID);
			}

			bindNewProxies();
			
			// delete the task so that the reasoner can send a success signal
			// after all proxies have been written
			for (WorkingMemoryPointer pointer : _relationWMPs) {
				deleteFromWorkingMemory(pointer.m_address);
			}
			deleteFromWorkingMemory(_wmc.m_address);
			
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
	
	
	private String generateNewInstanceProxy(ComaInstance _instance) {
		log("generate new instance proxy");
		// get the actual feature from working memory
		try {
			startNewBasicProxy();
			
			// get all concepts
			String[] cons = _instance.m_mostSpecificConcepts;
			for (String con : cons) {
				Concept _currConFt = new Concept();
				_currConFt.m_concept = con;
				addFeatureToCurrentProxy(_currConFt);
			}

			// get all given names
			String[] names = _instance.m_givenNames;
			for (String name : names) {
				Name _currNameFt = new Name();
				_currNameFt.m_name = name;
				addFeatureToCurrentProxy(_currNameFt);
			}

			// get areaID if possible
			if (_instance.m_name.startsWith("area")) {
				String areaID = _instance.m_name.replace("area", "");
				AreaID _insArID = new AreaID();
				_insArID.m_id = Integer.parseInt(areaID);
				addFeatureToCurrentProxy(_insArID);
			}

			// add instance name as debug string
			DebugString _dbgStr = new DebugString();
			_dbgStr.m_debugString = _instance.m_namespace + 
				_instance.m_sep + 
				_instance.m_name;
			addFeatureToCurrentProxy(_dbgStr);
			
			return storeCurrentProxy();
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}
	
	private String generateNewConceptProxy(ComaConcept  _concept) {
		log("generate new concept proxy");
		// get the actual feature from working memory
		try {
			startNewBasicProxy();

			// add concept name feature
			Concept _conFt = new Concept();
			_conFt.m_concept = _concept.m_name;
			addFeatureToCurrentProxy(_conFt);

			// add full concept name as debug string
			DebugString _dbgStr = new DebugString();
			_dbgStr.m_debugString = _concept.m_namespace + 
				_concept.m_sep + 
				_concept.m_name;
			addFeatureToCurrentProxy(_dbgStr);
			
			return storeCurrentProxy();
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}
	
	
	private String generateNewMultiConceptProxy(String[]  _concepts) {
		log("generate new multi concept proxy");
		// get the actual feature from working memory
		try {
			startNewBasicProxy();

			for (String _conString : _concepts) {
				// add concept name feature
				Concept _conFt = new Concept();
				_conFt.m_concept = _conString;
				addFeatureToCurrentProxy(_conFt);
			}

			// add default nature of that proxy to debug string
			DebugString _dbgStr = new DebugString();
			_dbgStr.m_debugString = "this is a default concept proxy from coma.sa";
			addFeatureToCurrentProxy(_dbgStr);
			
			return storeCurrentProxy();
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	private String generateNewRelationProxy(ComaRelation _relation) {
		log("generate new relation proxy");
		// get the actual feature from working memory
		try {
			log("m_WMID2proxyIDMap has size = " + m_WMID2proxyIDMap.size());
			String _from = m_WMID2proxyIDMap.get(_relation.m_ins1ptr.m_address.m_id);
			String _to = m_WMID2proxyIDMap.get(_relation.m_ins2ptr.m_address.m_id);
			String _label = _relation.m_name;
			
			// TODO make sure, not all coma relations are "TYPICAL"!!!
			log("about to add simple relation");
			log("from = " + _from + " -- _relation.m_ins1ptr.m_address.m_id = " + _relation.m_ins1ptr.m_address.m_id);
			log("to = "   + _to   + " -- _relation.m_ins2ptr.m_address.m_id = " + _relation.m_ins2ptr.m_address.m_id);
			log("label = " + _label);
			addSimpleRelation(_from, _to, _label, TemporalFrameType.TYPICAL);
			
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
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
