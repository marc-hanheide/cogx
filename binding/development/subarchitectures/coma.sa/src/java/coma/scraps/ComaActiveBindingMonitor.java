package coma.scraps;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Properties;

import ComaData.ComaConcept;
import ComaData.ComaInstance;
import ComaData.ComaReasonerFunction;
import ComaData.ComaReasonerFunctionType;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.DoesNotExistOnWMException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import cast.core.data.CASTData;
import binding.abstr.AbstractMonitor;
import BindingData.*;
import BindingFeatures.Concept;
import BindingFeatures.DebugString;
import BindingFeatures.Name;
import BindingFeatures.SourceID;
import BindingFeaturesCommon.TruthValue;

public class ComaActiveBindingMonitor extends AbstractMonitor {

	private HashMap<String,String> m_instancesToProxies;
	private HashMap<String,HashSet<WorkingMemoryAddress>> m_instancesToOtherSAProxies;
	private HashMap<WorkingMemoryAddress,HashSet<String>> m_otherSAProxiesToInstances;
	
	
	public void handleOtherProxyDeleted(WorkingMemoryChange _wmc) {
		log("handleOtherProxyDeleted called...");
		WorkingMemoryAddress deletedWMA  = _wmc.m_address;
		log("WMA of the deleted proxy="+deletedWMA);
		log("Before the deletion:");
		log("m_otherSAProxiesToInstances = " + m_otherSAProxiesToInstances);
		log("m_instancesToOtherSAProxies = " + m_instancesToOtherSAProxies);
		log("m_instancesToProxies = " + m_instancesToProxies);

		// now deleted that proxy and see whether we need to
		// delete a proxy of our own
		HashSet<String> _affectedInstances = m_otherSAProxiesToInstances.remove(deletedWMA);
		if (_affectedInstances==null) {
			log("The deleted proxy does not have any counterparts (_affectedInstances = null)... doing nothing...");
		}
		else {
			log("The deleted proxy has counterparts -> processing the deletion...");
			for (String _affectedInstance : _affectedInstances) {
				HashSet<WorkingMemoryAddress> _remainingProxies = m_instancesToOtherSAProxies.get(_affectedInstance);
				_remainingProxies.remove(deletedWMA);
				if (_remainingProxies.size()==0) {
					m_instancesToOtherSAProxies.remove(_affectedInstance);
					try {
						deleteExistingProxy(_affectedInstance);
					} catch (SubarchitectureProcessException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
				else {
					m_instancesToOtherSAProxies.put(_affectedInstance, _remainingProxies);
				}
			}
		}
		log("After the deletion:");
		log("m_otherSAProxiesToInstances = " + m_otherSAProxiesToInstances);
		log("m_instancesToOtherSAProxies = " + m_instancesToOtherSAProxies);
		log("m_instancesToProxies = " + m_instancesToProxies);
	}
			
			
			
	public void handleNewBindingProxy(WorkingMemoryChange _wmc) {
		log("handleNewBindingProxy called...");
		log("Before handling the new proxy:");
		log("m_otherSAProxiesToInstances = " + m_otherSAProxiesToInstances);
		log("m_instancesToOtherSAProxies = " + m_instancesToOtherSAProxies);
		log("m_instancesToProxies = " + m_instancesToProxies);
		try {
			// get the id and subarch_id of the working memory entry
			String dataId  = _wmc.m_address.m_id;
			String subArchId = _wmc.m_address.m_subarchitecture;
			log("dataID="+dataId);
			log("subArchId="+subArchId);
			
			if (!existsOnWorkingMemory(_wmc.m_address)) {
				log("trying to access a non-existing proxy...skipping!");
				return;
			}

			// get the data from working memory
			CASTData data = getWorkingMemoryEntry(dataId, subArchId);
			BindingProxy proxy = (BindingProxy) data.getData();
						
			log("got binding proxy. now iterate through feature pointer list of the binding proxy.");
			FeaturePointer _conPointer = null;
			log("A");
			boolean _notComaProxy = false;
			log("B");
			int i = 1;
			for (FeaturePointer _ftpointer : proxy.m_proxyFeatures) {
				log("C"+i++);
				if (_notComaProxy && _conPointer!=null) {
					log("notComaProxy && conPointer!=null -> break!");
					break;
				}
				log("current feature pointer: "+_ftpointer + " -- type: "+_ftpointer.m_type);
				if (_ftpointer.m_type.equals(CASTUtils.typeName(Concept.class))) {
					log("ftpointer=CONCEPT_TYPE -> conPointer = ftpointer");
					_conPointer = _ftpointer;
					continue;
				}
				else if (_ftpointer.m_type.equals(CASTUtils.typeName(SourceID.class))){
					// log("ftpointer = SOURCE ID TYPE => check original source subarch");
					SourceID _sourceSA_ID = (SourceID) getWorkingMemoryEntry(_ftpointer.m_address, subArchId).getData();
					if (_sourceSA_ID.m_sourceID.equals(m_subarchitectureID)) {
						log("we have a proxy from our SA: ignore it!");
						return;
					}
					else {
						log(" this is a proxy from someone else (namely: "+
								_sourceSA_ID +
								"), so let's process it (_notComaProxy will be set to true)!");
						_notComaProxy = true;
						continue;
					}
				}
				else {
					log("This feature type is not of any interest for us... doing nothing...");
				}
			}
			
			if (_conPointer!=null) {
				Concept conceptFeature = (Concept) getWorkingMemoryEntry(_conPointer.m_address, subArchId).getData();
				String _cID = newTaskID();
				addToWorkingMemory(_cID, new ComaConcept("", "", conceptFeature.m_concept));

				WorkingMemoryPointer _arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
						new WorkingMemoryAddress(_cID, m_subarchitectureID));

				// NEW
				ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.GetAllInstances,
						_arg1ptr,
						new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
						new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
						new WorkingMemoryPointer(CASTUtils.typeName(BindingProxy.class), _wmc.m_address));

				String _reqID = newDataID();
				addToWorkingMemory(_reqID, _req);

				addChangeFilter(ChangeFilterFactory.createAddressFilter(
						new WorkingMemoryAddress(_reqID,m_subarchitectureID), 
						WorkingMemoryOperation.OVERWRITE),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(WorkingMemoryChange _wmc) {
								// log(CASTUtils.toString(_wmc));
								handleComaReasonerFunctionChanged(_wmc);
							}
						});
			}
			else {
				log("This proxy does not contain a concept feature! ...doing nothing...");
			}
		} catch (SubarchitectureProcessException e) {
			println(e.getLocalizedMessage());
			e.printStackTrace();
		} // end try .. catch
		log("After handling the new proxy:");
		log("m_otherSAProxiesToInstances = " + m_otherSAProxiesToInstances);
		log("m_instancesToOtherSAProxies = " + m_instancesToOtherSAProxies);
		log("m_instancesToProxies = " + m_instancesToProxies);
	}
			
	public void handleComaReasonerFunctionChanged(WorkingMemoryChange _wmc) {
		boolean _bindingNecessary = false;
		log("got a callback: the comaReasoner has evaluated a function!");
		log("Before handling the function:");
		log("m_otherSAProxiesToInstances = " + m_otherSAProxiesToInstances);
		log("m_instancesToOtherSAProxies = " + m_instancesToOtherSAProxies);
		log("m_instancesToProxies = " + m_instancesToProxies);
		// so, we need to check whether that function is relevant for us
		// then we need to check whether it is still relevant
		// or whether the original proxy has already been deleted, 
		// then we don't have to do all this!
		try {
			// get the WME-ID of the function
			String dataId  = _wmc.m_address.m_id;
			String subArchId = _wmc.m_address.m_subarchitecture;
			
			// get the actual function from working memory
			CASTData data = getWorkingMemoryEntry(dataId, subArchId);
			ComaReasonerFunction function = (ComaReasonerFunction) data.getData();
			
			// check if the function is relevant for us
			// NB it should be cos we have installed this specific listener!
			if (function.m_functiontype.equals(ComaReasonerFunctionType.GetAllInstances)) {
				// first check whether the original proxy that triggered the function still exists
				try {
					WorkingMemoryAddress _otherProxyWMA = function.m_add_info_ptr.m_address;
					log("original binding proxy still exists. proxy ID: "+_otherProxyWMA);
					log("I have a GetAllInstances function... I will process it...");

					// get results
					ComaInstance[] comaInstanceList = (ComaInstance[]) getWorkingMemoryEntry(function.m_resultptr.m_address).getData();

					// process all returned instances:
					log("I got an instance list of length " + comaInstanceList.length + " as a result.");
					if (comaInstanceList.length>=15 ) {
						log("I got at least 15 answers. That is too much... the context is too ambiguous... I'm stopping here!");
						return;
					}
					int i = 1;
					for (ComaInstance instance : comaInstanceList) {
						log("["+ i++ +"] current instance: " + instance.m_name);
						// do we already have a proxy for our instance?
						if (m_instancesToProxies.containsKey(instance.m_namespace+instance.m_sep+instance.m_name)) {
							// we do already have a proxy for the given instance,
							// so let's see if we can provide more info to it
							
							// but first let us remember that we have a new external proxy for
							// an existing instance proxy, no matter if we have any more infor or not!
							HashSet<WorkingMemoryAddress> _otherProxies = m_instancesToOtherSAProxies.get(instance.m_namespace+instance.m_sep+instance.m_name);
							_otherProxies.add(_otherProxyWMA);
							m_instancesToOtherSAProxies.put(instance.m_namespace+instance.m_sep+instance.m_name, _otherProxies);

							// establish the inverse map:
							HashSet<String> _myInstanceProxies;
							if (m_otherSAProxiesToInstances.containsKey(_otherProxyWMA)) {
								_myInstanceProxies = m_otherSAProxiesToInstances.get(_otherProxyWMA);
							} else {
								_myInstanceProxies = new HashSet<String>();
								// now monitor the other proxy: make sure, our instances get removed if the 
								// original proxy ceases to exist!
								try {
									addChangeFilter(ChangeFilterFactory.createAddressFilter(_otherProxyWMA,WorkingMemoryOperation.DELETE), 
											new WorkingMemoryChangeReceiver() {
										public void workingMemoryChanged(WorkingMemoryChange _wmc) {
											// log(CASTUtils.toString(_wmc));
											handleOtherProxyDeleted(_wmc);
										}
									});
								}
								catch (SubarchitectureProcessException e) {
									e.printStackTrace();
								} // end try..catch
							}
							_myInstanceProxies.add(instance.m_namespace+instance.m_sep+instance.m_name);
							m_otherSAProxiesToInstances.put(_otherProxyWMA,_myInstanceProxies);
							
							// now it is time to look at our own proxy
							// always overwrite the concept info w/ the most specific one
							HashSet<String> _excludeFeatures = new HashSet<String>();
							_excludeFeatures.add(CASTUtils.typeName(Concept.class));
							_excludeFeatures.add(CASTUtils.typeName(Name.class));
							changeExistingProxy(m_instancesToProxies.get(instance.m_namespace+instance.m_sep+instance.m_name),_excludeFeatures);

							for (String _currConString : instance.m_mostSpecificConcepts) {
//								Concept _concpt = new Concept(_currConString.toLowerCase(), false, "");
								Concept _concpt = new Concept();
								_concpt.m_concept=_currConString.toLowerCase();
								addFeatureToCurrentProxy(_concpt);
							}

							// removed the following block of code after changing from
							// a single name to a list of names
							// too lazy to fix this now, hz 2008-07-24
//							if (!instance.m_nameFeature.equals("")) {
//								Name _name = new Name();
//								_name.m_name=instance.m_nameFeature;
//								addFeatureToCurrentProxy(_name);
//							}
							// end removal
							
							storeCurrentProxy();
							_bindingNecessary = true;

							// OLD CODE: here we check for concept feature and do not touch it
							// in case we know a new concept, we just add it...
							/*
							BindingProxy _existingProxy = (BindingProxy) getWorkingMemoryEntry(m_instancesToProxies.get(instance.m_shortname), m_bindingSA).getData();
							boolean containsOriginalConcept = false;
							for (FeaturePointer _ftpointer : _existingProxy.m_proxyFeatures) {
								// we are interested in concepts only
								if (_ftpointer.m_type.equals(BindingOntology.CONCEPT_TYPE)) {
									log("we know about some concept already.");
									// aha, we know about some concept already
									Concept _currConValue = (Concept) getWorkingMemoryEntry(_ftpointer.m_address, m_bindingSA).getData();
									log("the current concept of the existing proxy = "+_currConValue.m_concept + " -- and the original concept = "+originalCon.m_concept);
									if (_currConValue.m_concept.equals(originalCon.m_concept)) {
										log("ok, set containsOriginalConcept to true");
										// aha, we indeed already knew the "new" concept
										// so we can abort this loop through the concept features at this point
										containsOriginalConcept = true;
										break;
									}
									else {
										// if we are here, we have found a concept, but not the one
										// that is new now... so we still think that we have new knowledge...
										// let's go through the rest of the loop
										continue; // I know that that's redundant...
									}
								}
							}
							// ok, we have gone through all features and we know
							// whether we can provide a new feature or not
							if (!containsOriginalConcept) {
								log("The existing proxy does not contain the original concept.");
								// we can indeed provide new info!
								log("starting change existing proxy");
								log("current instance is " + instance.m_shortname);
								log("the corresponding proxy is " + m_instancesToProxies.get(instance.m_shortname));
								log("-> changeExistingProxy!");
								changeExistingProxy(m_instancesToProxies.get(instance.m_shortname));
								addFeatureToCurrentProxy(originalCon);
								storeCurrentProxy();
								_bindingNecessary = true;
								log("performed changes -> storeCurrentProxy!");
							}
							*/
							
							// no matter if we could provide more info,
							// we have stored the fact that we have a new external proxy that 
							// elicited a proxy of ours!
						} // end "do we already have a proxy for the instance?"
						else { 
							// so we don't have a proxy for this instance, let's create one!
							log("startNewBasicProxy!");
							startNewBasicProxy();
							for (String _currConString : instance.m_mostSpecificConcepts) {
//								Concept _concpt = new Concept(_currConString.toLowerCase(), false, "");
								Concept _concpt = new Concept();
								_concpt.m_concept=_currConString.toLowerCase();
								addFeatureToCurrentProxy(_concpt);
							}

							// removed the following block of code after changing from
							// a single name to a list of names
							// too lazy to fix this now, hz 2008-07-24
//							if (!instance.m_nameFeature.equals("")) {
//								Name _name = new Name();
//								_name.m_name=instance.m_nameFeature;
//								addFeatureToCurrentProxy(_name);
//							}
							// end removal
								
								
//							addFeatureToCurrentProxy(new DebugString("coma_instance_id:" + instance.m_shortname, false, ""));
							DebugString _dbgStr = new DebugString();
							_dbgStr.m_debugString="coma_instance_id:" + instance.m_name;
							addOtherSourceIDToCurrentProxy(m_subarchitectureID, TruthValue.POSITIVE);
							String newProxy = storeCurrentProxy();
							_bindingNecessary = true;
							log("performed changes -> storeCurrentProxy!");
							// now store the mappings between other proxies and our instance proxies
							// 1. establish link between our ontology instances and our own proxies
							m_instancesToProxies.put(instance.m_namespace+instance.m_sep+instance.m_name, newProxy);
							// 2. map our instance proxies to the other proxies they belong to
							// get the ID of the proxy we are reacting to, i.e. a proxy from a different SA
							HashSet<WorkingMemoryAddress> _otherProxies;
							if (m_instancesToOtherSAProxies.containsKey(instance.m_namespace+instance.m_sep+instance.m_name)) {
								_otherProxies = m_instancesToOtherSAProxies.get(instance.m_namespace+instance.m_sep+instance.m_name);
							} else {
								_otherProxies = new HashSet<WorkingMemoryAddress>();
							}
							_otherProxies.add(_otherProxyWMA);
							m_instancesToOtherSAProxies.put(instance.m_namespace+instance.m_sep+instance.m_name, _otherProxies);
							// 3. establish the inverse map:
							// 
							HashSet<String> _myInstanceProxies;
							if (m_otherSAProxiesToInstances.containsKey(_otherProxyWMA)) {
								_myInstanceProxies = m_otherSAProxiesToInstances.get(_otherProxyWMA);
							} else {
								_myInstanceProxies = new HashSet<String>();
								// now monitor the other proxy: make sure, our instances get removed if the 
								// original proxy ceases to exist!
								try {
									addChangeFilter(ChangeFilterFactory.createAddressFilter(_otherProxyWMA, WorkingMemoryOperation.DELETE),
											new WorkingMemoryChangeReceiver() {
										public void workingMemoryChanged(WorkingMemoryChange _wmc) {
											// log(CASTUtils.toString(_wmc));
											handleOtherProxyDeleted(_wmc);
										}
									});
								}
								catch (SubarchitectureProcessException e) {
									e.printStackTrace();
								} // end try..catch
							}
							_myInstanceProxies.add(instance.m_namespace+instance.m_sep+instance.m_name);
							m_otherSAProxiesToInstances.put(_otherProxyWMA,_myInstanceProxies);
						}
					}
				// only now trigger binding of the new/changed proxies!	
				if (_bindingNecessary) {
					log("we have performed changes involving proxies -> bindNewProxies()");
					bindNewProxies();	
				}
				} catch (DoesNotExistOnWMException e) {
					log("caught execption: the proxy had ceased to exist!!! doing nothing...");
				}
			}
			else {
				log ("The function is irrelevant for us! exiting...");
				return;
			}
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		} // end try..catch
		log("After handling the function:");
		log("m_otherSAProxiesToInstances = " + m_otherSAProxiesToInstances);
		log("m_instancesToOtherSAProxies = " + m_instancesToOtherSAProxies);
		log("m_instancesToProxies = " + m_instancesToProxies);
	}




	
	
	public ComaActiveBindingMonitor(String _id) {
		super(_id);
		// TODO Auto-generated constructor stub
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
    	   addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(BindingProxy.class, WorkingMemoryOperation.ADD),
               new WorkingMemoryChangeReceiver() {
                   public void workingMemoryChanged(
                           WorkingMemoryChange _wmc) {
                	   log("Someone has added a new proxy to the binder...");
                       handleNewBindingProxy(_wmc);
                   }
               });
       }
       catch (SubarchitectureProcessException e) {
           e.printStackTrace();
       } // end try..catch
   } // end method  
	
   // Methods that Henrik insists on:
   @Override
   public void configure(Properties _config) {
       super.configure(_config);
   
       //set the source id to be this subarch id
       m_sourceID = m_subarchitectureID;
       m_instancesToProxies = new HashMap<String,String>();
       m_instancesToOtherSAProxies = new HashMap<String,HashSet<WorkingMemoryAddress>>();
       m_otherSAProxiesToInstances = new HashMap<WorkingMemoryAddress, HashSet<String>>();
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
	