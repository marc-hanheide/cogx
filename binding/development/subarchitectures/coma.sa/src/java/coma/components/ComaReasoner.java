/*
 * The Conceptual Mapping Reasoner
 *
 * Copyright (c) 2007, 2008 Hendrik Zender, zender@dfki.de
 */
package coma.components;

import java.util.ArrayList;
import java.util.Properties;
import java.util.Set;
import java.util.TreeSet;

import binding.util.BindingUtils;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.AlreadyExistsOnWMException;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.*;
import cast.core.CASTUtils;
import cast.core.data.CASTData;

import org.cognitivesystems.reasoner.base.ReasonerConcept;
import org.cognitivesystems.reasoner.base.ReasonerInstance;
import org.cognitivesystems.reasoner.base.ReasonerInterface;
import org.cognitivesystems.reasoner.base.OntologyMemberFactory;
import org.cognitivesystems.reasoner.base.ReasonerException;
import org.cognitivesystems.reasoner.base.ReasonerRelation;
import org.cognitivesystems.reasoner.crowl.CrowlWrapper;
import org.cognitivesystems.reasoner.pellet.PelletWrapper;


import BindingData.BindingProxy;
import BindingFeatures.AreaID;
import BindingFeatures.Concept;
import BindingFeatures.RelationLabel;
import BindingQueries.BasicQuery;
import BindingQueries.FeatureRequest;
import ComaData.ComaReasonerFunction;
import ComaData.ComaReasonerFunctionType;
import ComaData.ComaInstance;
import ComaData.ComaConcept;
import ComaData.ComaRelation;
import ComaData.StringWrapper;
import ComaData.TriBoolResult;
import coma.aux.ComaDefaultLocationProvider;
import coma.aux.ComaGREAlgorithm;
import coma.aux.ComaReferenceResolver;
import coma.aux.ComaTypeException;
import planning.autogen.Action;

/**
 * @author zender
 */
public class ComaReasoner extends PrivilegedManagedProcess {

	private ReasonerInterface m_reasoner;
	private String m_ontologyNamespace;
	private String m_ontologySep;
//	private OntologyMemberFactory m_oeMemberFactory;
//	private ComaFunctionWriter m_comaFuncWriter;
	private String m_bindingSA;
	
    /**
     * @param _id
     */
    public ComaReasoner(String _id) {
        super(_id);
        m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
    }
    
    /*
     * ComaReasoner can take its ontology filename and namespace
     * from the config line. If they're not specified in the config
     * file, ComaReasoner uses default values.
     */
    public void configure(Properties _config) {
    	super.configure(_config);
    	log("configure()");

    	String ontologyFile;
    	if (_config.containsKey("--onto_file")) {
    		// ontologyFile = "file://"+_config.getProperty("--onto_file");
    		ontologyFile = _config.getProperty("--onto_file");
    	}
        else {
            //ontologyFile = "file:///project/cl/cosy/svn.cosy/code/subarchitectures/coma.sa/ontologies/officeenv_wn.owl";
            ontologyFile = "/project/cl/cosy/svn.cosy/code/subarchitectures/coma.sa/ontologies/officeenv_wn.owl";
        }
        
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
        String reasoner;
        if (_config.containsKey("--reasoner")) {
            reasoner = _config.getProperty("--reasoner").toLowerCase();
        }
        else {
        	reasoner = "pellet";
        }
        log("Using reasoner: "+reasoner);
        log("Using namespace: " +m_ontologyNamespace);
        log("Using separator: " + m_ontologySep);
        if (reasoner.equals("crowl")) {
    		if (_config.containsKey("--crowl_cfg")) {
            	try {
            		m_reasoner = new CrowlWrapper(m_ontologyNamespace, m_ontologySep, _config.getProperty("--crowl_cfg").toLowerCase());
            	} catch (ReasonerException e) {
            		e.printStackTrace();
            		log("EXITING!");
            		System.exit(0);
            	}
    		} else {
    			log("You must specify a Crowl config file!");
    			System.exit(0);
    		}
        } else {
        	try {
        		boolean _pelletLog = false;
        		if (_config.containsKey("--pellet_log")) {
        			if (!_config.getProperty("--pellet_log").equals("false")) _pelletLog = true;
        		}
        		log("Using ontology file: " + ontologyFile);
        		m_reasoner = new PelletWrapper(ontologyFile, m_ontologyNamespace, m_ontologySep, _pelletLog);
        	} catch (ReasonerException e) {
        		e.printStackTrace();
        		log("EXITING!");
        		System.exit(0);
        	}
        }
        
        if (_config.containsKey("--bsa")) {
            this.m_bindingSA = _config.getProperty("--bsa");
        } else if (_config.containsKey("-bsa")) {
            this.m_bindingSA = _config.getProperty("-bsa");
        } else {
        	throw new RuntimeException("You must specify the binding SA ID: --bsa <SA-ID> or -bsa <SA-ID>!");
        }

        
        log("Successfully configured ComaReasoner:");
        log("ontology file: " + ontologyFile);
        log("ontology namespace: " + m_ontologyNamespace);
        log("using reasoner: " + reasoner);
//        this.m_oeMemberFactory = new OntologyMemberFactory(this.m_ontologyNamespace, this.m_ontologySep);
//        this.m_comaFuncWriter = new ComaFunctionWriter(this);
    }


    public void start() {
        super.start();
        try {
        	addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
        			ComaReasonerFunction.class, 
        			WorkingMemoryOperation.ADD),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                        processAction(_wmc);
                    }
                });
        	
			addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
					Action.class, WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							processNewPlanningAction(_wmc);
						}
					});
        }
        catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException("Error! Could not register change filter. Aborting!");
        }

        // writing all instances that are already present in the ontology
        // to coma WM, so that at every time all known instances are stored 
        // on coma WM
        Set<ReasonerInstance> _allPredefinedInstances = m_reasoner.getInstances(
        		OntologyMemberFactory.createConcept("owl", ":", "Thing"));
        
        log("Registering all predefined instances on coma WM");
        log("The reasoner returned a set of " + _allPredefinedInstances.size() +
        		" instances of the concept owl:Thing");
        int i=0;
        for (ReasonerInstance instance : _allPredefinedInstances) {
			log("writing instance #"+ i++ + ": " + instance);
			// initialize ComaInstance
			ComaInstance _currIns = new ComaInstance(instance.getNamespace(),
					instance.getSep(),
					instance.getName(),
					new String[0],
					new String[0]);
			// fill in the most specific concepts and given name(s)
			_currIns = refreshInstanceStruct(_currIns);
			// write out ComaInstance to WM
	    	try {
				addToWorkingMemory(newDataID(), _currIns, OperationMode.BLOCKING);
			} catch (AlreadyExistsOnWMException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (SubarchitectureProcessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
        
    }
    
	public ReasonerInterface getReasoner() {
    	return m_reasoner;
    }
	
//	public OntologyMemberFactory getOntoMemberFactory() {
//		return m_oeMemberFactory;
//	}
    
//	public ComaFunctionWriter getComaFuncWriter() {
//		return m_comaFuncWriter;
//	}

	public String getOntologyNS() {
    	return m_ontologyNamespace;
    }
    
    public String getOntologySep() {
    	return m_ontologySep;
    }
    
//    public String getBindingSA() {
//    	return m_bindingSA;
//    }

    
    private void processNewPlanningAction(WorkingMemoryChange _actionWMC) {
		log("got a planning action with ID " + _actionWMC.m_address.m_id);
    	try {    		
			CASTData<?> _actionWME = getWorkingMemoryEntry(_actionWMC.m_address);
			Action _myAction = (Action) _actionWME.getData();
			if (! _myAction.m_action.m_type.equals(CASTUtils.typeName(FeatureRequest.class))) {
				log("This action is of type: " + _myAction.m_action.m_type + " -- so I can't handle it. Doing nothing... for"+ _actionWMC.m_address.m_id);
				return;
			}
			FeatureRequest _myResRequest = (FeatureRequest) getWorkingMemoryEntry(_myAction.m_action.m_address).getData();
			BasicQuery _myQuery = _myResRequest.m_request;
			
			if (_myQuery.m_featurePointer.m_type.equals(CASTUtils.typeName(AreaID.class))) {
				//
				// RESOLVE A REFERENCE TO A PROXY -> DETERMINE AREA-ID
				//
				log("got a planning action about an AreaID feature -- will try to resolve it against my conceptual spatial map. -- for "+ _actionWMC.m_address.m_id);
				WorkingMemoryAddress _proxyToCheckWMA = new WorkingMemoryAddress(_myQuery.m_proxyID,
						m_bindingSA);
				ComaReferenceResolver _resolver = new ComaReferenceResolver(this, _actionWMC);
				_resolver.resolveAgainstProxy(_proxyToCheckWMA);
			} // modify from here on for other cases, like, (default) locations
			else if (_myQuery.m_featurePointer.m_type.equals(CASTUtils.typeName(RelationLabel.class))) {
				//
				// This Action is about a Relation; determine behavior based on relation label
				//
				log("got a planning action about a RelationLabel at " + _actionWMC.m_address.m_id);
				String _relLabel = ((RelationLabel) getWorkingMemoryEntry(
						_myQuery.m_featurePointer.m_address, m_bindingSA).
						getData()).m_label;
				if (_relLabel.equalsIgnoreCase("position")) {
					//
					// RESOLVE POSITION OF A PROXY
					//
					log("the RelationLabel is 'position' -- going to provide an as good estimate of the proxy's position as possible. -- for " + _actionWMC.m_address.m_id);
					
					WorkingMemoryAddress _proxyToCheckWMA = new WorkingMemoryAddress(_myQuery.m_proxyID,
							m_bindingSA);

					// first read the proxy
					// then get its concept
					BindingProxy _proxyInQuestion = null;
					try {
						_proxyInQuestion = BindingUtils.getProxy(this, m_bindingSA, _myQuery.m_proxyID);
						// TODO check other features as well
						// eg objectID
						ArrayList<Concept> _allCons = BindingUtils.getBindingFeatures(this, m_bindingSA, _proxyInQuestion, Concept.class);
						
						// ok assume if I only get Concept, then I shall look for any instance; or for default instances
						// if I have multiple concepts, I shall only look for thos instances that match all concepts
						Set<ReasonerConcept> _allRCons = new TreeSet<ReasonerConcept>();
						for (Concept concept : _allCons) {
							ReasonerConcept _rcon = OntologyMemberFactory.createConcept(
									m_ontologyNamespace, 
									m_ontologySep, 
									concept.m_concept);
							_allRCons.add(_rcon);
						}
						// now retrieve the set of instances
						Set<ReasonerInstance> _allInstances = m_reasoner.getInstances(_allRCons);
						
						// if we got any instances then put those on the binder
						if (!_allInstances.isEmpty()) {
							//
							// TODO PRESENT INSTANCE KNOWLDGE
							//
							log("TODO: put all instances on the binder!!!");
						} else { // if not, go for default locs
							//
							// RESOLVE DEFAULT LOCATIONS
							//
							log("did not find any specific instances; going to present default locations to the binder -- for "+ _actionWMC.m_address.m_id);

							
							ComaDefaultLocationProvider _defaultLocProvider = new ComaDefaultLocationProvider(this, _actionWMC);
							_defaultLocProvider.provideDefaultLocForProxy(_proxyToCheckWMA, _allRCons);
							
						}
						
						
					} catch (SubarchitectureProcessException e) {
						e.printStackTrace();
						log("cannot resolve non existing proxy. Returning... for " + _actionWMC.m_address.m_id);
						return;
					}
				} else {
					log("the RelationLabel is '"+ _relLabel +"' -- don't know about that. doing nothing... --- for "+ _actionWMC.m_address.m_id);
				}
				
// 	DUNNO WHAT THIS CODE IS DOING??? Taken out: Sep 02 2008, hz.				
//				WorkingMemoryAddress _proxyToCheckWMA = new WorkingMemoryAddress(_myQuery.m_proxyID,
//						m_bindingSA);
//				
//				
//				ComaReferenceResolver _resolver = new ComaReferenceResolver(this, _actionWMC);
//				_resolver.resolveAgainstProxy(_proxyToCheckWMA);
//				UNTIL HERE Sep 02 2008, hz.
			
			}
			
			// this is what needs to be set in the end:
			//			_myQuery.m_answer
			//			_myQuery.m_parameters;
			//			_myQuery.m_processed; // set this one when done
						
			//			ResolveEntity _myResTask = (ResolveEntity) wme.getData();
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/**
     * @param _address
     */
    private void processAction(WorkingMemoryChange _actionChange) {
        log("processAction()");
    	// get the action from wm
        try {
            CASTData<?> wme = getWorkingMemoryEntry(_actionChange.m_address);
            ComaReasonerFunction action = (ComaReasonerFunction) wme.getData();
            String wme_ID = wme.getID();
            switch (action.m_functiontype.value()) {
                case ComaReasonerFunctionType._AreConsEquivalent:
                    areConsEquiv(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._IsConSubcon:
                    isConSubcon(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._IsConSupercon:
                    isConSupercon(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._IsInstanceOf:
                    isInstanceOf(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._GetAllConcepts:
                    getAllCons(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._GetAllDirectConcepts:
                	getAllDirectCons(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._GetMostSpecificConcepts:
                	getMostSpecificCons(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._GetAllInstances:
                	getAllInstances(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._GetInstancesByName:
                	getInstancesByName(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._GetRelatedInstances:
                	getRelatedInstances(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._GetBasicLevelConcepts:
                	getBasicLevelCons(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._CompareCons:
                	compareConcepts(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._GetObjectMobility:
                	getObjectMobility(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._GetTypicalObjects:
                	getTypicalObjectCons(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._DeleteInstance:
                	deleteInstance(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._AddInstance:
                	addInstance(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._AddInstanceNumberTag:
                	addInstanceNumberTag(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._AddRelation:
                	addRelation(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._AddInstanceName:
                	addInstanceName(action, wme_ID);
                    break;
                case ComaReasonerFunctionType._GenerateRefEx:
                	generateRefEx(action, wme_ID);
                    break;
                default:
                    log("Unknown action type");
                    break;
            }

            // // now delete the action itself from our sa wm
            // debug("deleting action at: " + CASTUtils.toString(_wma));
            // deleteFromWorkingMemory(_actionChange.m_address.m_id);
        }
        catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
        }
        log("end processAction()");
    }

    
    private void areConsEquiv(ComaReasonerFunction _action, String _wmeID) {
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
    	WorkingMemoryPointer _arg2ptr = _action.m_arg2ptr;
    	TriBoolResult _result = new TriBoolResult();
    	_result.m_tribool = TriBool.triIndeterminate;
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class)) &&
    			_arg2ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
    		ComaConcept _arg1;
    		try {
    			_arg1 = (ComaConcept) getWorkingMemoryEntry(_arg1ptr.m_address).getData();
    		} catch (SubarchitectureProcessException e) {
    			e.printStackTrace();
    			throw new RuntimeException(e);
    		}
    		ComaConcept _arg2;
    		try {
    			_arg2 = (ComaConcept) getWorkingMemoryEntry(_arg2ptr.m_address).getData();
    		} catch (SubarchitectureProcessException e) {
    			e.printStackTrace();
    			throw new RuntimeException(e);
    		}
    		ReasonerConcept _con1 = OntologyMemberFactory.createConcept(
    				(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
    				(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
    				_arg1.m_name);
    		ReasonerConcept _con2 = OntologyMemberFactory.createConcept(
    				(_arg2.m_namespace.equals("") ? m_ontologyNamespace : _arg2.m_namespace), 
    				(_arg2.m_sep.equals("") ? m_ontologySep : _arg2.m_sep), 
    				_arg2.m_name);
    		if (m_reasoner.areConceptsEquivalent(
    				_con1, 
    				_con2)) {
    			_result.m_tribool = TriBool.triTrue;
    			log("concepts are equiv!");
    		}
    		else if (m_reasoner.conceptExists(_con1) && m_reasoner.conceptExists(_con2)) {
    			_result.m_tribool = TriBool.triFalse;
    			log("concepts are not equiv!");
    		}
    		else {
    			log("at least one of the concepts does not exist -> indeterminate!");
				}
		} else {
			String message = "";
			if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
				message += "arg1 WME has wrong IDL type. It should be of type ComaConcept, but it is of type: " + _arg1ptr.m_type;
			}
			if (_arg2ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
				message += "\n arg2 WME has wrong IDL type. It should be of type ComaConcept, but it is of type: " + _arg1ptr.m_type;
			}
			throw new ComaTypeException(message);
		}
    	String _newID = newDataID();
        try {
        	addToWorkingMemory(_newID, _result,OperationMode.BLOCKING);
	        _action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(TriBoolResult.class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
	        overwriteWorkingMemory(_wmeID, _action,OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}

    private void compareConcepts(ComaReasonerFunction _action, String _wmeID) {
    	log("compareConcepts() called!");
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
    	WorkingMemoryPointer _arg2ptr = _action.m_arg2ptr;
		TriBoolResult _result = new TriBoolResult();
		_result.m_tribool = TriBool.triIndeterminate;
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class)) &&
				_arg2ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
    		log("going to compare two COMA_CONCEPTS...");
			try {
				ComaConcept _arg1 = (ComaConcept) getWorkingMemoryEntry(_arg1ptr.m_address).getData();
				ComaConcept _arg2 = (ComaConcept) getWorkingMemoryEntry(_arg2ptr.m_address).getData();
				ReasonerConcept _con1 = OntologyMemberFactory.createConcept(
						(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
						(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
						_arg1.m_name);
				ReasonerConcept _con2 = OntologyMemberFactory.createConcept(
						(_arg2.m_namespace.equals("") ? m_ontologyNamespace : _arg2.m_namespace), 
						(_arg2.m_sep.equals("") ? m_ontologySep : _arg2.m_sep), 
						_arg2.m_name);
				log("comparing ["+_arg1.m_name+"] vs. ["+_arg2.m_name+"]...");
				if (m_reasoner.areConceptsEquivalent(_con1,_con2) ||
						m_reasoner.isSubConcept(_con1, _con2) ||
						m_reasoner.isSuperConcept(_con1, _con2)) {
					_result.m_tribool = TriBool.triTrue;
					log("["+_arg1.m_name+"] and ["+_arg2.m_name+"] are comparable concepts -> triTrue");
				}
				else if (m_reasoner.conceptExists(_con1) && m_reasoner.conceptExists(_con2)) {
					_result.m_tribool = TriBool.triFalse;
					log("["+_arg1.m_name+"] and ["+_arg2.m_name+"] are not comparable concepts -> triFalse.");
				}
				else {
			    	log("at least one of the concepts does not exist -> triIndeterminate!");
				}
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			}
		} else {
			String message = "";
			if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
				message += "arg1 WME has wrong IDL type. It should be of type ComaConcept, but it is of type: " + _arg1ptr.m_type;
			}
			if (_arg2ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
				message += "\n arg2 WME has wrong IDL type. It should be of type ComaConcept, but it is of type: " + _arg1ptr.m_type;
			}
			throw new ComaTypeException(message);
		}
    	String _newID = newDataID();
        try {
	    addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
	        _action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(TriBoolResult.class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
	        overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}

    
    private void isConSubcon(ComaReasonerFunction _action, String _wmeID) {
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
    	WorkingMemoryPointer _arg2ptr = _action.m_arg2ptr;
		TriBoolResult _result = new TriBoolResult();
		_result.m_tribool = TriBool.triIndeterminate;
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class)) &&
				_arg2ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
			try {
				ComaConcept _arg1 = (ComaConcept) getWorkingMemoryEntry(_arg1ptr.m_address).getData();
				ComaConcept _arg2 = (ComaConcept) getWorkingMemoryEntry(_arg2ptr.m_address).getData();

				ReasonerConcept _con1 = OntologyMemberFactory.createConcept(
						(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
						(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
						_arg1.m_name);
				ReasonerConcept _con2 = OntologyMemberFactory.createConcept(
						(_arg2.m_namespace.equals("") ? m_ontologyNamespace : _arg2.m_namespace), 
						(_arg2.m_sep.equals("") ? m_ontologySep : _arg2.m_sep), 
						_arg2.m_name);
				if (m_reasoner.isSubConcept(
						_con1, 
						_con2)) {
					_result.m_tribool = TriBool.triTrue;
			    	log("concept1 is subcon of concept2!");
				}
				else if (m_reasoner.conceptExists(_con1) && m_reasoner.conceptExists(_con2)) {
					_result.m_tribool = TriBool.triFalse;
			    	log("concept1 is not a subcon of concept2");
				}
				else {
			    	log("at least one of the concepts does not exist -> indeterminate!");
				}
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			}
		} else {
			String message = "";
			if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
				message += "arg1 WME has wrong IDL type. It should be of type ComaConcept, but it is of type: " + _arg1ptr.m_type;
			}
			if (_arg2ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
				message += "\n arg2 WME has wrong IDL type. It should be of type ComaConcept, but it is of type: " + _arg1ptr.m_type;
			}
			throw new ComaTypeException(message);
		}
    	String _newID = newDataID();
        try {
	    addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
			_action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(TriBoolResult.class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
			overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}

    
    private void isConSupercon(ComaReasonerFunction _action, String _wmeID) {
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
    	WorkingMemoryPointer _arg2ptr = _action.m_arg2ptr;
		TriBoolResult _result = new TriBoolResult();
		_result.m_tribool = TriBool.triIndeterminate;
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class)) &&
				_arg2ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
			try {
				ComaConcept _arg1 = (ComaConcept) getWorkingMemoryEntry(_arg1ptr.m_address).getData();
				ComaConcept _arg2 = (ComaConcept) getWorkingMemoryEntry(_arg2ptr.m_address).getData();

				ReasonerConcept _con1 = OntologyMemberFactory.createConcept(
						(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
						(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
						_arg1.m_name);
				ReasonerConcept _con2 = OntologyMemberFactory.createConcept(
						(_arg2.m_namespace.equals("") ? m_ontologyNamespace : _arg2.m_namespace), 
						(_arg2.m_sep.equals("") ? m_ontologySep : _arg2.m_sep), 
						_arg2.m_name);
				if (m_reasoner.isSuperConcept(
						_con1, 
						_con2)) {
					_result.m_tribool = TriBool.triTrue;
			    	log("concept1 is supercon of concept2!");
				}
				else if (m_reasoner.conceptExists(_con1) && m_reasoner.conceptExists(_con2)) {
					_result.m_tribool = TriBool.triFalse;
			    	log("concept1 is not a supercon of concept2!");
				}
				else {
			    	log("at least one of the concepts does not exist -> indeterminate!");
				}
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			}
		} else {
			String message = "";
			if (!_arg1ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
				message += "arg1 WME has wrong IDL type. It should be of type ComaConcept, but it is of type: " + _arg1ptr.m_type;
			}
			if (!_arg2ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
				message += "\n arg2 WME has wrong IDL type. It should be of type ComaConcept, but it is of type: " + _arg1ptr.m_type;
			}
			throw new ComaTypeException(message);
		}
    	String _newID = newDataID();
        try {
	    addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
	    _action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(TriBoolResult.class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
			overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}

    private void isInstanceOf(ComaReasonerFunction _action, String _wmeID) {
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
    	WorkingMemoryPointer _arg2ptr = _action.m_arg2ptr;
		TriBoolResult _result = new TriBoolResult();
		_result.m_tribool = TriBool.triIndeterminate;
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class)) &&
				_arg2ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
			try {
				ComaInstance _arg1 = (ComaInstance) getWorkingMemoryEntry(_arg1ptr.m_address).getData();
				ComaConcept _arg2 = (ComaConcept) getWorkingMemoryEntry(_arg2ptr.m_address).getData();

				ReasonerInstance _ins = OntologyMemberFactory.createInstance(
						(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
						(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
						_arg1.m_name);
				ReasonerConcept _con = OntologyMemberFactory.createConcept(
						(_arg2.m_namespace.equals("") ? m_ontologyNamespace : _arg2.m_namespace), 
						(_arg2.m_sep.equals("") ? m_ontologySep : _arg2.m_sep), 
						_arg2.m_name);
				if (m_reasoner.instanceExists(_ins)) {
					// update the instance WME
					_arg1 = refreshInstanceStruct(_arg1);
					overwriteWorkingMemory(_arg1ptr.m_address, 
							_arg1, OperationMode.BLOCKING);
					if (m_reasoner.isInstanceOf(
							_ins, 
							_con)) {
						_result.m_tribool = TriBool.triTrue;
					} else {
						_result.m_tribool = TriBool.triFalse;
					}
				} else {
					_result.m_tribool = TriBool.triIndeterminate;					
				}
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			}
		} else {
			String message = "";
			if (!_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) {
				message += "arg1 WME has wrong IDL type. It should be of type ComaInstance, but it is of type: " + _arg1ptr.m_type;
			}
			if (!_arg2ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
				message += "\n arg2 WME has wrong IDL type. It should be of type ComaConcept, but it is of type: " + _arg1ptr.m_type;
			}
			throw new ComaTypeException(message);
		}
    	String _newID = newDataID();
        try {
	    addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
	    _action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(TriBoolResult.class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
			overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}

    private void getAllCons(ComaReasonerFunction _action, String _wmeID) {
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) {
    		try {
    			ComaInstance _arg1 = (ComaInstance) getWorkingMemoryEntry(_arg1ptr.m_address).getData();

				// update the instance WME
				_arg1 = refreshInstanceStruct(_arg1);
				overwriteWorkingMemory(_arg1ptr.m_address, 
						_arg1, OperationMode.BLOCKING);
    			
    			Set<ReasonerConcept> _resultConceptSet = 
    				m_reasoner.getAllConcepts(
    					OntologyMemberFactory.createInstance(
						(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
						(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
						_arg1.m_name));
    			ComaConcept[] _result = new ComaConcept[_resultConceptSet.size()];	
    			int i = 0;
    			for (ReasonerConcept _currConcept : _resultConceptSet) {
    				_result[i] = new ComaConcept(_currConcept.getNamespace(), _currConcept.getSep(), _currConcept.getName());
    				i++;
    			}

    			String _newID = newDataID();
    			addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
    			_action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept[].class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
    			overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
    		} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
    		}
    	} else {
    		throw new ComaTypeException("arg1 WME has wrong IDL type. It should be of type ComaInstance, but it is of type: " + _arg1ptr.m_type);
    	}
    }

    private void getAllDirectCons(ComaReasonerFunction _action, String _wmeID) {
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) {
    		try {
    			ComaInstance _arg1 = (ComaInstance) getWorkingMemoryEntry(_arg1ptr.m_address).getData();

				// update the instance WME
				_arg1 = refreshInstanceStruct(_arg1);
				overwriteWorkingMemory(_arg1ptr.m_address, 
						_arg1, OperationMode.BLOCKING);
    			
    			Set<ReasonerConcept> _resultSet =
    				m_reasoner.getDirectConcepts(
    					OntologyMemberFactory.createInstance(
    							(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
    							(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
    							_arg1.m_name));	
    			ComaConcept[] _result = new ComaConcept[_resultSet.size()];	
    			int i = 0;
    			for (ReasonerConcept _currConcept : _resultSet) {
    				_result[i] = new ComaConcept(
    						_currConcept.getNamespace(), _currConcept.getSep(), _currConcept.getName());
    				i++;
    			}
    			String _newID = newDataID();
    			addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
    			_action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept[].class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
    			overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
    		} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
    		}
    	} else {
    		throw new ComaTypeException("arg1 WME has wrong IDL type. It should be of type ComaInstance, but it is of type: " + _arg1ptr.m_type);
    	}
    }    

    private void getMostSpecificCons(ComaReasonerFunction _action, String _wmeID) {
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) {
    		try {
    			ComaInstance _arg1 = (ComaInstance) getWorkingMemoryEntry(_arg1ptr.m_address).getData();
    			
				// update the instance WME
				_arg1 = refreshInstanceStruct(_arg1);
				overwriteWorkingMemory(_arg1ptr.m_address, 
						_arg1, OperationMode.BLOCKING);

    			Set<ReasonerConcept> _resultSet =  
    				m_reasoner.getMostSpecificConcepts(
    					OntologyMemberFactory.createInstance(
    							(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
    							(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
    							_arg1.m_name)); 	
    			ComaConcept[] _result = new ComaConcept[_resultSet.size()];	
    			int i = 0;
    			for (ReasonerConcept _currConcept: _resultSet) {
    				_result[i] = new ComaConcept(
    						_currConcept.getNamespace(),_currConcept.getSep(),_currConcept.getName());
    				i++;
    			}
    			String _newID = newDataID();
    			addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
    			_action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept[].class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
    			overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
    		} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
    		}
    	} else {
    		throw new ComaTypeException("arg1 WME has wrong IDL type. It should be of type ComaInstance, but it is of type: " + _arg1ptr.m_type);
    	}
    }    

    
    private void getBasicLevelCons(ComaReasonerFunction _action, String _wmeID) {
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) {
    		try {
    			ComaInstance _arg1 = (ComaInstance) getWorkingMemoryEntry(_arg1ptr.m_address).getData();

				// update the instance WME
				_arg1 = refreshInstanceStruct(_arg1);
				overwriteWorkingMemory(_arg1ptr.m_address, 
						_arg1, OperationMode.BLOCKING);
    			
    			Set<ReasonerConcept> _resultSet =  
    				m_reasoner.getBasicLevelConcepts(
    					OntologyMemberFactory.createInstance(
    							(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
    							(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
    							_arg1.m_name));
    			ComaConcept[] _result = new ComaConcept[_resultSet.size()];	
    			int i = 0;
    			for (ReasonerConcept _currConcept : _resultSet) {
    				_result[i] = new ComaConcept(
    						_currConcept.getNamespace(), _currConcept.getSep(), _currConcept.getName());
    				i++;
    			}
    			String _newID = newDataID();
    			addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
    			_action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept[].class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
    			overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
    		} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
    		}
    	} else {
    		throw new ComaTypeException("arg1 WME has wrong IDL type. It should be of type ComaInstance, but it is of type: " + _arg1ptr.m_type);
    	}
    }    
        
    
    private void getTypicalObjectCons(ComaReasonerFunction _action, String _wmeID) {
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) {
    		try {
    			ComaInstance _arg1 = (ComaInstance) getWorkingMemoryEntry(_arg1ptr.m_address).getData();
    			
				// update the instance WME
				_arg1 = refreshInstanceStruct(_arg1);
				overwriteWorkingMemory(_arg1ptr.m_address, 
						_arg1, OperationMode.BLOCKING);

    			ReasonerInstance _arg1Ins = OntologyMemberFactory.createInstance(
    					(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
    					(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
    					_arg1.m_name);
    			log("getting typical Objects for "+_arg1Ins);
    			ReasonerConcept _areaCon = OntologyMemberFactory.createConcept(
    					m_ontologyNamespace, 
    					m_ontologySep, 
    					"Area");
    			Set<ReasonerConcept> _resultSet = new TreeSet<ReasonerConcept>();
    			if (m_reasoner.isInstanceOf(_arg1Ins, _areaCon)) {
    				for (ReasonerConcept _currConcept : m_reasoner.getMostSpecificConcepts(_arg1Ins)) {
    					ReasonerConcept _compoundConcept = OntologyMemberFactory.createConcept(
    							_currConcept.getNamespace(), 
    							_currConcept.getSep(), 
    							_currConcept.getName()+"Object");
    					_resultSet.addAll(m_reasoner.getAllSubConcepts(_compoundConcept));
    					log("adding all subconcepts of " + _compoundConcept.getName());
    				}
    			}
    			ComaConcept[] _result = new ComaConcept[_resultSet.size()];	
    			int i = 0;
    			for (ReasonerConcept _currResConcept : _resultSet) {
    				_result[i] = new ComaConcept(
    						_currResConcept.getNamespace(), _currResConcept.getSep(), _currResConcept.getName());
    				i++;
    			}
    			String _newID = newDataID();
    			addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
    			_action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept[].class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
    			overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
    		} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
    		}
    	} else {
    		throw new ComaTypeException("arg1 WME has wrong IDL type. It should be of type ComaInstance, but it is of type: " + _arg1ptr.m_type);
    	}
	}
    
    
    private void getObjectMobility(ComaReasonerFunction _action, String _wmeID) {
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
		TriBoolResult _result = new TriBoolResult();
		_result.m_tribool = TriBool.triIndeterminate;
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
			try {
				ComaConcept _arg1 = (ComaConcept) getWorkingMemoryEntry(_arg1ptr.m_address).getData();

				ReasonerConcept _con1 = OntologyMemberFactory.createConcept(
						(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
						(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
						_arg1.m_name);
				if (m_reasoner.isSubConcept(
						_con1, 
						OntologyMemberFactory.createConcept(m_ontologyNamespace, m_ontologySep, "MobileObject"))) {
					_result.m_tribool = TriBool.triTrue;
			    	log(_con1.getName() + " is a mobile object!");
				} else if (m_reasoner.isSubConcept(
						_con1, 
						OntologyMemberFactory.createConcept(m_ontologyNamespace, m_ontologySep, "ImmobileObject"))) {
					_result.m_tribool = TriBool.triFalse;
			    	log(_con1.getName() + " is an immobile object!");
				} else {
			    	log("the concept is neither a mobile nor an immobile object -> indeterminate!");
				}
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			}
    	} else {
    		throw new ComaTypeException("arg1 WME has wrong IDL type. It should be of type ComaConcept, but it is of type: " + _arg1ptr.m_type);
    	}
    	String _newID = newDataID();
        try {
	    addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
			_action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(TriBoolResult.class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
			overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}


    private void getAllInstances(ComaReasonerFunction _action, String _wmeID) {
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
    	WorkingMemoryPointer[] _result = new WorkingMemoryPointer[0];
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
    		try {
    			ComaConcept _arg1 = (ComaConcept) getWorkingMemoryEntry(_arg1ptr.m_address).getData();
    			ReasonerConcept _rcon = OntologyMemberFactory.createConcept(
						(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
						(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
						_arg1.m_name);
    			Set<ReasonerInstance> _instances = m_reasoner.getInstances(_rcon);
//    			log("getting all instances for " + _rcon.getFullName());
    			_result = new WorkingMemoryPointer[_instances.size()];
    			int i = 0;
    			for (ReasonerInstance instance : _instances) {
    				// add pointer to existing instance WME to the result list
    				_result[i] = getInstanceWME(instance);
    				
    				// refresh instance info on WM
    				ComaInstance _wmInstance = (ComaInstance) getWorkingMemoryEntry(_result[i].m_address).getData();
    				_wmInstance = refreshInstanceStruct(_wmInstance);
    				overwriteWorkingMemory(_result[i].m_address, _wmInstance, OperationMode.BLOCKING);
    				i++;
    			}
    		} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
    		}
    	} else {
    		throw new ComaTypeException("arg1 WME has wrong IDL type. It should be of type ComaConcept, but it is of type: " + _arg1ptr.m_type);
    	}
    	String _newID = newDataID();
        try {
        	addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
        	_action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(WorkingMemoryPointer[].class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
        	overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}

    private void getInstancesByName(ComaReasonerFunction _action, String _wmeID) {
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
    	WorkingMemoryPointer[] _result = new WorkingMemoryPointer[0];
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(String.class))) {
    		try {
    			String _arg1 = (String) getWorkingMemoryEntry(_arg1ptr.m_address).getData();
    			Set<ReasonerInstance> _instances = 
    				m_reasoner.getInstancesByName(_arg1);
    			_result = new WorkingMemoryPointer[_instances.size()];
    			int i = 0;
    			for (ReasonerInstance instance : _instances) {
    				// add pointer to existing instance WME to the result list
    				_result[i] = getInstanceWME(instance);
    				
    				// refresh instance info on WM
    				ComaInstance _wmInstance = (ComaInstance) getWorkingMemoryEntry(_result[i].m_address).getData();
    				_wmInstance = refreshInstanceStruct(_wmInstance);
    				overwriteWorkingMemory(_result[i].m_address, _wmInstance, OperationMode.BLOCKING);
    				i++;
    			}
    		} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
    		}
    	} else {
    		throw new ComaTypeException("arg1 WME has wrong IDL type. It should be of type ComaConcept, but it is of type: " + _arg1ptr.m_type);
    	}
    	String _newID = newDataID();
        try {
        	addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
	        _action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(WorkingMemoryPointer[].class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
	        overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}

    /**
     * This is a polymorphic method that will either get
     * ALL instances that are related to a given instance
     * or
     * only those instances that are related to a given instance by a given relation.
     * 
     * For polymorphism the WME pointers of the ComaReasonerFunction are used.
     * For ALL instances use only arg1.
     * If a torso/summy ComaRelation (not containing instances, just a relation label) is
     * passed via the addInfo slot, then this relation will be used to restrict the returned 
     * instances (see above).
     * 
     * @param _action
     * @param _wmeID
     */
    private void getRelatedInstances(ComaReasonerFunction _action, String _wmeID) {
    	log("getRelatedInstances() called...");
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
    	WorkingMemoryPointer _addInfoptr = _action.m_add_info_ptr;
    	Set<ReasonerInstance> _returnedInstances = new TreeSet<ReasonerInstance>();
    	WorkingMemoryPointer[] _result = new WorkingMemoryPointer[0];
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) {
    		try {
    			ComaInstance _arg1 = (ComaInstance) getWorkingMemoryEntry(_arg1ptr.m_address).getData();
    			
				// refresh instance info on WM
				_arg1 = refreshInstanceStruct(_arg1);
				overwriteWorkingMemory(_arg1ptr.m_address, _arg1, OperationMode.BLOCKING);

    			
    			if (_addInfoptr.m_type.equals(CASTUtils.typeName(ComaRelation.class))) {
    				log("Got a COMA_RELATION restriction.");
    				ComaRelation _rel = (ComaRelation) getWorkingMemoryEntry(_addInfoptr.m_address).getData();
    				_returnedInstances = m_reasoner.getRelatedInstances(
    						OntologyMemberFactory.createInstance(
    								(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
    								(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
    								_arg1.m_name),
    						OntologyMemberFactory.createRelation(
    								(_rel.m_namespace.equals("") ? m_ontologyNamespace : _rel.m_namespace), 
    								(_rel.m_sep.equals("") ? m_ontologySep : _rel.m_sep), 
    								_rel.m_name, 
    								null, null));
    			}
    			else _returnedInstances = m_reasoner.getRelatedInstances(
						OntologyMemberFactory.createInstance(
								(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
								(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
								_arg1.m_name));

    			_result = new WorkingMemoryPointer[_returnedInstances.size()];
    			int i = 0;
    			for (ReasonerInstance instance : _returnedInstances) {
    				// add pointer to existing instance WME to the result list
    				_result[i] = getInstanceWME(instance);
    				log("current result["+i+"]: "+instance);
    				// refresh instance info on WM
    				ComaInstance _wmInstance = (ComaInstance) getWorkingMemoryEntry(_result[i].m_address).getData();
    				_wmInstance = refreshInstanceStruct(_wmInstance);
    				overwriteWorkingMemory(_result[i].m_address, _wmInstance, OperationMode.BLOCKING);
    				i++;
    			}
    		} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
    		}
    	} else {
    		throw new ComaTypeException("arg1 WME has wrong IDL type. It should be of type ComaInstance, but it is of type: " + _arg1ptr.m_type);
    	}
    	String _newID = newDataID();
        try {
        	addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
	        _action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(WorkingMemoryPointer[].class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
	        overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}
    
    
    private void addInstance(ComaReasonerFunction _action, String _wmeID) {
    	// do I really have to write back a result? YES! for proper deletion!	
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
    	WorkingMemoryPointer _arg2ptr = _action.m_arg2ptr;
		TriBoolResult _result = new TriBoolResult();
		_result.m_tribool = TriBool.triIndeterminate;
		
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class)) && 
    			_arg2ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
			try {
				ComaInstance _arg1 = (ComaInstance) getWorkingMemoryEntry(_arg1ptr.m_address).getData();
				ReasonerInstance _ins = OntologyMemberFactory.createInstance(
						(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
						(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
						_arg1.m_name);
					
				ComaConcept _arg2 = (ComaConcept) getWorkingMemoryEntry(_arg2ptr.m_address).getData();
				ReasonerConcept _con = OntologyMemberFactory.createConcept(
						(_arg2.m_namespace.equals("") ? m_ontologyNamespace : _arg2.m_namespace), 
						(_arg2.m_sep.equals("") ? m_ontologySep : _arg2.m_sep), 
						_arg2.m_name);
				
				log(_con.getFullName());
				
				try {
					m_reasoner.addInstance(_ins, _con);
					
    				// refresh instance info on WM
    				_arg1 = refreshInstanceStruct(_arg1);
    				overwriteWorkingMemory(_arg1ptr.m_address, _arg1, OperationMode.BLOCKING);

					
				} catch (ReasonerException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				_result.m_tribool = TriBool.triTrue;
			} catch (SubarchitectureProcessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				_result.m_tribool = TriBool.triFalse;
			}
		} else {
			String message = "";
			if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) {
				message += "arg1 WME has wrong IDL type. It should be of type ComaInstance, but it is of type: " + _arg1ptr.m_type;
			}
			if (_arg2ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class))) {
				message += "\n arg2 WME has wrong IDL type. It should be of type ComaConcept, but it is of type: " + _arg1ptr.m_type;
			}
			throw new ComaTypeException(message);
		}
    	String _newID = newDataID();
        try {
        	addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
			_action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(TriBoolResult.class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
			overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}

    private void deleteInstance(ComaReasonerFunction _action, String _wmeID) {
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
		TriBoolResult _result = new TriBoolResult();
		_result.m_tribool = TriBool.triIndeterminate;
		
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) {
			try {
				ComaInstance _arg1 = (ComaInstance) getWorkingMemoryEntry(_arg1ptr.m_address).getData();
				ReasonerInstance _ins = OntologyMemberFactory.createInstance(
						(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
						(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
						_arg1.m_name);

				boolean successful = m_reasoner.deleteInstance(_ins);
				_result.m_tribool = (successful ? TriBool.triTrue : TriBool.triFalse);
			} catch (SubarchitectureProcessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				_result.m_tribool = TriBool.triFalse;
			}
		} else {
			String message = "";
			if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) {
				message += "arg1 WME has wrong IDL type. It should be of type ComaInstance, but it is of type: " + _arg1ptr.m_type;
			}
			throw new ComaTypeException(message);
		}
    	String _newID = newDataID();
        try {
        	addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
			_action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(TriBoolResult.class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
			overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}

    
    
    private void addRelation(ComaReasonerFunction _action, String _wmeID) {
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;

    	TriBoolResult _result = new TriBoolResult();
		_result.m_tribool = TriBool.triIndeterminate;
		
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaRelation.class))) {
			try {
				ComaRelation _rel = (ComaRelation) getWorkingMemoryEntry(_arg1ptr.m_address).getData();
				
				ComaInstance _ins1 = (ComaInstance) getWorkingMemoryEntry(_rel.m_ins1ptr.m_address).getData();
				ComaInstance _ins2 = (ComaInstance) getWorkingMemoryEntry(_rel.m_ins2ptr.m_address).getData();
				
				// refresh instance info on WM
				_ins1 = refreshInstanceStruct(_ins1);
				_ins2 = refreshInstanceStruct(_ins2);
				overwriteWorkingMemory(_rel.m_ins1ptr.m_address, _ins1, OperationMode.BLOCKING);
				overwriteWorkingMemory(_rel.m_ins2ptr.m_address, _ins2, OperationMode.BLOCKING);

				
				ReasonerRelation relation = OntologyMemberFactory.createRelation(
						(_rel.m_namespace.equals("") ? m_ontologyNamespace : _rel.m_namespace), 
						(_rel.m_sep.equals("") ? m_ontologySep : _rel.m_sep), 
						_rel.m_name, 
						OntologyMemberFactory.createInstance(
								(_ins1.m_namespace.equals("") ? m_ontologyNamespace : _ins1.m_namespace), 
								(_ins1.m_sep.equals("") ? m_ontologySep : _ins1.m_sep), 
								_ins1.m_name),
						OntologyMemberFactory.createInstance(
								(_ins2.m_namespace.equals("") ? m_ontologyNamespace : _ins2.m_namespace), 
								(_ins2.m_sep.equals("") ? m_ontologySep : _ins2.m_sep), 
								_ins2.m_name));

				m_reasoner.assertRelation(relation);
				
				_result.m_tribool = TriBool.triTrue;
			} catch (SubarchitectureProcessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				_result.m_tribool = TriBool.triFalse;
			}
    	} else {
    		throw new ComaTypeException("arg1 WME has wrong IDL type. It should be of type ComaRelation, but it is of type: " + _arg1ptr.m_type);
    	}
    	String _newID = newDataID();
        try {
        	addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
			_action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(TriBoolResult.class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
			overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}

    
    private void addInstanceName(ComaReasonerFunction _action, String _wmeID) {
    	TriBoolResult _result = new TriBoolResult();
		_result.m_tribool = TriBool.triIndeterminate;
		
    	if (_action.m_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))
    			&& _action.m_arg2ptr.m_type.equals(CASTUtils.typeName(StringWrapper.class))) {
			try {
				ComaInstance _ins = (ComaInstance) getWorkingMemoryEntry(_action.m_arg1ptr.m_address).getData();
				String _name = ((StringWrapper) getWorkingMemoryEntry(_action.m_arg2ptr.m_address).getData()).m_string;
				
				ReasonerInstance _instance = OntologyMemberFactory.createInstance(
						(_ins.m_namespace.equals("") ? m_ontologyNamespace : _ins.m_namespace), 
						(_ins.m_sep.equals("") ? m_ontologySep : _ins.m_sep), 
						_ins.m_name);
				m_reasoner.addName(_instance, _name);
				
				// refresh instance info on WM
				_ins = refreshInstanceStruct(_ins);
				overwriteWorkingMemory(_action.m_arg1ptr.m_address, _ins, OperationMode.BLOCKING);
				
				_result.m_tribool = TriBool.triTrue;
			} catch (SubarchitectureProcessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				_result.m_tribool = TriBool.triFalse;
			}
    	} else {
    		throw new ComaTypeException("arg1 or _arg2 WME has wrong IDL type. It should be of type ComaInstance or StringWrapper resp.");
    	}
    	String _newID = newDataID();
        try {
	    addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
			_action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(TriBoolResult.class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
			overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}
    
    private void addInstanceNumberTag(ComaReasonerFunction _action, String _wmeID) {
    	TriBoolResult _result = new TriBoolResult();
		_result.m_tribool = TriBool.triIndeterminate;
		
    	if (_action.m_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))
    			&& _action.m_arg2ptr.m_type.equals(CASTUtils.typeName(StringWrapper.class))) {
			try {
				ComaInstance _ins = (ComaInstance) getWorkingMemoryEntry(_action.m_arg1ptr.m_address).getData();
				String _number = ((StringWrapper) getWorkingMemoryEntry(_action.m_arg2ptr.m_address).getData()).m_string;
				ReasonerInstance _instance = OntologyMemberFactory.createInstance(
						(_ins.m_namespace.equals("") ? m_ontologyNamespace : _ins.m_namespace), 
						(_ins.m_sep.equals("") ? m_ontologySep : _ins.m_sep), 
						_ins.m_name);
				m_reasoner.addNumberTag(_instance, Integer.parseInt(_number));
				
				// refresh instance info on WM
				_ins = refreshInstanceStruct(_ins);
				overwriteWorkingMemory(_action.m_arg1ptr.m_address, _ins, OperationMode.BLOCKING);
				
				_result.m_tribool = TriBool.triTrue;
			} catch (SubarchitectureProcessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				_result.m_tribool = TriBool.triFalse;
			}
    	} else {
    		throw new ComaTypeException("arg1 or _arg2 WME has wrong IDL type. It should be of type ComaInstance or StringWrapper resp.");
    	}
    	String _newID = newDataID();
        try {
	    addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
			_action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(TriBoolResult.class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
			overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}

    
    private void generateRefEx(ComaReasonerFunction _action, String _wmeID) {
    	log("got a GenerateRefEx task! Processing...");
    	WorkingMemoryPointer _arg1ptr = _action.m_arg1ptr;
    	WorkingMemoryPointer _arg2ptr = _action.m_arg2ptr;
		StringWrapper _result = new StringWrapper("");
		
    	if (_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class)) &&
				_arg2ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) {
			try {
				ComaInstance _arg1 = (ComaInstance) getWorkingMemoryEntry(_arg1ptr.m_address).getData();
				ComaInstance _arg2 = (ComaInstance) getWorkingMemoryEntry(_arg2ptr.m_address).getData();

				ReasonerInstance _intdRef = OntologyMemberFactory.createInstance(
						(_arg1.m_namespace.equals("") ? m_ontologyNamespace : _arg1.m_namespace), 
						(_arg1.m_sep.equals("") ? m_ontologySep : _arg1.m_sep), 
						_arg1.m_name);
				ReasonerInstance _origin = OntologyMemberFactory.createInstance(
						(_arg2.m_namespace.equals("") ? m_ontologyNamespace : _arg2.m_namespace), 
						(_arg2.m_sep.equals("") ? m_ontologySep : _arg2.m_sep), 
						_arg2.m_name);
				if (m_reasoner.instanceExists(_intdRef) && m_reasoner.instanceExists(_origin)) {
					// update the instance WME
					_arg1 = refreshInstanceStruct(_arg1);
					_arg2 = refreshInstanceStruct(_arg2);

					ComaGREAlgorithm _refExGenerator = new ComaGREAlgorithm(this);
					String refEx = _refExGenerator.generateRefEx(_intdRef, _origin);
					
					_result.m_string = refEx;
				}
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			}
		} else {
			String message = "";
			if (!_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) {
				message += "arg1 WME has wrong IDL type. It should be of type ComaInstance, but it is of type: " + _arg1ptr.m_type;
			}
			if (!_arg2ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) {
				message += "\n arg2 WME has wrong IDL type. It should be of type ComaInstance, but it is of type: " + _arg1ptr.m_type;
			}
			throw new ComaTypeException(message);
		}
    	String _newID = newDataID();
        try {
	    addToWorkingMemory(_newID, _result, OperationMode.BLOCKING);
	    _action.m_resultptr=new WorkingMemoryPointer(CASTUtils.typeName(StringWrapper.class), new WorkingMemoryAddress(_newID,m_subarchitectureID));
			overwriteWorkingMemory(_wmeID, _action, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}
    
    
    
	/**
	 * This method tests whether a given concept already exists on WM.
	 * If so, it returns the corresponding WME.
	 * Otherwise, it returns null.
	 *
	 * @param _con
	 * @return
	 */
	public WorkingMemoryPointer getConceptWMP(ReasonerConcept _con) {
		try {
//			m_parentcomponent.log(m_parentcomponent);
//			m_parentcomponent.log(m_subarchitectureID);
//			m_parentcomponent.log(m_parentcomponent.getWorkingMemoryEntries(m_subarchitectureID, ComaConcept.class).length);
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
			e.printStackTrace();
			throw new RuntimeException(e);
		}
		return null;
	}
	
	
	
	
	public WorkingMemoryPointer getUptodateInstanceWMP(ReasonerInstance _ins) throws SubarchitectureProcessException {
		log("getUptodateInstanceWMP("+_ins.getFullName()+") called.");
		WorkingMemoryPointer _insWMP = getInstanceWME(_ins);
		log("getInstanceWMP returned " + _insWMP);
		log("this _insWMP has type " + _insWMP.m_type);
		
		// refresh instance info on WM
		ComaInstance _wmInstance = (ComaInstance) getWorkingMemoryEntry(_insWMP.m_address).getData();
		_wmInstance = refreshInstanceStruct(_wmInstance);
		overwriteWorkingMemory(_insWMP.m_address, _wmInstance, OperationMode.BLOCKING);
		
		return _insWMP;
	}
	
	private ComaInstance refreshInstanceStruct(ComaInstance _comaIns) {
		// create an instance for querying the reasoner
		ReasonerInstance _reasonerIns = OntologyMemberFactory.createInstance(
				(_comaIns.m_namespace.equals("") ? m_ontologyNamespace : _comaIns.m_namespace), 
				(_comaIns.m_sep.equals("") ? m_ontologySep : _comaIns.m_sep), 
				_comaIns.m_name);

		// update the fields of the instance struct

		// name
		Set<String> _iNames = m_reasoner.getNames(_reasonerIns);
		_comaIns.m_givenNames=_iNames.toArray(new String[_iNames.size()]);
		
		// most specific concepts
		Set<ReasonerConcept> _mostSpecCons = m_reasoner.getMostSpecificConcepts(_reasonerIns);
		String[] _mostSpecConNames = new String[_mostSpecCons.size()];
		int j=0;
		for (ReasonerConcept string : _mostSpecCons) {
			_mostSpecConNames[j] = string.getName();
			j++;
		}
		_comaIns.m_mostSpecificConcepts = _mostSpecConNames;
		
		// get full namespace and separator
		_comaIns.m_namespace=_reasonerIns.getNamespace();
		_comaIns.m_sep=_reasonerIns.getSep();

		return _comaIns;
	}
	

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskAdopted(java.lang.String)
     */
    @Override
    protected void taskAdopted(String _taskID) {}

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskRejected(java.lang.String)
     */
    @Override
    protected void taskRejected(String _taskID) {}

}