package org.cognitivesystems.comsys.processing;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Properties;

import org.cognitivesystems.reasoner.base.OntologyMemberFactory;
import org.cognitivesystems.reasoner.base.ReasonerInstance;

import binding.common.BindingComponentException;
import binding.util.BindingUtils;

import coma.aux.ComaFunctionWriter;

import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.DoesNotExistOnWMException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import cast.core.data.CASTData;
import BindingData.BindingProxy;
import BindingData.BindingUnion;
import BindingData.FeaturePointer;
import BindingFeatures.AreaID;
import BindingFeatures.Concept;
import BindingFeatures.RelationLabel;
import BindingFeatures.TemporalFrame;
import BindingFeaturesCommon.TemporalFrameType;
import ComaData.ComaReasonerFunction;
import ComaData.StringWrapper;

/**
 * The class <b>ComaGREProcess</b> implements a GRE
 * algorithm backed by the A-Box and T-Box reasoning facilities 
 * of the conceptual spatial mapping subarchitecture (coma.sa). 
 * 
 * @started 080811
 * @version 080814
 * @author Hendrik Zender (zender@dfki.de)
 * 
 */
public class ComaGREProcess extends AbstractGREProcess {

	String m_comaSubarchitectureID;
	ComaFunctionWriter m_comaFunctionWriter;
	HashMap<String, GRETaskWrapper> m_comaWMID2GRETaskMap;
	
	public ComaGREProcess(String _id) {
		super(_id);
		m_comaFunctionWriter = new ComaFunctionWriter(this);
		m_comaWMID2GRETaskMap = new HashMap<String, GRETaskWrapper>();
	}
	
    public void configure(Properties _config) {
    	super.configure(_config);
        if (_config.containsKey("--csa")) {
            this.m_comaSubarchitectureID = _config.getProperty("--csa");
        } else if (_config.containsKey("-csa")) {
            this.m_comaSubarchitectureID = _config.getProperty("-csa");
        } else {
        	throw new RuntimeException("You must specify the coma SA ID: --csa <SA-ID> or -csa <SA-ID>!");
        }
        m_comaFunctionWriter.setComaSubarchitectureID(m_comaSubarchitectureID);
    }


	@Override
	public void processGRETask(GRETaskWrapper _myWrappedTask) {
		// OK, now first let's read in the intended referent proxy
		BindingProxy _intdRefProxy = null;
		try {
			log("Trying to get intended referent proxy at: " + _myWrappedTask.getIntendedRefProxyAdress().m_id);
			_intdRefProxy = this.getProxy(_myWrappedTask.getIntendedRefProxyAdress().m_id);
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
//		try {
//			_intdRefProxy = (BindingProxy) getWorkingMemoryEntry(_myWrappedTask.getIntendedRefProxyAdress()).getData();
//		} catch (SubarchitectureProcessException e) {
//			e.printStackTrace();
//			log("Error! Could not read intended referent proxy. Aborting!");
//			throw new RuntimeException(e);
//		}
		
		// try to access the union to get some feature that helps coma
		// to identify the referent's coma instance.
		BindingUnion _intdRefUnion = null;

		while(_intdRefUnion==null) {
			log("trying to read intended referent union...");
			try {
				log("now accessing the proxy's union.");
				_intdRefUnion = this.getUnion(_intdRefProxy.m_unionID);
			} catch (SubarchitectureProcessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
//		while (_intdRefUnion==null) {
//			try {
//				_intdRefUnion = (BindingUnion) getWorkingMemoryEntry(_intdRefProxy.m_unionID,_myWrappedTask.getIntendedRefProxyAdress().m_subarchitecture).getData();
//			} catch (SubarchitectureProcessException e) {
//				log("argh! union disappeared... trying again!");
//			}
//		}

		// now check the features of the union
		FeaturePointer[] _unionFts = _intdRefUnion.m_unionFeatures;
		
		ReasonerInstance _intendedReferentInstance = null;
		
		// here is room for other features
		try {
			for (AreaID _areaID : BindingUtils.getBindingFeatures(this, _intdRefUnion, AreaID.class)) {
				// I suppose it is only one area ID anyway...
				_intendedReferentInstance = OntologyMemberFactory.createInstance("oe", ":", "area"+_areaID.m_id);
				log("there is an AreaID feature; generated intRefInstance on basis of area ID -> " + _intendedReferentInstance.getName());
			}
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			log("could not access intended referent proxy!");
			// could not identify my intended referent!
			_myWrappedTask.setResultLF("");
			returnRefExLF(_myWrappedTask);
			return;
		}
		// eg object ID -- or person ID ???? whatever! maybe name...?
		
		
		if (_intendedReferentInstance!=null) {
			// I know the intended referent
			// now let's look for the referential anchor!
			log("now looking for referential anchor; defaulting to robot's current position.");
			
			// origin needs to be the robot's position right now:
			// 1) find robot
			// 2) get its "position" relation
			// 3) get the related areaID proxy
			// NEW CODE from ReferenceResolver!
			// resolve discourse anchor
			// if no anchor is specified, the robot's current location is assumed
			// if that cannot be resolves, the world is assumed as referential anchor
			ReasonerInstance _discourseAnchor = OntologyMemberFactory.createInstance(
					"oe", ":", "world");
			boolean _anchorProperlySet = false;

			
			// commented out Sep 18 2008 (hz) -- this looks like the old code
			// with access to the position relation via the union -- not good ;-)
			// replaced with newer code that goes directly via the robot proxy!
//			BindingProxy _robotSelfProxy = null;
//			try {
//				CASTData<BindingProxy>[] _allProxies =  
//					getWorkingMemoryEntries(getBindingSA(), BindingProxy.class);
//				
//				for (CASTData<BindingProxy> data : _allProxies) {
//					BindingProxy _currProxy = (BindingProxy) data.getData();
//					
//					// this needs to be changed if there is a consistent representation of SELF
//					ArrayList<Concept> _allConFts = BindingUtils.getBindingFeatures(
//							this, getBindingSA(), _currProxy, Concept.class);
//					
//					for (Concept _currConcept : _allConFts) {
//						if (_currConcept.m_concept.equalsIgnoreCase("robot")) {
//							_robotSelfProxy = _currProxy;
//							break;
//						}
//					}
//					
//					if (_robotSelfProxy!=null) break;
//				}
//				
//				// ok we now have the robot's self proxy
//				// need to access its union to get to its position relatee
//				ArrayList<BindingProxy> _allRobotRels = BindingUtils.getRelationsFromUnion(
//						this, getBindingSA(), 
//						BindingUtils.getUnion(this, getBindingSA(), _robotSelfProxy));
//				
//				for (BindingProxy _currRel : _allRobotRels) {
//					ArrayList<RelationLabel> _allRelLabels = BindingUtils.getBindingFeatures(
//							this, getBindingSA(), _currRel, RelationLabel.class);
//					for (RelationLabel _currLabel : _allRelLabels) {
//						if (_currLabel.m_label.equalsIgnoreCase("position")) {
//							ArrayList<CASTData<BindingProxy>> _allPositionRelatees = BindingUtils.getProxiesViaToPort(
//									this, getBindingSA(), _currRel);
//							for (CASTData<BindingProxy> data : _allPositionRelatees) {
//								BindingProxy _currRelatee = data.getData();
//								
//								// now check all possible features that identify a proxy that
//								// has a coma instance counterpart
//								
//								// AreaID
//								ArrayList<AreaID> _allAreaIDs = BindingUtils.getBindingFeatures(
//										this, getBindingSA(), _currRelatee, AreaID.class);
//								for (AreaID _currAreaID : _allAreaIDs) {
//									String _areaInsName = "area" + _currAreaID;
//									_discourseAnchor = OntologyMemberFactory.createInstance(
//											"oe", ":", _areaInsName);
//									_anchorProperlySet = true;
//									break;
//								}
//								
//								
//								if (_anchorProperlySet) break;
//							}
//							if (_anchorProperlySet) break;
//						}
//						if (_anchorProperlySet) break;
//					}
//					if (_anchorProperlySet) break;
//				}

			// new code, pasted ftom ComaReferenceResolver Sep 18 2008
			// resolve discourse anchor
			// if no anchor is specified, the robot's current location is assumed
			BindingProxy _robotSelfProxy = null;
			try {
				CASTData<BindingProxy>[] _allProxies =  
					getWorkingMemoryEntries(getBindingSA(), BindingProxy.class);
				
				for (CASTData<BindingProxy> data : _allProxies) {
					BindingProxy _currProxy = (BindingProxy) data.getData();
					
					// this needs to be changed if there is a consistent representation of SELF
					ArrayList<Concept> _allConFts = BindingUtils.getBindingFeatures(
							this, getBindingSA(), _currProxy, Concept.class);
					
					for (Concept _currConcept : _allConFts) {
						if (_currConcept.m_concept.equalsIgnoreCase("robot")) {
							_robotSelfProxy = _currProxy;
							break;
						}
					}
					
					if (_robotSelfProxy!=null) break;
				}
				
				ArrayList<BindingProxy> _allRobotRels = null;
				// ok we now have the robot's self proxy
				// try to get its relation via the proxy itself first
				try {
					ArrayList<BindingProxy> _allRobotPRels = BindingUtils.getRelations(this, getBindingSA(), _robotSelfProxy);
					_allRobotRels = _allRobotPRels;
					// look at the relations to check if it has the PERCEIVED position
					for (BindingProxy _directProxyRel : _allRobotRels) {
						TemporalFrame _currTempFr = BindingUtils.getBindingFeature(this, getBindingSA(), _directProxyRel, TemporalFrame.class);
						if (_currTempFr.m_temporalFrame.equals(TemporalFrameType.PERCEIVED)) {
							// ok the PERCEIVED position is the robot's current location!
							ArrayList<RelationLabel> _labels = BindingUtils.getBindingFeatures(this, getBindingSA(),
									_directProxyRel, RelationLabel.class);
							for (RelationLabel _currLabel : _labels) {
								if (_currLabel.m_label.equalsIgnoreCase("position")) {

									ArrayList<CASTData<BindingProxy>> _allPositionRelatees = BindingUtils.getProxiesViaToPort(
											this, getBindingSA(), _directProxyRel);
									for (CASTData<BindingProxy> data : _allPositionRelatees) {
										BindingProxy _currRelatee = data.getData();
										
										// now check all possible features that identify a proxy that
										// has a coma instance counterpart
										
										// AreaID
										ArrayList<AreaID> _allAreaIDs = BindingUtils.getBindingFeatures(
												this, getBindingSA(), _currRelatee, AreaID.class);
										for (AreaID _currAreaID : _allAreaIDs) {
											String _areaInsName = "area" + _currAreaID;
											_discourseAnchor = OntologyMemberFactory.createInstance(
													"oe", ":", _areaInsName);
											_anchorProperlySet = true;
											break;
										}
										if (_anchorProperlySet) break;
									}
									if (_anchorProperlySet) break;
								} else continue; // no "position"
							}
						} else {
							continue;
						}
						if (_anchorProperlySet) break;
					}
				}
				catch (DoesNotExistOnWMException e) {
					log("could not read relation proxy. here's the trace:" + e.getStackTrace());
					throw new SubarchitectureProcessException("just to trigger achor setting...");
					
				}			
		
		
		
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				log("Cannot resolve dialogue anchor. Resolving against " + _discourseAnchor + " instead...");
				return;
			}
			// end: attempt to resovle discourse anchor
			log("discourse anchor = " + _discourseAnchor.getFullName());
			
			
			
			
			log("dispatching GRE task to coma.");
			// dispatch that task to coma
			// (coma will generate the full LF string itself and just return that string)
			m_comaWMID2GRETaskMap.put(m_comaFunctionWriter.generateRefEx(
	        		new WorkingMemoryChangeReceiver() {
					  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					  		handleRefExGenerated(_wmc);} },  
					_intendedReferentInstance, _discourseAnchor, m_comaSubarchitectureID),
					_myWrappedTask);
			log("successfully dispatched GRETask to coma");
			return;

			
//			 OLD CODE :
//			try {
//				for (CASTData<BindingProxy> _currProxyData : getWorkingMemoryEntries(getBindingSA(),BindingProxy.class)) {
//					BindingProxy _currProxy = _currProxyData.getData();
//					
//					for (FeaturePointer _currProxyFtPt : _currProxy.m_proxyFeatures) {
//						if (_currProxyFtPt.m_type.equals(CASTUtils.typeName(Concept.class))) {
//							String _con = ((Concept) getWorkingMemoryEntry(_currProxyFtPt.m_address,getBindingSA()).getData()).m_concept;
//							if (_con.contains("robot")) {
//								// ok, this ME -- the robot ;-)
//								// aka _currProxy
//								ReasonerInstance _originInstance = null;
//								
//								// read inports struct
//								for (BindingProxy _currRelProxy : BindingUtils.getRelations(this, _currProxy)) {
//									for (RelationLabel _currLabel : BindingUtils.getBindingFeatures(this, _currRelProxy, RelationLabel.class)) {
//										if (_currLabel.m_label.equalsIgnoreCase("location")) {
//											// ok, we have the location relation
//											// now get the location
//											for (CASTData<BindingProxy> _currToLocProxyData : BindingUtils.getProxiesViaToPort(this, _currRelProxy)) {
//												BindingProxy _currToLocProxy = _currToLocProxyData.getData();
//												for (AreaID _areaID : BindingUtils.getBindingFeatures(this, _currToLocProxy, AreaID.class)) {
//													// I suppose it is only one area ID anyway...
//													_originInstance = OntologyMemberFactory.createInstance("oe", ":", "area"+_areaID.m_id);
//												}
//											}
//										}
//									}
//								}
//								
//								if (_originInstance!=null) {
//									log("dispatching GRE task to coma.");
//									// dispatch that task to coma
//									// (coma will generate the full LF string itself and just return that string)
//									m_comaWMID2GRETaskMap.put(m_comaFunctionWriter.generateRefEx(
//							        		new WorkingMemoryChangeReceiver() {
//											  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
//											  		handleRefExGenerated(_wmc);} },  
//											_intendedReferentInstance, _originInstance),
//											_myWrappedTask);
//									log("successfully dispatched GRETask to coma");
//									return;
//								}
//								
//							}
//						}
//					}
//				}
//			} catch (BindingComponentException e) {
//				e.printStackTrace();
//				log("Could not resolve origin proxy!");
//				_myWrappedTask.setResultLF("");
//				returnRefExLF(_myWrappedTask);
//				return;
//			} catch (SubarchitectureProcessException e) {
//				e.printStackTrace();
//				log("Could not resolve origin proxy!");
//				_myWrappedTask.setResultLF("");
//				returnRefExLF(_myWrappedTask);
//				return;
//			}
		} else {
			log("unsuccessful in identifying the inteded referent: returning empty string GRE...");
			// could not identify my intended referent!
			_myWrappedTask.setResultLF("");
			returnRefExLF(_myWrappedTask);
			return;
		}
	}
	
	public void handleRefExGenerated(WorkingMemoryChange _wmc) {
        CASTData<?> wme;
		try {
			wme = getWorkingMemoryEntry(_wmc.m_address);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
		ComaReasonerFunction _returnedFunction = (ComaReasonerFunction) wme.getData();
		
		// now load the result LF string
		try {
			StringWrapper _resultRefEx = (StringWrapper) getWorkingMemoryEntry(_returnedFunction.m_resultptr.m_address).getData();
			GRETaskWrapper _myWrappedTask = m_comaWMID2GRETaskMap.remove(_wmc.m_address.m_id);
			_myWrappedTask.setResultLF(_resultRefEx.m_string);
			returnRefExLF(_myWrappedTask);
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}


}

//		for (int i = 0; i < _unionFts.length; i++) {
//			FeaturePointer pointer = _unionFts[i];
//			
//			if (pointer.m_type.equals(CASTUtils.typeName(AreaID.class))) {
//				// ok, have an area ID
//				// I can use that in coma
//				int _areaID = ((AreaID) getWorkingMemoryEntry(pointer.m_address,getBindingSA()).getData()).m_id;
//				
//				// now I need to know the current "position" in context
//				// for now I will assume the robot's current location
//				for (CASTData<BindingProxy> _currProxyData : getWorkingMemoryEntries(getBindingSA(),BindingProxy.class)) {
//					BindingProxy _currProxy = _currProxyData.getData();
//					
//					for (FeaturePointer _currProxyFtPt : _currProxy.m_proxyFeatures) {
//						if (_currProxyFtPt.m_type.equals(CASTUtils.typeName(Concept.class))) {
//							String _con = ((Concept) getWorkingMemoryEntry(_currProxyFtPt.m_address,getBindingSA()).getData()).m_concept;
//							if (_con.contains("robot")) {
//								// ok, this ME -- the robot ;-)
//								// read inports struct
//								ProxyPorts _currProxyPorts = (ProxyPorts) getWorkingMemoryEntry(getBindingSA(), _currProxy. m_inPortsID).getData();
//								for (ProxyPort _currPort : _currProxyPorts.m_ports) {
//									BindingProxy _currRelProxy = (BindingProxy) getWorkingMemoryEntry(_currPort.m_proxyID, getBindingSA()).getData();
//									// ok I now have the relation
//									for (FeaturePointer _currRelFtPt : _currRelProxy.m_proxyFeatures) {
//										if (_currRelFtPt.m_type.equals(CASTUtils.typeName(RelationLabel.class))) {
//											String _label = ((RelationLabel) getWorkingMemoryEntry(_currRelFtPt.m_address,getBindingSA()).getData()).;
//											g
//										}
//									}
//								}
//								
//								
//							}
//						}
//					}
//				}