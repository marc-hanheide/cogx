package coma.components;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import com.sun.source.tree.NewClassTree;


import binder.abstr.BindingWorkingMemoryWriter;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Proxy;
import binder.autogen.featvalues.StringValue;
import binder.utils.ProbabilityUtils;

import comadata.ComaReasonerInterfacePrx;

import Ice.Identity;
import Ice.ObjectPrx;
import Marshalling.MarshallerPrx;
import Marshalling.Marshaller;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import SpatialProperties.ConnectivityPathProperty;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTComponent;

/**
 * This is a simple monitor that replicates spatial WM entries, *PLACES*, inside coma.
 * **IMPORTANT** This must be adapted:
 * As of now, the OLD FNode WMEs are used in order to have access to properties.
 * 
 * This class, for the time being, also contains the interface to the mighty *BINDER*!
 * 
 * @author hendrik
 * CAST file arguments:
 * --reasoner_id <ComaReasoner>
 * -bsa <binder SA>
 */
public class PlaceMonitor extends ManagedComponent {


	
	Identity m_reasoner_id;
	ComaReasonerInterfacePrx m_comareasoner;
	
	String m_marshaller_component_name;
	MarshallerPrx m_proxyMarshall;
	
	String m_bindingSA;
	//Map<String, String> m_placeID2proxyWMID;
	// removed place proxy handling
	//	Map<Long,ComaPlace> m_placeID2ComaPlace;
	HashSet<Long> m_placeholders;
	HashMap<Long, HashSet<WorkingMemoryAddress>> m_tempAdjacencyStore;
	

	public void configure(Map<String, String> args) {
		log("configure() called");
		m_reasoner_id = new Identity();
		m_reasoner_id.name="";
		m_reasoner_id.category="ComaReasoner";

		if (args.containsKey("--reasoner-name")) {
			m_reasoner_id.name=args.get("--reasoner-name");
		}
		
		m_marshaller_component_name = ""; 
		
		if (args.containsKey("--marshaller-name")) {
			m_marshaller_component_name=args.get("--marshaller-name");
		}

		
		if (args.containsKey("-bsa")) {
			m_bindingSA=args.get("-bsa");
		} else if (args.containsKey("--binding-sa")) {
			m_bindingSA=args.get("--binding-sa");
		} else if (args.containsKey("--bsa")) {
			m_bindingSA=args.get("--bsa");
		}
	
		m_placeholders = new HashSet<Long>();
		m_tempAdjacencyStore = new HashMap<Long, HashSet<WorkingMemoryAddress>>();
		// removed place proxy handling
//		if (m_bindingSA!=null) {
////			m_placeID2proxyWMID = new HashMap<String, String>();
//			m_placeID2ComaPlace = new HashMap<Long, ComaPlace>();
//		}
		
	}
	
	
	public void start() {
		
		// register the monitoring change filters
		// this is the "proper" Place monitor
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				log("Got a callback for an ADDed Place WME!");
				try {
					Place _newPlaceNode = getMemoryEntry(_wmc.address, Place.class);
					log("Place ID = " + _newPlaceNode.id);
					log("Place status = " + (_newPlaceNode.status.equals(PlaceStatus.PLACEHOLDER) ? "PLACEHOLDER" : "TRUEPLACE"));

					if (_newPlaceNode.status.equals(PlaceStatus.TRUEPLACE)) {
						log("create test:place instance");
						m_comareasoner.addInstance("test:place"+_newPlaceNode.id, "test:Place");
						log("Added new Place instance. This is a list of all instances in the ABox");
						for (String _currIns : m_comareasoner.getAllInstances("owl:Thing")) {
							_currIns="test"+_currIns.replaceFirst("http://test.hz.owl#", ":");
							log("all instances // current instance: " + _currIns);
							for (String _currRelIns : m_comareasoner.getRelatedInstances(_currIns)) {
								log("all related instances of " + _currIns + " // " + _currRelIns);
							}
						}
						for (String _currIns : m_comareasoner.getAllInstances("test:Place")) {
							_currIns="test"+_currIns.replaceFirst("http://test.hz.owl#", ":");
							log("all instances of test:Place // current instance: " + _currIns);
							for (String _currRelIns : m_comareasoner.getRelatedInstances(_currIns)) {
								log("all related instances of " + _currIns + " // " + _currRelIns);
							}
							for (String _currRelIns : m_comareasoner.getRelatedInstances(_currIns)) {
								log("all related instances of " + _currIns + " // " + _currRelIns);
							}
						}
						for (String _currIns : m_comareasoner.getAllInstances("test:PhysicalRoom")) {
							_currIns="test"+_currIns.replaceFirst("http://test.hz.owl#", ":");
							log("all instances of test:Place // current instance: " + _currIns);
							for (String _currRelIns : m_comareasoner.getRelatedInstances(_currIns)) {
								log("all related instances of " + _currIns + " // " + _currRelIns);
							}
						}
					} else { // PLACEHOLDER block
						log("going to add " + _newPlaceNode.id + " to m_plaxceholders");
						m_placeholders.add(_newPlaceNode.id);
						log("added " + _newPlaceNode.id + " to m_plaxceholders");
						addChangeFilter(ChangeFilterFactory.createAddressFilter(_wmc.address, WorkingMemoryOperation.OVERWRITE),
								new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
								log("Got a callback for an OVERWRITTEN former Placeholder WME!");
								try {
									Place _newPlaceNode = getMemoryEntry(_wmc.address, Place.class);
									log("Place ID = " + _newPlaceNode.id);
									log("Place status = " + (_newPlaceNode.status.equals(PlaceStatus.PLACEHOLDER) ? "PLACEHOLDER" : "TRUEPLACE"));

									if (_newPlaceNode.status.equals(PlaceStatus.TRUEPLACE)) {
										log("create test:place instance");
										m_comareasoner.addInstance("test:place"+_newPlaceNode.id, "test:Place");
										for (String _currIns : m_comareasoner.getAllInstances("owl:Thing")) {
											_currIns="test"+_currIns.replaceFirst("http://test.hz.owl#", ":");
											log("all instances // current instance: " + _currIns);
											for (String _currRelIns : m_comareasoner.getRelatedInstances(_currIns)) {
												log("all related instances of " + _currIns + " // " + _currRelIns);
											}
										}
										for (String _currIns : m_comareasoner.getAllInstances("test:Place")) {
											_currIns="test"+_currIns.replaceFirst("http://test.hz.owl#", ":");
											log("all instances of test:Place // current instance: " + _currIns);
											for (String _currRelIns : m_comareasoner.getRelatedInstances(_currIns)) {
												log("all related instances of " + _currIns + " // " + _currRelIns);
											}
										}
										for (String _currIns : m_comareasoner.getAllInstances("test:PhysicalRoom")) {
											_currIns="test"+_currIns.replaceFirst("http://test.hz.owl#", ":");
											log("all instances of test:Place // current instance: " + _currIns);
											for (String _currRelIns : m_comareasoner.getRelatedInstances(_currIns)) {
												log("all related instances of " + _currIns + " // " + _currRelIns);
											}
										}
										m_placeholders.remove(_newPlaceNode.id);
										HashSet<WorkingMemoryAddress> _pendingPaths = m_tempAdjacencyStore.remove(_newPlaceNode.id);
										if (_pendingPaths!=null) {
											for (WorkingMemoryAddress _workingMemoryAddress : _pendingPaths) {
												ConnectivityPathProperty _currPendingPath = getMemoryEntry(_workingMemoryAddress, ConnectivityPathProperty.class);
												m_comareasoner.addRelation(":place"+_currPendingPath.place1Id, ":adjacent", ":place"+_currPendingPath.place2Id);
											}
										}
										removeChangeFilter(this);
									}
								} catch (DoesNotExistOnWMException e) {
									// TODO Auto-generated catch block
									e.printStackTrace();
								} catch (UnknownSubarchitectureException e) {
									// TODO Auto-generated catch block
									e.printStackTrace();
								}
							}
						});
					} // end PLACEHOLDER block
					
					// for both: register DELETE filter
					addChangeFilter(ChangeFilterFactory.createAddressFilter(_wmc.address, WorkingMemoryOperation.DELETE),
							new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(WorkingMemoryChange _wmc)
						throws CASTException {
							log("Got a callback for a DELETED Place WME!");
							// TODO
							// do something useful
						}
					});
					
// example code for place property handling
//					_newPlaceNode.status.equals(PlaceStatus.PLACEHOLDER);
//					SpatialProperties.GatewayPlaceProperty cpp;
//					cpp.

// example code for ontology querying
//					println("get all instances of test:Place and their related instances");
//					for (String _currIns : m_comareasoner.getAllInstances("test:Place")) {
//						_currIns="test"+_currIns.replaceFirst("http://test.hz.owl#", ":");
//						println("related instances of " + _currIns);
////						_currIns=_currIns.replace("#", ":");
//						for (String _currRelIns : m_comareasoner.getRelatedInstances(_currIns)) {
//							println(_currRelIns);
//						}
//					}
//
//					println("get all instances of test:PhysicalRoom and their related instances");
//					for (String _currIns : m_comareasoner.getAllInstances("test:PhysicalRoom")) {
//						_currIns="test"+_currIns.replaceFirst("http://test.hz.owl#", ":");
////						_currIns="test" + _currIns;
//						println("related instances of " + _currIns);
////						_currIns=_currIns.replace("#", ":");
//						for (String _currRelIns : m_comareasoner.getRelatedInstances(_currIns)) {
//							println(_currRelIns);
//						}
//					}
					
				} catch (DoesNotExistOnWMException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (UnknownSubarchitectureException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			};
		});
		
		
		// track connectivity
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(ConnectivityPathProperty.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						log("got a callback for an ADDED ConnectivityPathProperty");
						ConnectivityPathProperty _path = getMemoryEntry(_wmc.address, ConnectivityPathProperty.class);
						if (m_placeholders.contains(_path.place1Id)) {
							log("m_placeholders contains " + _path.place1Id);
							for (Long _pID : m_placeholders) {
								log("list of placeholder: " + _pID);
							}
							HashSet _allPaths4PH;
							if (m_tempAdjacencyStore.containsKey(_path.place1Id)) {
								_allPaths4PH = m_tempAdjacencyStore.get(_path.place1Id);
							}
							else {
								m_tempAdjacencyStore.put(_path.place1Id, new HashSet<WorkingMemoryAddress>());
								_allPaths4PH = m_tempAdjacencyStore.get(_path.place1Id);
							}
							_allPaths4PH.add(_wmc.address);
						} else if (m_placeholders.contains(_path.place2Id)) {
							log("m_placeholders contains " + _path.place2Id);
							for (Long _pID : m_placeholders) {
								log("list of placeholder: " + _pID);
							}
							HashSet<WorkingMemoryAddress> _allPaths4PH;
							if (m_tempAdjacencyStore.containsKey(_path.place2Id)) {
								_allPaths4PH = m_tempAdjacencyStore.get(_path.place2Id);
							}
							else {
								m_tempAdjacencyStore.put(_path.place2Id, new HashSet<WorkingMemoryAddress>());
								_allPaths4PH = m_tempAdjacencyStore.get(_path.place2Id);
							}
							_allPaths4PH.add(_wmc.address);
						} else { // both Places are TRUEPLACES!
							log("m_placeholders does not contain " + _path.place1Id + " nor " + _path.place2Id);
							for (Long _pID : m_placeholders) {
								log("list of placeholder: " + _pID);
							}
							log(_path.place1Id + " and "+ _path.place2Id + " should both be TRUEPLACES");
							m_comareasoner.addRelation(":place"+_path.place1Id, ":adjacent", ":place"+_path.place2Id);
						}
					}
				});

		log("Initiating connection to server " + m_reasoner_id.category + "::" + m_reasoner_id.name + "...");
		ObjectPrx base = getObjectAdapter().createProxy(m_reasoner_id);
		m_comareasoner = comadata.ComaReasonerInterfacePrxHelper.checkedCast(base);

	
	}

	@Override
	protected void runComponent() {
		try {
			m_proxyMarshall = getIceServer(m_marshaller_component_name, Marshaller.class , MarshallerPrx.class);
			log("initiated marshaller connection");
		} catch (CASTException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		m_proxyMarshall.addProxy("room", "area1", 1, new WorkingMemoryPointer());
		Feature _ftr = new Feature();
		_ftr.featlabel="area1";
		_ftr.alternativeValues = new FeatureValue[1];
		_ftr.alternativeValues[0]=new StringValue(1,getCASTTime(),"unknown");
		m_proxyMarshall.addFeature("room", "myfirstproxy", _ftr);
		
		super.runComponent();
	}


// register special filters where necessary, so this general filter is not necessary anymore
//		// this is the "proper" Place monitor
//		// for the time being, it does not create any coma instances
//		// instead the FNode monitor does this task
//		// once the interface changes, the relevant code from there has to go in here!
//		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class, WorkingMemoryOperation.OVERWRITE), 
//				new WorkingMemoryChangeReceiver() {
//			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
//				log("Got a callback for an OVERWRITTEN Place WME!");
//				try {
//					Place _placeNode = getMemoryEntry(_wmc.address, Place.class);
//					log("Place ID = " + _placeNode.id);
//
//					// removed place proxy handling
////					maintainPlaceProxy(_placeNode);
//
//					
//					//					log("Doing nothing else...");
//					// propagate this to the coma ontology and the binder when possible!
//					
//					// old code:
////					m_comareasoner.addInstance("test:place"+_newPlaceNode.nodeId, "test:Place");
////					log("Added new Place instance. This is a list of all instances in the ABox");
////					for (String _currIns : m_comareasoner.getAllInstances("owl:Thing")) {
////						log("instance: " + _currIns);
////					}
//				} catch (DoesNotExistOnWMException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				} catch (UnknownSubarchitectureException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
//			};
//		});
		
//		// this is the "hacky" FNode monitor, 
//		// which is used to get access to Place properties right away.
//		// once the Place WMEs contain all relevant information, copy this code to the correct monitor above
//		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(FNode.class, WorkingMemoryOperation.ADD), 
//				new WorkingMemoryChangeReceiver() {
//			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
//				log("Got a callback for an ADDed FNode!");
//				try {
//					FNode _newPlaceNode = getMemoryEntry(_wmc.address, FNode.class);
//					log("FNode ID (node ID) = " + _newPlaceNode.nodeId);
//					log("FNode status (gateway) = " + _newPlaceNode.gateway);
//					
//					// propagate this to the coma ontology and to the binder 
//					// 1st coma
//					m_comareasoner.addInstance("test:place"+_newPlaceNode.nodeId, "test:Place");
//					log("Added new Place instance."); // This is a list of all instances in the ABox");
////					for (String _currIns : m_comareasoner.getAllInstances("owl:Thing")) {
////						log("instance: " + _currIns);
////					}
//					// 2nd binder
////					boolean _isGateway = (_newPlaceNode.gateway==0 ? false: true);
////					createNewPlaceProxy(_newPlaceNode.nodeId, _isGateway);
//					maintainPlaceProxy(_newPlaceNode);
//					
//				} catch (DoesNotExistOnWMException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				} catch (UnknownSubarchitectureException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
//			};
//		});
		
//		// this is the "hacky" FNode monitor, 
//		// which is used to get access to Place properties right away.
//		// once the Place WMEs contain all relevant information, copy this code to the correct monitor above
//		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(FNode.class, WorkingMemoryOperation.OVERWRITE), 
//				new WorkingMemoryChangeReceiver() {
//			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
//				log("Got a callback for an OVERWRITTEN FNode!");
//				try {
//					FNode _fNode = getMemoryEntry(_wmc.address, FNode.class);
//					log("FNode ID (node ID) = " + _fNode.nodeId);
//					log("FNode status (gateway) = " + _fNode.gateway);
//					
//					// propagate this to the coma ontology and to the binder 
//					// 1st coma
//					m_comareasoner.addInstance("test:place"+_fNode.nodeId, "test:Place");
//					log("Added new Place instance."); // This is a list of all instances in the ABox");
////					for (String _currIns : m_comareasoner.getAllInstances("owl:Thing")) {
////						log("instance: " + _currIns);
////					}
//					// 2nd binder
////					boolean _isGateway = (_newPlaceNode.gateway==0 ? false: true);
////					createNewPlaceProxy(_newPlaceNode.nodeId, _isGateway);
//					maintainPlaceProxy(_fNode);
//					
//				} catch (DoesNotExistOnWMException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				} catch (UnknownSubarchitectureException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
//			};
//		});

//		// temporaray ("hacky") monitor for Place connectivity
//		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(AEdge.class, WorkingMemoryOperation.ADD), 
//				new WorkingMemoryChangeReceiver() {
//			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
//				log("Got a callback for an ADDed AEdge!");
//				try {
//					AEdge _newEdge = getMemoryEntry(_wmc.address, AEdge.class);
//					log("Start Node ID = " + _newEdge.startNodeId);
//					log("End Node ID = " + _newEdge.endNodeId);
//					m_comareasoner.addRelation("test:place"+_newEdge.startNodeId, "test:adjacent", "test:place"+_newEdge.endNodeId);
//					log("Added new Edge relation to coma ontology."); // This is a list of all room instances in the ABox");
////					for (String _currIns : m_comareasoner.getAllInstances("test:PhysicalRoom")) {
////						log("instance: " + _currIns);
////					}
//				} catch (DoesNotExistOnWMException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				} catch (UnknownSubarchitectureException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
//			};
//		});
		
	
	
	
	// removed place proxy handling
//	private void maintainPlaceProxy(Place _placeNode) {
//		log("maintaining a Place proxy for a Place struct.");
//		ComaPlace _comaPlace;
//		if (!(m_placeID2ComaPlace.containsKey(_placeNode.id))) {
//			_comaPlace = new ComaPlace(_placeNode.id);
//		} else {
//			_comaPlace = m_placeID2ComaPlace.get(_placeNode.id);
//		}
//		_comaPlace.setPlaceStatus(_placeNode.status);
//		
//		m_placeID2ComaPlace.put(_placeNode.id, _comaPlace);
//
//		if (_comaPlace.m_proxyWMid==null) {
//			log("no proxy WMA known...");
//			createNewPlaceProxy(_comaPlace);
//		} else {
//			log("proxy WMA known!");
//			updatePlaceProxy(_comaPlace);
//		}
//	}


	
	//	private void maintainPlaceProxy(FNode _placeNode) {
//		log("maintaining a Place proxy for an FNode struct.");
//		ComaPlace _comaPlace;
//		if (!(m_placeID2ComaPlace.containsKey(_placeNode.nodeId))) {
//			_comaPlace = new ComaPlace(_placeNode.nodeId);
//		} else {
//			_comaPlace = m_placeID2ComaPlace.get(_placeNode.nodeId);
//		}
//		_comaPlace.setGatewayStatus(_placeNode.gateway);
//
//		m_placeID2ComaPlace.put(_placeNode.nodeId, _comaPlace);
//
//		if (_comaPlace.m_proxyWMid==null) {
//			log("no proxy WMA known...");
//			createNewPlaceProxy(_comaPlace);
//		} else {
//			log("proxy WMA known!");
//			updatePlaceProxy(_comaPlace);
//		}
//	}
	
//	// removed place proxy handling
//	/**
//	 * 
//	 * @param _comaPlace
//	 */
//	public void createNewPlaceProxy(ComaPlace _comaPlace) {
//		// check for presence of binder
//		if (m_bindingSA==null) {
//			log("No binder specified. Not creating any proxy...");
//			return;
//		}
//		log("creating a new place proxy.");
//		
//		// create new proxy
//		Proxy _newPlaceProxy = new Proxy();
//		_newPlaceProxy.entityID = newDataID();
////		_newPlaceProxy.subarchId = this.getSubarchitectureID();
//		_newPlaceProxy.probExists = 1;
//			
//		_newPlaceProxy.features = new Feature[2];
//		
//		_newPlaceProxy.features[0] = new Feature();
//		_newPlaceProxy.features[0].featlabel = "place_id";
//		_newPlaceProxy.features[0].alternativeValues = new FeatureValue[1];
//		_newPlaceProxy.features[0].alternativeValues[0] = new StringValue(1,getCASTTime(),_comaPlace.m_id.toString());
//		
//		_newPlaceProxy.features[1] = new Feature();
//		_newPlaceProxy.features[1].featlabel = "place_type";
//		_newPlaceProxy.features[1].alternativeValues = new FeatureValue[1];
//		if (_comaPlace.getPlaceStatus()==null) {
//			_newPlaceProxy.features[1].alternativeValues[0] = new StringValue(1,getCASTTime(), "unknown");
////			_newPlaceProxy.features[1].alternativeValues[0] = new StringValue(0.5f, "Place");
////			_newPlaceProxy.features[1].alternativeValues[1] = new StringValue(0.5f, "Placeholder");
//			}
//		else if (_comaPlace.getPlaceStatus()==PlaceStatus.TRUEPLACE) {
//			_newPlaceProxy.features[1].alternativeValues[0] = new StringValue(1,getCASTTime(), "Place");
////			_newPlaceProxy.features[1].alternativeValues[0] = new StringValue(1, "Place");
////			_newPlaceProxy.features[1].alternativeValues[1] = new StringValue(0, "Placeholder");
//		} else {
//			_newPlaceProxy.features[1].alternativeValues[0] = new StringValue(1, getCASTTime(),"Placeholder");
////			_newPlaceProxy.features[1].alternativeValues[0] = new StringValue(0, "Place");
////			_newPlaceProxy.features[1].alternativeValues[1] = new StringValue(1, "Placeholder");
//		}
//			
////		_newPlaceProxy.features[2] = new Feature();
////		_newPlaceProxy.features[2].featlabel = "node_type";
////		_newPlaceProxy.features[2].alternativeValues = new FeatureValue[2];
////		if (_comaPlace.getGatewayStatus()==-1) {
////			_newPlaceProxy.features[2].alternativeValues[0] = new StringValue(0.5f, "Free Node");
////			_newPlaceProxy.features[2].alternativeValues[1] = new StringValue(0.5f, "Gateway");
////		} else if (_comaPlace.getGatewayStatus()==0) {
////			_newPlaceProxy.features[2].alternativeValues[0] = new StringValue(1, "Free Node");
////			_newPlaceProxy.features[2].alternativeValues[1] = new StringValue(0, "Gateway");
////		} else {
////			_newPlaceProxy.features[2].alternativeValues[0] = new StringValue(0, "Free Node");
////			_newPlaceProxy.features[2].alternativeValues[1] = new StringValue(1, "Gateway");
////		}
//		
//
//		_newPlaceProxy.distribution = ProbabilityUtils.generateProbabilityDistribution(_newPlaceProxy);
//
//		try {
////			addToWorkingMemory(new WorkingMemoryAddress(m_bindingSA,_newPlaceProxy.entityID), _newPlaceProxy);
//			addToWorkingMemory(new WorkingMemoryAddress(_newPlaceProxy.entityID, m_bindingSA), _newPlaceProxy);
//		} catch (AlreadyExistsOnWMException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//			System.exit(0);
//		} catch (DoesNotExistOnWMException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//			System.exit(0);
//		} catch (UnknownSubarchitectureException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//			System.exit(0);
//		}
//		_comaPlace.setProxyWMid(_newPlaceProxy.entityID);
//		m_placeID2ComaPlace.put(_comaPlace.m_id, _comaPlace);
////		m_placeID2proxyWMID.put(String.valueOf(_placeID), _newPlaceProxy.entityID);
//		log("Successfully added a place proxy to the binder.");
//		
//	}

	
	// removed place proxy handling
//	public void updatePlaceProxy(ComaPlace _comaPlace) {
//		// check for presence of binder
//		if (m_bindingSA==null) {
//			log("No binder specified. Not updating any proxy...");
//			return;
//		}
//		log("updating an existing place proxy.");
//		log("Place ID: " + _comaPlace.m_id);
//
//		try {
//			WorkingMemoryAddress _proxyWMA = new WorkingMemoryAddress(_comaPlace.getProxyWMid(), m_bindingSA);
//			Proxy _placeProxy = getMemoryEntry(_proxyWMA, Proxy.class);
//			
//			for (int i = 0; i < _placeProxy.features.length; i++) {
//				if (_placeProxy.features[i].featlabel.equals("place_type")) {
//					log("now looking at place_type.");
//					if (_comaPlace.getPlaceStatus()==null) {
//						log("place status is null.");
//						_placeProxy.features[i].alternativeValues[0] = new StringValue(1, getCASTTime(),"unknown");
////						_placeProxy.features[i].alternativeValues[0] = new StringValue(0.5f, "Place");
////						_placeProxy.features[i].alternativeValues[1] = new StringValue(0.5f, "Placeholder");
//					}
//					else if (_comaPlace.getPlaceStatus()==PlaceStatus.TRUEPLACE) {
//						log("place status is TRUEPLACE");
//						_placeProxy.features[i].alternativeValues[0] = new StringValue(1, getCASTTime(),"Place");
////						_placeProxy.features[i].alternativeValues[0] = new StringValue(1, "Place");
////						_placeProxy.features[i].alternativeValues[1] = new StringValue(0, "Placeholder");
//					} else {
//						log("place status is PLACEHOLDER");
//						_placeProxy.features[i].alternativeValues[0] = new StringValue(1, getCASTTime(),"Placeholder");
////						_placeProxy.features[i].alternativeValues[0] = new StringValue(0, "Place");
////						_placeProxy.features[i].alternativeValues[1] = new StringValue(1, "Placeholder");
//					}
//				} 
////					else if (_placeProxy.features[i].featlabel.equals("node_type")) {
////					if (_comaPlace.getGatewayStatus()==-1) {
////						_placeProxy.features[i].alternativeValues[0] = new StringValue(0.5f, "Free Node");
////						_placeProxy.features[i].alternativeValues[1] = new StringValue(0.5f, "Gateway");
////					} else if (_comaPlace.getGatewayStatus()==0) {
////						_placeProxy.features[i].alternativeValues[0] = new StringValue(1, "Free Node");
////						_placeProxy.features[i].alternativeValues[1] = new StringValue(0, "Gateway");
////					} else {
////						_placeProxy.features[i].alternativeValues[0] = new StringValue(0, "Free Node");
////						_placeProxy.features[i].alternativeValues[1] = new StringValue(1, "Gateway");
////					}
////					log("Feature: " + _placeProxy.features[i].featlabel + 
////							" -- gateway status:" + (_comaPlace.getGatewayStatus()==1 ? "Doorway" : (_comaPlace.getGatewayStatus()==-1 ? "unknown" :  "Free Node")));							
////				}
//			}
//			_placeProxy.distribution = ProbabilityUtils.generateProbabilityDistribution(_placeProxy);
//			
//			overwriteWorkingMemory(_proxyWMA, _placeProxy);
//			
//		} catch (DoesNotExistOnWMException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		} catch (UnknownSubarchitectureException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		} catch (ConsistencyException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		} catch (PermissionException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//		log("Successfully updated a place proxy on the binder.");
//	}
	
	// removed place proxy handling
//	private class ComaPlace {
//		private PlaceStatus m_placeStatus;
//		private Long m_id;
//		private short m_gateway;
//		private String m_proxyWMid;
//		
//		public ComaPlace(Long _placeId) {
//			m_id = _placeId;
//			m_gateway = -1;
//		}
//		
//		public boolean equals(ComaPlace _comaPlace) {
//			return this.m_id.equals(_comaPlace.m_id);
//		}
//		
//		public PlaceStatus getPlaceStatus() {
//			return m_placeStatus;
//		}
//		public void setPlaceStatus(PlaceStatus _placeStatus) {
//			m_placeStatus = _placeStatus;
//		}
//		
//		public Long getPlaceId() {
//			return m_id;
//		}
//		
//		public short getGatewayStatus() {
//			return m_gateway;
//		}
//		public void setGatewayStatus(short _gatewayStatus) {
//			m_gateway = _gatewayStatus;
//		}
//		
//		public String getProxyWMid() {
//			return m_proxyWMid;
//		}
//		public void setProxyWMid(String _proxyWMid) {
//			m_proxyWMid = _proxyWMid;
//		}
//	}
	
	
//	/**
//	 * @todo THIS DOES NOT SUPPORT PLACEHOLDERS YET!
//	 * 
//	 * @param _placeID
//	 * @param _isGateway
//	 */
//	public void createNewPlaceProxy(long _placeID, boolean _isGateway) {
//		// check for presence of binder
//		if (m_bindingSA==null) {
//			log("No binder specified. Not creating any proxy...");
//			return;
//		}
//		
//		log("Warning: Placeholders not supported yet!");
//		// create new proxy
//		Proxy _newPlaceProxy = new Proxy();
//		_newPlaceProxy.entityID = newDataID();
//		_newPlaceProxy.subarchId = this.getSubarchitectureID();
//		_newPlaceProxy.probExists = 1;
//			
//		_newPlaceProxy.features = new Feature[2];
//		
//		_newPlaceProxy.features[0] = new Feature();
//		_newPlaceProxy.features[0].featlabel = "place_id";
//		_newPlaceProxy.features[0].alternativeValues = new FeatureValue[1];
//		_newPlaceProxy.features[0].alternativeValues[0] = new StringValue(1,String.valueOf(_placeID));
//		
//		_newPlaceProxy.features[1] = new Feature();
//		_newPlaceProxy.features[1].featlabel = "place_type";
//		_newPlaceProxy.features[1].alternativeValues = new FeatureValue[1];
//		_newPlaceProxy.features[1].alternativeValues[0] = new StringValue(1, (_isGateway ? "Doorway" : "Place"));
//			
//		_newPlaceProxy.distribution = ProbabilityDistributionUtils.generateProbabilityDistribution(_newPlaceProxy);
//
//		try {
////			addToWorkingMemory(new WorkingMemoryAddress(m_bindingSA,_newPlaceProxy.entityID), _newPlaceProxy);
//			addToWorkingMemory(new WorkingMemoryAddress(_newPlaceProxy.entityID, m_bindingSA), _newPlaceProxy);
//		} catch (AlreadyExistsOnWMException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//			System.exit(0);
//		} catch (DoesNotExistOnWMException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//			System.exit(0);
//		} catch (UnknownSubarchitectureException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//			System.exit(0);
//		}
//		m_placeID2proxyWMID.put(String.valueOf(_placeID), _newPlaceProxy.entityID);
//		log("Successfully added a place proxy to the binder.");
//		
//	}
	
}