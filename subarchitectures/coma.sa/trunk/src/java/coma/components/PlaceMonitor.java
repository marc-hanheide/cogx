package coma.components;

import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;


import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.featvalues.StringValue;

import coma.components.ComaReasoner.ComaReasonerInterfaceI;
import comadata.ComaReasonerInterfacePrx;

import Ice.Identity;
import Ice.ObjectPrx;
import Marshalling.MarshallerPrx;
import Marshalling.Marshaller;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import SpatialProperties.ConnectivityPathProperty;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;

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
	String m_comareasoner_component_name;
	ComaReasonerInterfacePrx m_comareasoner;
	
	String m_marshaller_component_name;
	MarshallerPrx m_proxyMarshall;
	
	String m_bindingSA;
	//Map<String, String> m_placeID2proxyWMID;
	// removed place proxy handling
	//	Map<Long,ComaPlace> m_placeID2ComaPlace;
	HashSet<Long> m_placeholders;
	HashSet<Long> m_trueplaces;
	HashMap<Long, HashSet<WorkingMemoryAddress>> m_tempAdjacencyStore;
	

	public void configure(Map<String, String> args) {
		log("configure() called");
		m_reasoner_id = new Identity();
		m_reasoner_id.name="";
		m_reasoner_id.category="ComaReasoner";

		if (args.containsKey("--reasoner-name")) {
			m_reasoner_id.name=args.get("--reasoner-name");
			m_comareasoner_component_name=args.get("--reasoner-name");
		}
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
		m_trueplaces = new HashSet<Long>();
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
				processAddedPlace(_wmc);
			};
		});
		
		// track connectivity
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(ConnectivityPathProperty.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						processAddedConnectivityPath(_wmc);
					}
				});

		log("Initiating connection to server " + m_reasoner_id.category + "::" + m_reasoner_id.name + "...");
		ObjectPrx base = getObjectAdapter().createProxy(m_reasoner_id);
		m_comareasoner = comadata.ComaReasonerInterfacePrxHelper.checkedCast(base);

		try {
			if (m_marshaller_component_name!=null) log("initiating connection to Ice server " + m_marshaller_component_name);
			if (m_marshaller_component_name!=null) m_proxyMarshall = getIceServer(m_marshaller_component_name, Marshaller.class , MarshallerPrx.class);
			if (m_proxyMarshall!=null) log("initiated marshaller connection");
		} catch (CASTException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			log("connection to Ice server "+ m_marshaller_component_name + " failed!");
		}	
		// this code does not work :-(
//		try {
//			if (m_comareasoner_component_name!=null) log("initiating connection to Ice server " + m_comareasoner_component_name);
//			if (m_comareasoner_component_name!=null) m_comareasoner = getIceServer(m_comareasoner_component_name, comadata.ComaReasonerInterface.class , comadata.ComaReasonerInterfacePrxHelper.class);
//			if (m_comareasoner!=null) log("initiated comareasoner connection");
//		} catch (CASTException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//			log("connection to Ice server "+ m_comareasoner_component_name + " failed!");
//		}	
	}

	protected void runComponent() {
		// TODO replace this with real room proxy creation!
		if (m_proxyMarshall!=null) {
			log("Baseline: simply create a room proxy");
			m_proxyMarshall.addProxy("room", "area1", 1, new WorkingMemoryPointer());
			Feature _ftr = new Feature();
			_ftr.featlabel="area1";
			_ftr.alternativeValues = new FeatureValue[1];
			_ftr.alternativeValues[0]=new StringValue(1,getCASTTime(),"unknown");
			m_proxyMarshall.addFeature("room", "myfirstproxy", _ftr);
		}
		super.runComponent();
	}
	
	
	private void processAddedPlace(WorkingMemoryChange _wmc) {
		log("Got a callback for an ADDed Place WME:");
		try {
			// read place struct from WM
			Place _newPlaceNode = getMemoryEntry(_wmc.address, Place.class);
			log("Place ID = " + _newPlaceNode.id + " " +
			"Place status = " + (_newPlaceNode.status.equals(PlaceStatus.PLACEHOLDER) ? "PLACEHOLDER" : "TRUEPLACE"));

			// check the Place status
			if (_newPlaceNode.status.equals(PlaceStatus.TRUEPLACE)) { 
				// TRUEPLACE block
				log("create dora:Place instance");
				m_comareasoner.addInstance("dora:place"+_newPlaceNode.id, "dora:Place");
				// keep track of created true place instances
				m_trueplaces.add(Long.valueOf(_newPlaceNode.id));
				logInstances("owl:Thing");
				logInstances("dora:Place");
				logInstances("dora:PhysicalRoom");
				
				// process pending paths
				HashSet<WorkingMemoryAddress> _pendingPaths = m_tempAdjacencyStore.remove(_newPlaceNode.id);
				if (_pendingPaths!=null) {
					log("process pending paths:");
					for (WorkingMemoryAddress _workingMemoryAddress : _pendingPaths) {
						ConnectivityPathProperty _currPendingPath = getMemoryEntry(_workingMemoryAddress, ConnectivityPathProperty.class);
						log("add relation to coma: " + "dora:place"+_currPendingPath.place1Id + " dora:adjacent " + "dora:place"+_currPendingPath.place2Id);
						m_comareasoner.addRelation("dora:place"+_currPendingPath.place1Id, "dora:adjacent", "dora:place"+_currPendingPath.place2Id);
					}
				}

			} else { 
				// PLACEHOLDER block
				log("going to add " + _newPlaceNode.id + " to m_placeholders");
				// keep track of placeholders that should not be put into the ontology
				m_placeholders.add(Long.valueOf(_newPlaceNode.id));
				log("added " + _newPlaceNode.id + " to m_placeholders");
				
				// add a change filter so that they are processed when they turn into true places
				addChangeFilter(ChangeFilterFactory.createAddressFilter(_wmc.address, WorkingMemoryOperation.OVERWRITE),
						new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
						boolean _removeFiler = processOverwrittenPlace(_wmc);
						// remove this temporary filter if the place is a true place
						if (_removeFiler) removeChangeFilter(this);
						// otherwise keep it, i.e., do nothing...
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
			
//example code for place property handling
//			_newPlaceNode.status.equals(PlaceStatus.PLACEHOLDER);
//			SpatialProperties.GatewayPlaceProperty cpp;
//			cpp.

		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	
	private void processAddedConnectivityPath(WorkingMemoryChange _wmc) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		// get path from WM
		ConnectivityPathProperty _path = getMemoryEntry(_wmc.address, ConnectivityPathProperty.class);
		log("got a callback for an ADDED ConnectivityPathProperty between " + _path.place1Id + " and " +_path.place2Id);
		
		log("Set of known placeholders: " + m_placeholders);
		log("Set of known places: " + m_trueplaces);
		// suspend paths that involve placeholders
		if (m_placeholders.contains(Long.valueOf(_path.place1Id))) {
			// place1 is a placeholder, that means its connectivity should be marked as pending
			// store all pending connectivity path WMEs
			HashSet<WorkingMemoryAddress> _allPaths4PH;
			if (m_tempAdjacencyStore.containsKey(_path.place1Id)) {
				_allPaths4PH = m_tempAdjacencyStore.get(_path.place1Id);
			}
			else {
				m_tempAdjacencyStore.put(_path.place1Id, new HashSet<WorkingMemoryAddress>());
				_allPaths4PH = m_tempAdjacencyStore.get(_path.place1Id);
			}
			_allPaths4PH.add(_wmc.address);
		} else if (m_placeholders.contains(Long.valueOf(_path.place2Id))) {
			// place 2 is a placeholder, that means its connectivity should be marked as pending
			HashSet<WorkingMemoryAddress> _allPaths4PH;
			if (m_tempAdjacencyStore.containsKey(_path.place2Id)) {
				_allPaths4PH = m_tempAdjacencyStore.get(_path.place2Id);
			}
			else {
				m_tempAdjacencyStore.put(_path.place2Id, new HashSet<WorkingMemoryAddress>());
				_allPaths4PH = m_tempAdjacencyStore.get(_path.place2Id);
			}
			_allPaths4PH.add(_wmc.address);
		} else { 
			// both Places are not known PLACEHOLDERS
			// might still be the case that one of them is still unknown, 
			// in which case adjacency creation is also postponed!
			if (m_trueplaces.contains(Long.valueOf(_path.place1Id)) && m_trueplaces.contains(Long.valueOf(_path.place2Id))) {
				log("*** about to assert an adjacency relation between: " + _path.place1Id + " and " + _path.place2Id + ". Please check whether one of them is contained in the following list!");
				logInstances("dora:Place");
				m_comareasoner.addRelation("dora:place"+_path.place1Id, "dora:adjacent", "dora:place"+_path.place2Id);
				log("added adjacency relation: dora:place"+_path.place1Id + " dora:adjacent " + " dora:place"+_path.place2Id);
				logInstances("dora:Place");
			} else {
				if (!m_trueplaces.contains(Long.valueOf(_path.place1Id))) {
					HashSet<WorkingMemoryAddress> _allPaths4PH;
					if (m_tempAdjacencyStore.containsKey(_path.place1Id)) {
						_allPaths4PH = m_tempAdjacencyStore.get(_path.place1Id);
					}
					else {
						m_tempAdjacencyStore.put(_path.place1Id, new HashSet<WorkingMemoryAddress>());
						_allPaths4PH = m_tempAdjacencyStore.get(_path.place1Id);
					}
					_allPaths4PH.add(_wmc.address);
				} else if (!m_trueplaces.contains(Long.valueOf(_path.place2Id))) {
					HashSet<WorkingMemoryAddress> _allPaths4PH;
					if (m_tempAdjacencyStore.containsKey(_path.place2Id)) {
						_allPaths4PH = m_tempAdjacencyStore.get(_path.place2Id);
					}
					else {
						m_tempAdjacencyStore.put(_path.place2Id, new HashSet<WorkingMemoryAddress>());
						_allPaths4PH = m_tempAdjacencyStore.get(_path.place2Id);
					}
					_allPaths4PH.add(_wmc.address);
				} else {
					log("There was an inconsistency! ABORTING!");
					System.exit(0);
				}
			}
		}
	}
	
	
	private boolean processOverwrittenPlace(WorkingMemoryChange _wmc) {
		boolean _removeFilterAfterwards = false;
		log("Got a callback for an OVERWRITTEN former Placeholder WME!");
		try {
			// get the place from WM
			Place _newPlaceNode = getMemoryEntry(_wmc.address, Place.class);
			log("Place ID = " + _newPlaceNode.id + " " +
			"Place status = " + (_newPlaceNode.status.equals(PlaceStatus.PLACEHOLDER) ? "PLACEHOLDER" : "TRUEPLACE"));

			// for the moment we are only interested in true places
			if (_newPlaceNode.status.equals(PlaceStatus.TRUEPLACE)) {
				log("create dora:place instance");
				m_comareasoner.addInstance("dora:place"+_newPlaceNode.id, "dora:Place");
				logInstances("owl:Thing");
				logInstances("dora:Place");
				logInstances("dora:PhysicalRoom");

				m_placeholders.remove(Long.valueOf(_newPlaceNode.id));
				m_trueplaces.add(Long.valueOf(_newPlaceNode.id));
				
				// process pending paths
				HashSet<WorkingMemoryAddress> _pendingPaths = m_tempAdjacencyStore.remove(_newPlaceNode.id);
				if (_pendingPaths!=null) {
					log("process pending paths:");
					for (WorkingMemoryAddress _workingMemoryAddress : _pendingPaths) {
						ConnectivityPathProperty _currPendingPath = getMemoryEntry(_workingMemoryAddress, ConnectivityPathProperty.class);
						log("add relation to coma: " + "dora:place"+_currPendingPath.place1Id + " dora:adjacent " + "dora:place"+_currPendingPath.place2Id);
						m_comareasoner.addRelation("dora:place"+_currPendingPath.place1Id, "dora:adjacent", "dora:place"+_currPendingPath.place2Id);
					}
				}
				_removeFilterAfterwards = true;
			}
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return _removeFilterAfterwards;
	}
	
	
	
	
	
	private void logInstances(String _con) {
		StringBuffer _allInsLogMsg = new StringBuffer("all instances of " + _con + " ==> ");
		String[] _allIns = m_comareasoner.getAllInstances(_con);
		StringBuffer[] _allRelInsLogArray = new StringBuffer[_allIns.length];
		StringBuffer[] _allRelInsByRelLogArray = new StringBuffer[_allIns.length];
		StringBuffer[] _allConsLogArray = new StringBuffer[_allIns.length];
		
		for (int i = 0; i < _allIns.length; i++) {
			String _currIns = _allIns[i];
			if (!_currIns.startsWith("dora")) _currIns="dora"+_currIns.replaceFirst("http://dora.cogx.eu#", ":");
			_allInsLogMsg.append(_currIns + " ");

			_allRelInsLogArray[i] = new StringBuffer("all related instances of " + _currIns + " ==> ");
			for (String _currRelIns : m_comareasoner.getRelatedInstances(_currIns)) {
				_allRelInsLogArray[i].append(_currRelIns + " ");
			}
			
			_allRelInsByRelLogArray[i] = new StringBuffer("all related instances of " + _currIns + " via dora:sameRoomAs ==> ");
			for (String _currRelIns : m_comareasoner.getRelatedInstancesByRelation(_currIns, "dora:sameRoomAs")) {
				_allRelInsByRelLogArray[i].append(_currRelIns + " ");
			}

			_allConsLogArray[i] = new StringBuffer("all concepts of " + _currIns + " ==> ");
			for (String _currCon : m_comareasoner.getAllConcepts(_currIns)) {
				_allConsLogArray[i].append(_currCon + " ");
			}
			
		}
		
		log(_allInsLogMsg);
		for (StringBuffer stringBuffer : _allRelInsLogArray) {
			log(stringBuffer);
		}
		for (StringBuffer stringBuffer : _allRelInsByRelLogArray) {
			log(stringBuffer);
		}
		for (StringBuffer stringBuffer : _allConsLogArray) {
			log(stringBuffer);
		}
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
////					m_comareasoner.addInstance("dora:place"+_newPlaceNode.nodeId, "dora:Place");
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
//					m_comareasoner.addInstance("dora:place"+_newPlaceNode.nodeId, "dora:Place");
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
//					m_comareasoner.addInstance("dora:place"+_fNode.nodeId, "dora:Place");
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
//					m_comareasoner.addRelation("dora:place"+_newEdge.startNodeId, "dora:adjacent", "dora:place"+_newEdge.endNodeId);
//					log("Added new Edge relation to coma ontology."); // This is a list of all room instances in the ABox");
////					for (String _currIns : m_comareasoner.getAllInstances("dora:PhysicalRoom")) {
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