package coma.components;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.TreeSet;

import SpatialData.Place;
import SpatialData.PlaceStatus;
import SpatialProbabilities.ProbabilityDistribution;
import SpatialProperties.ConnectivityPathProperty;
import SpatialProperties.GatewayPlaceProperty;
import SpatialProperties.ObjectPlaceProperty;

import coma.aux.ComaHelper;
import comadata.ComaReasonerInterfacePrx;
import comadata.ComaRoom;
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
import cast.core.CASTData;

/**
 * This is a simple monitor that replicates spatial WM entries, *PLACES*, inside coma.
 * 
 * The current version has been adapted for the Dora Yr2 system and hence does not
 * contain any binder-related code anymore. However, it still requires the 
 * CROWL/Jena-based ComaReasoner to be present.
 * 
 * **IMPORTANT** This must be adapted:
 * As of now, the OLD FNode WMEs are used in order to have access to properties.
 * 
 * 
 * @author hendrik
 * CAST file arguments:
 * --reasoner_id <ComaReasoner>
 * -bsa <binder SA>
 */
public class PlaceMonitor extends ManagedComponent {

	public PlaceMonitor() {
		return;
	}
	private static final int TIME_TO_WAIT_TO_SETTLE = 1000;
	
	private String m_comareasoner_component_name;
	private ComaReasonerInterfacePrx m_comareasoner;
	
	private HashSet<Long> m_placeholders;
	private HashSet<Long> m_trueplaces;
	private HashMap<Long, HashSet<WorkingMemoryAddress>> m_tempAdjacencyStore;
	
	private int m_roomIndexCounter = 0;
	private int m_objectIndexCounter = 0;
	private HashSet<String> m_existingRoomProxies;
	private HashMap<String,HashSet<String>> m_existingRelationProxies;
	
	private boolean m_createDummyObjects;
	
	private boolean maintainRoomsTaskPending = false;

	public void configure(Map<String, String> args) {
		log("configure() called");

		if (args.containsKey("--reasoner-name")) {
			m_comareasoner_component_name=args.get("--reasoner-name");
		}
		if (args.containsKey("--dummy-objects")) {
			if (!args.get("--dummy-objects").equals("false")) {
				m_createDummyObjects = true;
			}
		}

		m_placeholders = new HashSet<Long>();
		m_trueplaces = new HashSet<Long>();
		m_tempAdjacencyStore = new HashMap<Long, HashSet<WorkingMemoryAddress>>();
		m_existingRoomProxies = new HashSet<String>();
		m_existingRelationProxies = new HashMap<String,HashSet<String>>();
	}
	
	
	public void start() {
		if (m_comareasoner_component_name==null) {
			log("No coma reasoner present. Exiting! (Specify the ComaReasoner component name using --reasoner-name)");
			System.exit(-1);
		}
			
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

		// track gateways
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(GatewayPlaceProperty.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) 
			throws CASTException {
				processAddedGatewayProperty(_wmc);
			}
		});

		// track objects found by AVS
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(ObjectPlaceProperty.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) 
			throws CASTException {
				processAddedObjectProperty(_wmc);
			}
		});

		
		// initiate ice server connections
		
		// connection to the coma reasoner
		try {
			if (m_comareasoner_component_name!=null) log("initiating connection to Ice server " + m_comareasoner_component_name);
			if (m_comareasoner_component_name!=null) m_comareasoner = getIceServer(m_comareasoner_component_name, comadata.ComaReasonerInterface.class , comadata.ComaReasonerInterfacePrx.class);
			if (m_comareasoner!=null) log("initiated comareasoner connection");
			else throw new CASTException();
		} catch (CASTException e) {
			e.printStackTrace();
			log("Connection to the coma reasoner Ice server at "+ m_comareasoner_component_name + " failed! Exiting. (Specify the coma reasoner component name using --reasoner-name)");
			System.exit(-1);
		}	
	}
	
	@Override
	protected void runComponent() {
		while (isRunning()) {
			try {
				synchronized (this) {
					// if there is no pending task, continue the loop 
					if (!this.maintainRoomsTaskPending) {
						log("no room maintenance task pending. waiting...");
						this.wait();
						continue;
					}
				}
				synchronized (this) {
					log("pending room maintenance task. acknowledged.");
					this.maintainRoomsTaskPending = false;
				}
				// wait for another change
				log("wait for some time to let it settle.");
				Thread.sleep(TIME_TO_WAIT_TO_SETTLE);
				synchronized (this) {			
					// if there were no more changes
					if (this.maintainRoomsTaskPending) {
						log("ok. got a fresh pending maintenance task. not yet processing it.");
						continue;
					}
				}
				log("waited long enough. no more fresh maintenance tasks for a while. gonna process them!");
				// execute the room maintenance algorithm
				this.lockComponent();
				executeMaintainRoomsAlgorithm();
				this.unlockComponent();
			}  catch (InterruptedException e) {
				logException(e);
			}
		}
	}
	
	
	private void processAddedPlace(WorkingMemoryChange _wmc) {
		debug("Got a callback for an ADDed Place WME:");
		try {
			// read place struct from WM
			Place _newPlaceNode = getMemoryEntry(_wmc.address, Place.class);
			log("processAddedPlace() called: Place ID = " + _newPlaceNode.id + " " +
			"Place status = " + (_newPlaceNode.status.equals(PlaceStatus.PLACEHOLDER) ? "PLACEHOLDER" : "TRUEPLACE"));

			// check the Place status
			if (_newPlaceNode.status.equals(PlaceStatus.TRUEPLACE)) { 
				// TRUEPLACE block
				log("create dora:Place instance " + "dora:place"+_newPlaceNode.id);
				m_comareasoner.addInstance("dora:place"+_newPlaceNode.id, "dora:Place");
				// keep track of created true place instances
				m_trueplaces.add(Long.valueOf(_newPlaceNode.id));
				logInstances("owl:Thing");
				logInstances("dora:Place");
				logInstances("dora:PhysicalRoom");
				
				if (m_createDummyObjects) {
					// TODO *** this is just test code *** REMOVE afterwards!
					log("***** THIS IS JUST TEST CODE!!!!!");
					log("going to add random objects");
					if (_newPlaceNode.id%3==0) {
						// log every third place creates a dummy object
						createObject(new ObjectPlaceProperty(_newPlaceNode.id,
								null, null, true, "dummyObject"));
					}
				}

				// creating an initial seed room for the first Place added!
				if (_newPlaceNode.id==0) maintainRooms();
				
				// process pending paths
				HashSet<WorkingMemoryAddress> _pendingPaths = m_tempAdjacencyStore.remove(_newPlaceNode.id);
				if (_pendingPaths!=null) {
					debug("process pending paths:");
					for (WorkingMemoryAddress _workingMemoryAddress : _pendingPaths) {
						ConnectivityPathProperty _currPendingPath = getMemoryEntry(_workingMemoryAddress, ConnectivityPathProperty.class);
						log("add relation to coma: " + "dora:place"+_currPendingPath.place1Id + " dora:adjacent " + "dora:place"+_currPendingPath.place2Id);
						m_comareasoner.addRelation("dora:place"+_currPendingPath.place1Id, "dora:adjacent", "dora:place"+_currPendingPath.place2Id);
					}
					// trigger room creation, splitting, merging, maintenance
					maintainRooms();
				}
				
			} else { 
				// PLACEHOLDER block
				debug("going to add " + _newPlaceNode.id + " to m_placeholders");
				// keep track of placeholders that should not be put into the ontology
				m_placeholders.add(Long.valueOf(_newPlaceNode.id));
				debug("added " + _newPlaceNode.id + " to m_placeholders");
				
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
			
			final long _placeID = _newPlaceNode.id;
			// for both: register DELETE filter
			addChangeFilter(ChangeFilterFactory.createAddressFilter(_wmc.address, WorkingMemoryOperation.DELETE),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc)
				throws CASTException {
					processDeletedPlace(_placeID);
				}
			});
			
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
//		maintainRooms();
	}
	
	
	private void processAddedConnectivityPath(WorkingMemoryChange _wmc) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		// get path from WM
		ConnectivityPathProperty _path = getMemoryEntry(_wmc.address, ConnectivityPathProperty.class);
		log("processAddedConnectivityPath() called: got a callback for an ADDED ConnectivityPathProperty between " + _path.place1Id + " and " +_path.place2Id);
		
		debug("Set of known placeholders: " + m_placeholders);
		debug("Set of known places: " + m_trueplaces);
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
				m_comareasoner.addRelation("dora:place"+_path.place1Id, "dora:adjacent", "dora:place"+_path.place2Id);
				debug("added adjacency relation: dora:place"+_path.place1Id + " dora:adjacent " + " dora:place"+_path.place2Id);
				logInstances("dora:Place");
				// trigger room creation, splitting, merging, maintenance
				maintainRooms();
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
					System.exit(-1);
				}
			}
		}
	}
	
	private void processAddedGatewayProperty(WorkingMemoryChange _wmc) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		// get path from WM
		GatewayPlaceProperty _gateProp = getMemoryEntry(_wmc.address, GatewayPlaceProperty.class);
		log("processAddedGatewayProperty() called: got a callback for an ADDED GatewayPlaceProperty for " + _gateProp.placeId + ". The probability distribution is not yet taken into account!");
		
		// TODO handle probability distribution 
		// for now this property is only added for true gateways, but it is not guaranteed to remain like that
//		DiscreteProbabilityDistribution _gatewayProbability = (DiscreteProbabilityDistribution) _gateProp.distribution;
		
		// the place is immediately asserted to instantiate Doorway
		// this should be safe because Doorway Placeholders are seldom and their connectivity
		// is not stored in the ontology anyway.
		log("assert dora:Doorway instance dora:place"+ _gateProp.placeId);
		m_comareasoner.addInstance("dora:place"+_gateProp.placeId, "dora:Doorway");
		
		// trigger room creation, splitting, merging, maintenance
		maintainRooms();
	}
	
	private void processAddedObjectProperty(WorkingMemoryChange _wmc) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		log("processAddedObjectProperty() called.");
		// get object from WM
		ObjectPlaceProperty _objProp = getMemoryEntry(_wmc.address, ObjectPlaceProperty.class);
		
		debug("got a callback for an ADDED ObjectPlaceProperty for " + _objProp.placeId + " with category "+ _objProp.category + " and mapValue " + ((SpatialProperties.StringValue)_objProp.mapValue).value + ". The probability distribution is not yet taken into account!");
		
		// TODO handle probability distribution 
//		DiscreteProbabilityDistribution _gatewayProbability = (DiscreteProbabilityDistribution) _gateProp.distribution;
		
		boolean _objectCreated = createObject(_objProp);
		// trigger room creation, splitting, merging, maintenance
		// because their class might have changed
		if (_objectCreated) maintainRooms();
	}
		
	private boolean createObject(ObjectPlaceProperty _objProp) {
		// establish the ontology member names
		String category = "dora:" + ComaHelper.firstCap(((SpatialProperties.StringValue)_objProp.mapValue).value);
		String placeIns = "dora:place"+_objProp.placeId;
		String inRel = "dora:in";
		String containsRel = "dora:contains";
		
		log("createObject of category " + category + " in place " + placeIns + " called.");
		
		// check whether the given place already contains an instance of the given category
		String [] objsInPlace = m_comareasoner.getRelatedInstancesByRelation(placeIns, containsRel);
		for (String obj : objsInPlace) {
			if (obj.startsWith(":")) obj = "dora" + obj;
			if (m_comareasoner.isInstanceOf(obj, category)) {
				log("object of the given category already exists in the given place - doing nothing else.");
				return false;
				// if such an object exists, don't create a new instance!
			}
		}
		
		String objIns = "dora:object" + m_objectIndexCounter++; 
		
		log("going to add new instance " + objIns + " of category " + category);
		m_comareasoner.addInstance(objIns, category);
		log("going to add new relation " + objIns + " " + inRel + " " + placeIns);
		m_comareasoner.addRelation(objIns, inRel, placeIns);
		return true;
	}

	private boolean processOverwrittenPlace(WorkingMemoryChange _wmc) {
		boolean _removeFilterAfterwards = false;
		debug("Got a callback for an OVERWRITTEN former Placeholder WME!");
		try {
			// get the place from WM
			Place _newPlaceNode = getMemoryEntry(_wmc.address, Place.class);
			log("processOverwrittenPlace() called: Place ID = " + _newPlaceNode.id + " " +
			"Place status = " + (_newPlaceNode.status.equals(PlaceStatus.PLACEHOLDER) ? "PLACEHOLDER" : "TRUEPLACE"));

			// for the moment we are only interested in true places
			if (_newPlaceNode.status.equals(PlaceStatus.TRUEPLACE)) {
				log("create dora:Place instance " + "dora:place"+_newPlaceNode.id);
				m_comareasoner.addInstance("dora:place"+_newPlaceNode.id, "dora:Place");
				logInstances("owl:Thing");
				logInstances("dora:Place");
				logInstances("dora:PhysicalRoom");

				m_placeholders.remove(Long.valueOf(_newPlaceNode.id));
				m_trueplaces.add(Long.valueOf(_newPlaceNode.id));
				
				// process pending paths
				HashSet<WorkingMemoryAddress> _pendingPaths = m_tempAdjacencyStore.remove(_newPlaceNode.id);
				if (_pendingPaths!=null) {
					debug("process pending paths:");
					for (WorkingMemoryAddress _workingMemoryAddress : _pendingPaths) {
						ConnectivityPathProperty _currPendingPath = getMemoryEntry(_workingMemoryAddress, ConnectivityPathProperty.class);
						log("add relation to coma: " + "dora:place"+_currPendingPath.place1Id + " dora:adjacent " + "dora:place"+_currPendingPath.place2Id);
						m_comareasoner.addRelation("dora:place"+_currPendingPath.place1Id, "dora:adjacent", "dora:place"+_currPendingPath.place2Id);
					}
					// trigger room creation, splitting, merging, maintenance
					maintainRooms();
				}
				
				_removeFilterAfterwards = true;
			}
		} catch (DoesNotExistOnWMException e) {
			log("The overwritten Place WME at " + _wmc.address + " ceased to exist. Not taking any further steps. Its deletion should be handled by the deletion change filter.");
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
//		maintainRooms();
		return _removeFilterAfterwards;
	}
	
	private void processDeletedPlace(long _deletedPlaceID) {
		log("processDeletedPlace() called: Got a callback for a DELETED Place with ID: " + _deletedPlaceID);
		boolean _successfullyDeleted = m_comareasoner.deleteInstance("dora:place"+_deletedPlaceID);
		if (_successfullyDeleted) log("successfully deleted " + "dora:place"+_deletedPlaceID);
		else log("There was an error deleting " + "dora:place"+_deletedPlaceID);
		
		// delete objects associated with that place
		_successfullyDeleted= m_comareasoner.deleteInstance("dora:object"+_deletedPlaceID);
		if (_successfullyDeleted) log("successfully deleted " + "dora:object"+_deletedPlaceID);
		else log("There was an error deleting " + "dora:object"+_deletedPlaceID);

		m_placeholders.remove(_deletedPlaceID);
		if (m_trueplaces.remove(_deletedPlaceID)) maintainRooms();
		logInstances("dora:Place");
	}
	
	
	private void logInstances(String _con) {
		// logInstances is now restricted to debug mode only. otherwise it gets too messy.
		if (!m_bDebugOutput) return;
		
		StringBuffer _allInsLogMsg = new StringBuffer("all instances of " + _con + " ==> ");
		String[] _allIns = m_comareasoner.getAllInstances(_con);
		StringBuffer[] _allRelInsLogArray = new StringBuffer[_allIns.length];
		StringBuffer[] _allRelInsByRelLogArray = new StringBuffer[_allIns.length];
		StringBuffer[] _allRelInsByRel2LogArray = new StringBuffer[_allIns.length];
		StringBuffer[] _allRelInsByRel3LogArray = new StringBuffer[_allIns.length];
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

			_allRelInsByRel2LogArray[i] = new StringBuffer("all related instances of " + _currIns + " via dora:constituentOfRoom ==> ");
			for (String _currRelIns : m_comareasoner.getRelatedInstancesByRelation(_currIns, "dora:constituentOfRoom")) {
				_allRelInsByRel2LogArray[i].append(_currRelIns + " ");
			}

			_allRelInsByRel3LogArray[i] = new StringBuffer("all related instances of " + _currIns + " via dora:in ==> ");
			for (String _currRelIns : m_comareasoner.getRelatedInstancesByRelation(_currIns, "dora:in")) {
				_allRelInsByRel3LogArray[i].append(_currRelIns + " ");
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
		for (StringBuffer stringBuffer : _allRelInsByRel2LogArray) {
			log(stringBuffer);
		}
		for (StringBuffer stringBuffer : _allRelInsByRel3LogArray) {
			log(stringBuffer);
		}
		for (StringBuffer stringBuffer : _allConsLogArray) {
			log(stringBuffer);
		}
	}
	
	private void maintainRooms() {
		synchronized(this) {
			log("maintainRooms() called: new room maintenance task pending.");
			this.maintainRoomsTaskPending = true;
			this.notifyAll();
		}
	}
	
	
	
	@SuppressWarnings("unchecked")
	private void executeMaintainRoomsAlgorithm() {
		log("entering executeMaintainRoomsAlgorithm().");
		try {
			// determine all places
			Queue<Long> _remainingPlaceIds;
			_remainingPlaceIds = new LinkedList<Long>();
			String[] _allPlaceIns = m_comareasoner.getAllInstances("dora:Place");
			for (String _placeIns : _allPlaceIns) {
				_remainingPlaceIds.add(Long.valueOf(_placeIns.replaceAll("\\D","")));
			}
			debug("remaining places: " + _remainingPlaceIds);
			Collections.sort((List<Long>)_remainingPlaceIds);
			
			// get all rooms previously known from WM
			List<CASTData<ComaRoom>> _knownRoomsOnWM;
			_knownRoomsOnWM = new LinkedList<CASTData<ComaRoom>>();
			int _count=0;
			getMemoryEntriesWithData(ComaRoom.class, _knownRoomsOnWM, _count);
			log("loaded all room WMEs. no. of room WMEs: " + _knownRoomsOnWM.size());
			Collections.sort(_knownRoomsOnWM, new Comparator<CASTData<ComaRoom>>() {
					public int compare(CASTData<ComaRoom> arg0, CASTData<ComaRoom> arg1) {
					int x = arg0.getData().roomId;
					int y = arg1.getData().roomId;
					if(x < y) {
					return -1;
					} else if(x == y) {
					return 0;
					} else {
					return 1;
					}}});
			debug("sorted _knownRoomsOnWM: " + _knownRoomsOnWM);

			// for each known room:
			for (CASTData<ComaRoom> comaRoomWME : _knownRoomsOnWM) {
				ComaRoom _currentRoomStruct = comaRoomWME.getData();
				String _seedPlaceInstance = _currentRoomStruct.seedPlaceInstance;
				Long _seedPlaceId = Long.valueOf(_currentRoomStruct.seedPlaceInstance.replaceAll("\\D",""));
				logRoom(_currentRoomStruct, "current room, as read from WM. ", comaRoomWME.getID());

				// check if the current room's seed is a Doorway
				// or if the current room's seed is not in the list of remaining places,
				// which means that that seed must already be in another room.
				// in both cases, the existing room is obsolete and can be deleted
				if (m_comareasoner.isInstanceOf(_seedPlaceInstance, "dora:Doorway")
						|| !_remainingPlaceIds.contains(_seedPlaceId)) {
					log("current room's seed has turned into a doorway or the seed has been merged with an existing room.");
					// if the current room's seed is a Doorway
					// remove the seed from the remaining places
					_remainingPlaceIds.remove(_seedPlaceId);
					// and delete the current room from WM!
					// TODO does this really work? i.e., is ID() the correct arg?
					deleteFromWorkingMemory(comaRoomWME.getID());
					logRoom(_currentRoomStruct, "deleted obsolete room WME ", comaRoomWME.getID());
					// 2010-09-13-YR2 deleteRoomProxy(_currentRoomStruct, comaRoomWME.getID());
					log("deleted room WME and corresponding room proxy.");
					m_comareasoner.deleteInstance("dora:room" + _currentRoomStruct.roomId);
					log("deleted instance dora:room" + _currentRoomStruct.roomId + " from the coma reasoner");
				} else {
					// otherwise -- i.e., if the current room's seed place is not a Doorway 
					log("current room's seed is not a doorway. going to maintain.");
					// check whether that room has changed at all...
					boolean _hasChanged = false;
					
					// get places in the same room as the seed
					String[] _placesInTheSameRoom = 
						m_comareasoner.getRelatedInstancesByRelation(_seedPlaceInstance,"dora:sameRoomAs");
					Set<Long> _setOfPlaceIDsInTheSameRoom = new HashSet<Long>();
					int i=0;
					for (String _placeIns : _placesInTheSameRoom) {
						Long _currPlaceID = Long.valueOf(_placeIns.replaceAll("\\D",""));
						_setOfPlaceIDsInTheSameRoom.add(_currPlaceID);
						m_comareasoner.addRelation((_placeIns.startsWith(":") ? "dora" + _placeIns : _placeIns),
								"dora:constituentOfRoom", 
								"dora:room" + _currentRoomStruct.roomId);
						log("added relation to the coma reasoner: " 
								+ (_placeIns.startsWith(":") ? "dora" + _placeIns : _placeIns) 
								+ " dora:constituentOfRoom " 
								+ "dora:room" + _currentRoomStruct.roomId);
						i++;
					}
					// add the seed to the list of contained places to be written to WM!
					_setOfPlaceIDsInTheSameRoom.add(_seedPlaceId);
					m_comareasoner.addRelation(_seedPlaceInstance, "dora:constituentOfRoom", "dora:room" + _currentRoomStruct.roomId);
					log("added relation to the coma reasoner: " + _seedPlaceInstance + " dora:constituentOfRoom " + "dora:room" + _currentRoomStruct.roomId);

					// discard the found places from the set of remaining places
					_remainingPlaceIds.removeAll(_setOfPlaceIDsInTheSameRoom);

					// now construct an array to be written to WM
					long[] _arrayOfPlaceIDsInTheSameRoom = new long[_setOfPlaceIDsInTheSameRoom.size()];
					int j=0;
					for (Long _currPlaceID: _setOfPlaceIDsInTheSameRoom) {
						_arrayOfPlaceIDsInTheSameRoom[j]=_currPlaceID.longValue();
						j++;
					}
					// now check whether that has changed at all, 
					// else we don't have to create WM and binder traffic!
					Set<Long> _oldSetOfContainedPlaces = new HashSet<Long>();
					for (long _currKnownContainedPlaceID : _currentRoomStruct.containedPlaceIds) {
						_oldSetOfContainedPlaces.add(_currKnownContainedPlaceID);
					}
					if (!(_oldSetOfContainedPlaces.equals(_setOfPlaceIDsInTheSameRoom))) {
						_hasChanged = true;
					}
					
					// removed for Dora yr2: room categories inferred by Andrzej
					/*
					log("checking whether new room concepts are known...");
					if (!new TreeSet<String>(Arrays.asList(_currentRoomStruct.concepts)).
							equals(new TreeSet<String>(Arrays.asList(
									m_comareasoner.getAllConcepts("dora:room" + _currentRoomStruct.roomId))))) {
						// now refresh the room concepts
						_currentRoomStruct.concepts = 
							m_comareasoner.getAllConcepts("dora:room" + _currentRoomStruct.roomId);
						log("room concept list has changed!");
						_hasChanged = true;
					} */
					
					if (_hasChanged) {
						// update the contained places in the room struct
						_currentRoomStruct.containedPlaceIds=_arrayOfPlaceIDsInTheSameRoom;
						
						// new interface for Dora yr2: remove categories when room has changed
						// so that Andrzej can overwrite it
						_currentRoomStruct.categories=new ProbabilityDistribution();
						
						// now overwrite the existing room WME
						overwriteWorkingMemory(comaRoomWME.getID(),_currentRoomStruct);
						logRoom(_currentRoomStruct, "updated room WME. ", comaRoomWME.getID());
						// 2010-09-13-YR2 maintainRoomProxy(_currentRoomStruct, comaRoomWME.getID());
					} else {
						logRoom(_currentRoomStruct, "did not update room WME or proxy. Room hasn't changed. ", comaRoomWME.getID());
					}
				}
				debug("remaining places: " + _remainingPlaceIds);
			} // end for each room loop
			
			// for each remaining place
			while (!_remainingPlaceIds.isEmpty()) {
//			for (Long _remainingPlace : _remainingPlaceIds) {
				Long _remainingPlace = _remainingPlaceIds.poll();
				// check if current place can be a seed, i.e., it is not a doorway
				String _currentplaceInstance = "dora:place"+_remainingPlace;
				if (m_comareasoner.isInstanceOf(_currentplaceInstance, "dora:Doorway")) {
					// current place is a doorway
					debug(_currentplaceInstance + " is a doorway. discarding this place...");
					_remainingPlaceIds.remove(_remainingPlace);
				} else {
					// current place can be a seed for a new room
					log(_currentplaceInstance + " serving as seed for a new room");
					// create new room
					ComaRoom _newRoom = new ComaRoom(m_roomIndexCounter++, _currentplaceInstance, new long[0], new ProbabilityDistribution());
					
					// create new room on the reasoner
					m_comareasoner.addInstance("dora:room" + _newRoom.roomId, "dora:PhysicalRoom");
					log("created new instance " + "dora:room" + _newRoom.roomId +  " of concept dora:PhysicalRoom");
					
					// now initialize the room concepts
					// changed for Dora yr2!
					//_newRoom.concepts = m_comareasoner.getAllConcepts("dora:room" + _newRoom.roomId);
					_newRoom.categories = new ProbabilityDistribution();
					
					String[] _placesInTheSameRoom = 
						m_comareasoner.getRelatedInstancesByRelation(_currentplaceInstance,"dora:sameRoomAs");
					Set<Long> _setOfPlaceIDsInTheSameRoom = new HashSet<Long>();
					int i=0;
					for (String _placeIns : _placesInTheSameRoom) {
						Long _currPlaceID = Long.valueOf(_placeIns.replaceAll("\\D",""));
						m_comareasoner.addRelation(
								(_placeIns.startsWith(":") ? "dora" + _placeIns : _placeIns), 
								"dora:constituentOfRoom", 
								"dora:room" + _newRoom.roomId);
						log("added relation to the coma reasoner: " 
								+ (_placeIns.startsWith(":") ? "dora" + _placeIns : _placeIns) 
								+ " dora:constituentOfRoom " 
								+ "dora:room" + _newRoom.roomId);
						_setOfPlaceIDsInTheSameRoom.add(_currPlaceID);
						i++;
					}
					// add the seed to the list of contained places to be written to WM!
					_setOfPlaceIDsInTheSameRoom.add(_remainingPlace);
					m_comareasoner.addRelation("dora:place" + _remainingPlace, "dora:constituentOfRoom", "dora:room" + _newRoom.roomId);
					log("added relation to the coma reasoner: dora:place" + _remainingPlace + " dora:constituentOfRoom " + "dora:room" + _newRoom.roomId);

					// discard the found places from the set of remaining places
					_remainingPlaceIds.removeAll(_setOfPlaceIDsInTheSameRoom);

					// now construct an array to be written to WM
					long[] _arrayOfPlaceIDsInTheSameRoom = new long[_setOfPlaceIDsInTheSameRoom.size()];
					int j=0;
					for (Long _currPlaceID: _setOfPlaceIDsInTheSameRoom) {
						_arrayOfPlaceIDsInTheSameRoom[j]=_currPlaceID.longValue();
						j++;
					}
					// update the contained places in the room struct
					_newRoom.containedPlaceIds=_arrayOfPlaceIDsInTheSameRoom;
					// write out the new room to WM
					String _newRoomWMEID = newDataID();
					addToWorkingMemory(_newRoomWMEID, _newRoom);
					logRoom(_newRoom, "added new room WME. ", _newRoomWMEID);
					// 2010-09-13-YR2 maintainRoomProxy(_newRoom, _newRoomWMEID);
				} // end else create a new room for non-doorway seeds
				debug("remaining places: " + _remainingPlaceIds);
			} // end for each remaining place loop
		} catch (NumberFormatException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (ConsistencyException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (PermissionException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		logInstances("owl:Thing");
	}
	
	/* private boolean maintainRoomProxy(ComaRoom _comaRoom, String _wmid) {
		log("maintainRoomProxy() called");
		
		if (m_proxyMarshall!=null) {
			// current room UID:
			String _currRoomUID = "area"+_comaRoom.roomId;
			
			// check if proxy already exists
			if (m_existingRoomProxies.contains(_currRoomUID)) {
				log("updating an existing room proxy.");
				
			} else {
				log("creating a new room proxy.");
				m_proxyMarshall.addProxy("room", _currRoomUID, 1, 
						new WorkingMemoryPointer(new WorkingMemoryAddress(_wmid, this.getSubarchitectureID()), _comaRoom.ice_id()));
				m_existingRoomProxies.add(_currRoomUID);
				
				// room ID:
				Feature _roomIDFtr = new Feature();
				_roomIDFtr.featlabel = "roomID";
				_roomIDFtr.alternativeValues = new FeatureValue[1];
				_roomIDFtr.alternativeValues[0] = new IntegerValue(1, getCASTTime(), _comaRoom.roomId);
				m_proxyMarshall.addFeature("room", _currRoomUID, _roomIDFtr);
			}
			
			// areaclasses
			// first clean existing areaclass facts
			String _areaclassFeatlabel = "areaclass";
			m_proxyMarshall.deleteFeature("room", _currRoomUID, _areaclassFeatlabel);
			// now create individual feature-value pairs for all concepts
			boolean _unknown = true;
			if (_comaRoom.concepts.length!=0) {
				for (String _currConcept : _comaRoom.concepts) {
					if (_currConcept.equals("owl:Thing") 
							|| _currConcept.endsWith("PhysicalRoom") 
							|| _currConcept.endsWith("Portion_of_Space") 
							|| _currConcept.endsWith("Portion_of_space")) {
						continue;
					}
					_currConcept = _currConcept.replace(":", "");
					log("current concept for proxy feature areaclass: " + _currConcept);
					FeatureValue[] _alternativeValues = new FeatureValue[1];
					_alternativeValues[0] = new StringValue(1, getCASTTime(), _currConcept);
					Feature _classFtr = new Feature(_areaclassFeatlabel, _alternativeValues);
					m_proxyMarshall.addFeature("room", _currRoomUID, _classFtr);
					_unknown = false;
//					log("TEMPORARY HACK: exiting loop over all area classes until the binder can handle multiple identical features!");
//					break;
				}
			} 
			if (_unknown) {
				log("areaclass set to unknown.");
				FeatureValue[] _alternativeValues = new FeatureValue[1];
				_alternativeValues[0] = new StringValue(1, getCASTTime(), "unknown");
				Feature _classFtr = new Feature(_areaclassFeatlabel, _alternativeValues);
				m_proxyMarshall.addFeature("room", _currRoomUID, _classFtr);
			}
			
			
			m_proxyMarshall.commitFeatures("room", _currRoomUID);
			log("maintained proxy for " + _currRoomUID);

			// containment
			HashSet<String> prevKnownRels = m_existingRelationProxies.remove(_currRoomUID);
			if (prevKnownRels==null) prevKnownRels=new HashSet<String>();
			HashSet<String> trueKnownRels = new HashSet<String>();
			for (long _currContPlID : _comaRoom.containedPlaceIds) {
				String _relUID = _comaRoom.roomId + ":" + _currContPlID;
				if (prevKnownRels.remove(_relUID)) {
					// the relation was known before,
					// so it is already on the binder
					// doing nothing
				} else {
					m_proxyMarshall.addRelation("contains", _relUID, 
							"room", _currRoomUID, "place", new Long(_currContPlID).toString(), 
							1, new WorkingMemoryPointer(new WorkingMemoryAddress(_wmid, this.getSubarchitectureID()), _comaRoom.ice_id()));
					Feature _containsFtr = new Feature();
					_containsFtr.featlabel = "contains";
					_containsFtr.alternativeValues = new FeatureValue[1];
					_containsFtr.alternativeValues[0] = new BooleanValue(1, getCASTTime(), true);
					m_proxyMarshall.addFeature("contains", _relUID, _containsFtr);
					m_proxyMarshall.commitFeatures("contains", _relUID);
				}
				trueKnownRels.add(_relUID);
			}
			// now each remaining relation must be deleted because it no longer holds
			for (String _obsoleteRel : prevKnownRels) {
				m_proxyMarshall.deleteProxy("contains", _obsoleteRel);
				String[] _rel = _obsoleteRel.split(":");
				m_comareasoner.deleteRelation("dora:place" + _rel[1], "dora:constituentOfRoom", "dora:room" + _rel[0]);
				log("deleted obsolete containment relations from binder and reasoner: " +
						"dora:place" + _rel[1] + " dora:constituentOfRoom " + "dora:room" + _rel[0]);
			}
			// update the set of known relations
			m_existingRelationProxies.put(_currRoomUID, trueKnownRels);
			
			return true;
		} else return false;
	} */
	
	/* private boolean deleteRoomProxy(ComaRoom _comaRoom, String _wmid) {
		if (m_proxyMarshall!=null) {
			// current room UID: * This class, for the time being, also contains the interface to the mighty *BINDER*!

			String _currRoomUID = "area"+_comaRoom.roomId;

			// first delete its relations:
			HashSet<String> prevKnownRels = m_existingRelationProxies.remove(_currRoomUID);
			for (String _obsoleteRel : prevKnownRels) {
				m_proxyMarshall.deleteProxy("contains", _obsoleteRel);
			}
			
			// then delete the actual room proxy
			m_proxyMarshall.deleteProxy("room", _currRoomUID);
			m_existingRoomProxies.remove(_currRoomUID);
			logRoom(_comaRoom, "deleted room proxy for: ", _wmid);
			return true;
		} else return false;
	} */
	
	private void logRoom(ComaRoom _room, String _prefix, String _wmid) {
		if (_wmid==null) _wmid="";
		
		// create readable representation of the contained place IDs
		StringBuffer _containedPlaces = new StringBuffer();
		_containedPlaces.append("[");
		boolean _insertComma=false;
		for (long _placeID : _room.containedPlaceIds) {
			if (_insertComma) _containedPlaces.append(",");
			_containedPlaces.append(_placeID);
			_insertComma=true;
		}
		_containedPlaces.append("]");
		
		// create readable representation of the area concepts
		// removed for Dora yr2
		/*
		StringBuffer _areaConcepts = new StringBuffer();
		_areaConcepts.append("[");
		_insertComma=false;
		for (String _rcon : _room.concepts) {
			if (_insertComma) _areaConcepts.append(",");
			_areaConcepts.append(_rcon);
			_insertComma=true;
		} 

		_areaConcepts.append("]");
		*/

		log(_prefix + "[[" +
				"room ID : " + _room.roomId +
				 " | seed place instance: " + _room.seedPlaceInstance +
				 " | contained place IDs: " + _containedPlaces +
				//  " | room concepts: " + _areaConcepts +							
				 (_wmid.equals("") ? " | WME ID: " + _wmid : "") +
				 "]]");
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
////			addToWorkingMemory(new WorkingMemoryAddress(_newPlaceProxy.entityID,m_bindingSA), _newPlaceProxy);
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
////			addToWorkingMemory(new WorkingMemoryAddress(_newPlaceProxy.entityID,m_bindingSA), _newPlaceProxy);
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