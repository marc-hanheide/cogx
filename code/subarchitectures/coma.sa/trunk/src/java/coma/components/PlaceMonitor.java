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
import DefaultData.ChainGraphInferencerServerInterfacePrx;
import SpatialData.MapLoadStatus;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import SpatialData.SpatialRelation;
import SpatialProbabilities.ProbabilityDistribution;
import SpatialProperties.BinaryValue;
import SpatialProperties.ConnectivityPathProperty;
import SpatialProperties.DiscreteProbabilityDistribution;
import SpatialProperties.GatewayPlaceProperty;
import SpatialProperties.ObjectPlaceProperty;
import SpatialProperties.PropertyValue;
import SpatialProperties.RoomHumanAssertionPlaceProperty;
import VisionData.VisualObject;

import coma.aux.ComaGBeliefHelper;
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
import cast.cdl.WorkingMemoryEntry;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTData;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.NegatedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationRequest;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

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
	private boolean mapLoadMode = false;
	private boolean mapLoadPlacesWritten = false;
	private boolean mapLoadRoomsWritten = false;
	private WorkingMemoryAddress m_mapLoadStatusAddress;
	
	private String m_comareasoner_component_name;
	private ComaReasonerInterfacePrx m_comareasoner;
	private String m_chaingraph_component_name;
	private ChainGraphInferencerServerInterfacePrx m_chaingraphinferencer;
	
	private String m_spatial_sa_name;
	
	private HashSet<Long> m_placeholders;
	private HashSet<Long> m_trueplaces;
	private Map<Long, HashSet<WorkingMemoryAddress>> m_tempAdjacencyStore;
	private Map<WorkingMemoryAddress, ConnectivityPathProperty> m_pathWMAtoPathObject;
	
	private int m_roomIndexCounter = 0;
	private int m_objectIndexCounter = 0;
	//private HashSet<String> m_existingRoomProxies;
	//private HashMap<String,HashSet<String>> m_existingRelationProxies;
	
	private boolean m_createDummyObjects = false;
	
	private boolean maintainRoomsTaskPending = false;

	public void configure(Map<String, String> args) {
		log("configure() called");

		if (args.containsKey("--chaingraph-name")) {
			m_chaingraph_component_name=args.get("--chaingraph-name");
		}
		else {
			log("Argument --chaingraph-name not specified.");
		}
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
		m_tempAdjacencyStore = Collections.synchronizedMap(new HashMap<Long, HashSet<WorkingMemoryAddress>>());
		m_pathWMAtoPathObject = Collections.synchronizedMap(new HashMap<WorkingMemoryAddress, ConnectivityPathProperty>());
		//m_existingRoomProxies = new HashSet<String>();
		//m_existingRelationProxies = new HashMap<String,HashSet<String>>();
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
		// track connectivity deletion
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(ConnectivityPathProperty.class, WorkingMemoryOperation.DELETE), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
				processDeletedConnectivityPath(_wmc);
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

		// track asserted room labels (aka human-attributed beliefs)
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(dBelief.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) 
			throws CASTException {
				processAddedDBelief(_wmc);
			}
		});

		// track grounded beliefs of objects (and ignore grounded beliefs about rooms)
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(GroundedBelief.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) 
			throws CASTException {
				processAddedGBelief(_wmc);
			}
		});

		// track grounded beliefs of objects (and ignore grounded beliefs about rooms)
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(GroundedBelief.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) 
			throws CASTException {
				processAddedGBelief(_wmc);
			}
		});

		// add state handling for a map loading (for non-exploring dora y4)
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(MapLoadStatus.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) 
			throws CASTException {
				processAddedMapLoadStatus(_wmc);
			}
		});
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(MapLoadStatus.class, WorkingMemoryOperation.OVERWRITE), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) 
			throws CASTException {
				processOverwrittenMapLoadStatus(_wmc);
			}
		});

		
		// track objects found by AVS
		// now done via GroundedBeliefs
		//addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(ObjectPlaceProperty.class, WorkingMemoryOperation.ADD), 
		//		new WorkingMemoryChangeReceiver() {
		//	public void workingMemoryChanged(WorkingMemoryChange _wmc) 
		//	throws CASTException {
		//		processAddedObjectProperty(_wmc);
		//	}
		//});

		
		// initialize ice server connections
		
		// connection to the coma reasoner
		try {
			if (m_comareasoner_component_name!=null) log("initiating connection to Ice server " + m_comareasoner_component_name);
			if (m_comareasoner_component_name!=null) m_comareasoner = getIceServer(m_comareasoner_component_name, comadata.ComaReasonerInterface.class , comadata.ComaReasonerInterfacePrx.class);
			if (m_comareasoner!=null) log("initiated comareasoner connection");
			else throw new CASTException();
		} catch (CASTException e) {
			logException(e);
			e.printStackTrace();
			log("Connection to the coma reasoner Ice server at "+ m_comareasoner_component_name + " failed! Exiting. (Specify the coma reasoner component name using --reasoner-name)");
			System.exit(-1);
		}	
		
		// create default scene instance
		m_comareasoner.addInstance("dora:defaultScene", "dora:Scene");
		
		// connection to chaingraph inferencer

		if (m_chaingraph_component_name!=null) {
			log("initiating connection to Ice server " + m_chaingraph_component_name);
			try {
				m_chaingraphinferencer = getIceServer(m_chaingraph_component_name, DefaultData.ChainGraphInferencerServerInterface.class , DefaultData.ChainGraphInferencerServerInterfacePrx.class);
				log("initiated chaingraph connection");
				
				// query chaingraph inferencer for object and room categories
				String[] objCats = m_chaingraphinferencer.getObjectCategories();
				StringBuffer _objTypes = new StringBuffer();
				for (String _currObjType : objCats) {
					if (_objTypes != null) _objTypes.append(", ");
					_objTypes.append(_currObjType);
				}
				log("Queried the chaingraph/default.sa, and the known object categories are " + _objTypes.toString());
				for (String _currObjType : objCats) {
					String _addQuery = 
						"INSERT { dora:" + ComaHelper.firstCapRestSmall(_currObjType) + 
						" rdfs:subClassOf " + 
						" dora:Object }";
					m_comareasoner.executeSPARQL(_addQuery);
					log("executed " + _addQuery);
				}
				
				String[] roomCats = m_chaingraphinferencer.getRoomCategories();
				StringBuffer _roomTypes = new StringBuffer();
				for (String _currRoomType : roomCats) {
					if (_roomTypes != null) _roomTypes.append(", ");
					_roomTypes.append(_currRoomType);
				}
				log("Queried the chaingraph/default.sa, and the known room categories are " + _roomTypes.toString());
				for (String _currRoomType : roomCats) {
					String _addQuery = 
						"INSERT { dora:" + ComaHelper.firstCapRestSmall(_currRoomType) + 
						" rdfs:subClassOf " + 
						" dora:PhysicalRoom }";
					m_comareasoner.executeSPARQL(_addQuery);
					log("executed " + _addQuery);
				}
				
			} catch (CASTException e) {
				logException(e);
				e.printStackTrace();
			}
		}
		else { 
			log("No chaingraph inferencer component name specified. Ignoring...");
		}
	}

	@Override
	protected void runComponent() {
		while (isRunning()) {
			try {
				synchronized (this) {
					// wait while the map is being loaded 
					if (this.mapLoadMode) {
						if (!this.mapLoadPlacesWritten) {
							log("map load mode active and places not yet written. waiting...");
							this.wait();
							continue;
						}
					}

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
				// handled in finally...
				// this.unlockComponent();
			}  catch (InterruptedException e) {
				logException(e);
			} finally {
				log("entered finally block in runComponent()");
				this.unlockComponent();
			}
		}
	}
	
	
	private void processAddedPlace(WorkingMemoryChange _wmc) {
		debug("Got a callback for an ADDed Place WME:");
		// initialize spatial.sa name!
		if (m_spatial_sa_name==null) {
			m_spatial_sa_name = _wmc.address.subarchitecture;
			log("initialized spatial.sa name = " + m_spatial_sa_name);
		}

		// read place struct from WM
		Place _newPlaceNode;
		try {
			_newPlaceNode = getMemoryEntry(_wmc.address, Place.class);
		} catch (DoesNotExistOnWMException e) {
			log("freshly ADDed Place ceased to exist before I could load it. Exiting callback method!");
			logException(e);
			return;
		} catch (UnknownSubarchitectureException e) {
			logException(e);
			return;
		}
		log("processAddedPlace() called: Place ID = " + _newPlaceNode.id + " " +
				"Place status = " + (_newPlaceNode.status.equals(PlaceStatus.PLACEHOLDER) ? "PLACEHOLDER" : "TRUEPLACE"));

		// check the Place status
		if (_newPlaceNode.status.equals(PlaceStatus.TRUEPLACE)) { 
			// TRUEPLACE block
			log("create dora:Place instance " + "dora:place"+_newPlaceNode.id);
			synchronized (m_comareasoner) {
				m_comareasoner.addInstance("dora:place"+_newPlaceNode.id, "dora:Place");
			}
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
					//						createObject(new ObjectPlaceProperty(_newPlaceNode.id,
					//								null, null, true, false, "dummyObject"));
				}
			}

			// creating an initial seed room for the first Place added!
			if (_newPlaceNode.id==0) maintainRooms();

			// process pending paths
			HashSet<WorkingMemoryAddress> _pendingPaths = m_tempAdjacencyStore.remove(_newPlaceNode.id);
			if (_pendingPaths!=null) {
				debug("process pending paths:");
				for (WorkingMemoryAddress _workingMemoryAddress : _pendingPaths) {
					ConnectivityPathProperty _currPendingPath;
					try {
						log("trying to load pending path");
						_currPendingPath = getMemoryEntry(_workingMemoryAddress, ConnectivityPathProperty.class);
						log("successfully loaded pending path from WM");
						log("add relation to coma: " + "dora:place"+_currPendingPath.place1Id + " dora:adjacent " + "dora:place"+_currPendingPath.place2Id);
						synchronized (m_comareasoner) {
							m_comareasoner.addRelation("dora:place"+_currPendingPath.place1Id, "dora:adjacent", "dora:place"+_currPendingPath.place2Id);
						}
					} catch (DoesNotExistOnWMException e) {
						log("could not load pending path from WM -- ceased to exist. ignoring this path...");
						logException(e);
						continue;
					} catch (UnknownSubarchitectureException e) {
						log("could not load pending path from WM -- unknown SA exception. ignoring this path...");
						logException(e);
						continue;
					}
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
	}
	
	private void processAddedMapLoadStatus(WorkingMemoryChange _wmc) {
		log("received ADDed MapLoadStatus -- setting mapLoadMode to TRUE");
		synchronized(this) {
			this.mapLoadMode = true;
			this.m_mapLoadStatusAddress = _wmc.address;
			log("mapLoadMode set to " + Boolean.toString(this.mapLoadMode));
			this.notifyAll();
		}
	}
	private void processOverwrittenMapLoadStatus(WorkingMemoryChange _wmc) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		MapLoadStatus _mlstatus = getMemoryEntry(_wmc.address, MapLoadStatus.class);
		log("received OVERWRITTEN MapLoadStatus");
		synchronized(this) {
			log("mapLoadPlacesWritten is currently set to " + Boolean.toString(this.mapLoadPlacesWritten));
			if (!this.mapLoadPlacesWritten) {
				log("MapLoadStatus WME reports placesWritten = " + Boolean.toString(_mlstatus.placesWritten));
				if (_mlstatus.placesWritten) {
					this.mapLoadPlacesWritten = true;
					log("mapLoadPlacesWritten set to " + Boolean.toString(this.mapLoadPlacesWritten));
				} else {
					log("no  need to change mapLoadPlacesWritten (inner block) -- map loader isn't done writing the places yet!");
				}
			} else {
				log("no need to change mapLoadPlacesWritten (outer block) -- already done so before!");
			}
			this.notifyAll();
		}
	}
	
	private void processAddedConnectivityPath(WorkingMemoryChange _wmc) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		// get path from WM
		ConnectivityPathProperty _path = getMemoryEntry(_wmc.address, ConnectivityPathProperty.class);
		log("processAddedConnectivityPath() called: got a callback for an ADDED ConnectivityPathProperty between " + _path.place1Id + " and " +_path.place2Id);

		// todo handle lookup for deleted paths WMEs!
		m_pathWMAtoPathObject.put(_wmc.address, _path);
		
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
				synchronized (m_comareasoner) {
					m_comareasoner.addRelation("dora:place"+_path.place1Id, "dora:adjacent", "dora:place"+_path.place2Id);
				}
				log("added adjacency relation: dora:place"+_path.place1Id + " dora:adjacent " + " dora:place"+_path.place2Id);
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
	
	private void processDeletedConnectivityPath(WorkingMemoryChange _wmc) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		// cannot get path from WM
		// retrieving old copy of the path
		ConnectivityPathProperty _deletedPath = m_pathWMAtoPathObject.get(_wmc.address);
		log("processDeletedConnectivityPath() called: got a callback for a DELETed ConnectivityPathProperty between " + _deletedPath.place1Id + " and " +_deletedPath.place2Id);

		synchronized (this.m_comareasoner) {
			m_comareasoner.deleteRelation("dora:place"+_deletedPath.place1Id, "dora:adjacent", "dora:place"+_deletedPath.place2Id);
			log("deleted adjacency relation: dora:place"+_deletedPath.place1Id + " dora:adjacent " + " dora:place"+_deletedPath.place2Id);
		}
		
		maintainRooms();
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

		synchronized (m_comareasoner) {
			m_comareasoner.addInstance("dora:place"+_gateProp.placeId, "dora:Doorway");
		}
		
		// trigger room creation, splitting, merging, maintenance
		maintainRooms();
	}
	
	//	private boolean createObject(ObjectPlaceProperty _objProp) {
	//		// establish the ontology member names
	//		String category = "dora:" + ComaHelper.firstCap(((SpatialProperties.StringValue)_objProp.mapValue).value);
	//		String placeIns = "dora:place"+_objProp.placeId;
	//		String inRel = "dora:in";
	//		String containsRel = "dora:contains";
	//		
	//		log("createObject of category " + category + " in place " + placeIns + " called.");
	//		
	//		// check whether the given place already contains an instance of the given category
	//		String [] objsInPlace = m_comareasoner.getRelatedInstancesByRelation(placeIns, containsRel);
	//		for (String obj : objsInPlace) {
	//			if (obj.startsWith(":")) obj = "dora" + obj;
	//			if (m_comareasoner.isInstanceOf(obj, category)) {
	//				log("object of the given category already exists in the given place - doing nothing else.");
	//				return false;
	//				// if such an object exists, don't create a new instance!
	//			}
	//		}
	//		
	//		String objIns = "dora:object" + m_objectIndexCounter++; 
	//		
	//		log("going to add new instance " + objIns + " of category " + category);
	//		m_comareasoner.addInstance(objIns, category);
	//		log("going to add new relation " + objIns + " " + inRel + " " + placeIns);
	//		m_comareasoner.addRelation(objIns, inRel, placeIns);
	//		return true;
	//	}
	
		/**
		 * This method is invoked whenever a new dBelief is written to some WorkingMemory.
		 * 
		 * It is meant to handle cases in which a user/human-asserted belief about a room category/label 
		 * is added to WM. This is the case, e.g., when a human answers a question about a room category.
		 * 
		 * @param _wmc
		 * @throws DoesNotExistOnWMException
		 * @throws UnknownSubarchitectureException
		 */
		private void processAddedDBelief(WorkingMemoryChange _wmc) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
			// get dBelief from WM
			dBelief _belief = getMemoryEntry(_wmc.address, dBelief.class);
			log("processAddedDBelief() called: got a callback for an ADDED dBelief");
			
			// we are only interested in certain dBeliefs
			EpistemicStatus _epi= _belief.estatus;
			if (_epi instanceof AttributedEpistemicStatus) {
				// only react on human-asserted facts
				if (((AttributedEpistemicStatus) _epi).attribagents.contains("human") && (_belief.type.equals("fact"))) {
					log("received a belief of type fact attributed by a human");
					
					// the rest that follows makes some ugly assumptions about the structure within the human-attributed factual belief
					if (_belief.content instanceof CondIndependentDistribs) {
						// this try has the intention of catching bad class cast operations 
						// -- which are the result of not met assumptions regarding the belief structure
						try {
							HashSet<String> _assertedRoomCats = new HashSet<String>(); 

							DistributionValues identityVals = ((BasicProbDistribution) ((CondIndependentDistribs) _belief.content).distribs.get("identity")).values;
							for (FormulaProbPair _currPair : ((FormulaValues) identityVals).values) {
								if (_currPair.val instanceof ElementaryFormula) {
									log("current identity: " + ((ElementaryFormula) _currPair.val).prop + " with " + _currPair.prob);
									if (_currPair.prob>=0.5) {
										String assertionCat = ((ElementaryFormula) _currPair.val).prop;
										if (assertionCat.equals("hall")) assertionCat = "corridor";
										_assertedRoomCats.add(assertionCat);
									}
								} else if (_currPair.val instanceof NegatedFormula) {
									log("current identity: NOT " + ((ElementaryFormula) ((NegatedFormula) _currPair.val)
											.negForm).prop + " with " + _currPair.prob);
									if (_currPair.prob>=0.5) {
										// TODO handle negative feedback according to distribution, mapValue, mapValueReliable!
										String assertionCat = ((ElementaryFormula) ((NegatedFormula) _currPair.val)
												.negForm).prop;
										if (assertionCat.equals("hall")) assertionCat = "corridor";
										_assertedRoomCats.add("!" + assertionCat);
									}
								} else {
									log("the received answer was neither a ElementaryFormula nor a NegatedFormula...");
								}
								
							}
							StringBuffer _assertedLabels = new StringBuffer();
							for (String _label : _assertedRoomCats) {
								_assertedLabels.append(_label).append(" ");
							}

							// this should be WMPs -- but the type field does not seem to be properly set anyway
							HashSet<WorkingMemoryAddress> _gBeliefs = new HashSet<WorkingMemoryAddress>();
							DistributionValues aboutVals = ((BasicProbDistribution) ((CondIndependentDistribs) _belief.content).distribs.get("about")).values;
							for (FormulaProbPair _currPair : ((FormulaValues) aboutVals).values) {
								log("current about: WMID=" + ((PointerFormula) _currPair.val).pointer.id + "WMSA=" + ((PointerFormula) _currPair.val).pointer.subarchitecture + " with " + _currPair.prob);
								if (_currPair.prob>=0.5) _gBeliefs.add(new WorkingMemoryAddress(((PointerFormula) _currPair.val).pointer.id, ((PointerFormula) _currPair.val).pointer.subarchitecture));
							} 

							for (WorkingMemoryAddress _gBelief : _gBeliefs) {
								GroundedBelief _currBelief = getMemoryEntry(_gBelief, GroundedBelief.class);

								// ok, the following iterator code works, 
								// but it might be jumping on false conclusions in case there exist multiple ancestors
								for (WorkingMemoryPointer _wmp : ((CASTBeliefHistory) _currBelief.hist).ancestors) {
									// naive sanity checking!
									if (_wmp.type.contains("ComaRoom")) {
										// load ComaRoomWME in order to check if a new category was asserted
										ComaRoom _currRoom = getMemoryEntry(_wmp.address, ComaRoom.class);
										log("Room with ID: " + _currRoom.roomId + " has new attributed labels: " + _assertedLabels.toString());

										// for now, associate the RoomPlaceProperty with the seed place of the room
										// that it has been resolved against -- later the assertion should be referenced
										// by the place at which it was made/received -- and that place should be used instead.
										// the current implementation does not account correctly for non-monotonic splits/merges of
										// rooms!
										for (String assertedRoomCat : _assertedRoomCats) {
											// for each belief that expresses a human assertion about a room category
											// we write a placeproperty -- there is no check for duplicates at this point!
											RoomHumanAssertionPlaceProperty roomCatAssertion = new RoomHumanAssertionPlaceProperty(
													Long.valueOf(_currRoom.seedPlaceInstance.replaceAll("\\D","")),
													new SpatialProperties.ProbabilityDistribution(), new PropertyValue(), // new StringValue(assertedRoomCat) ???
													true, false, assertedRoomCat);

											try {
												if (m_spatial_sa_name!=null) {
													addToWorkingMemory(new WorkingMemoryAddress(newDataID(), m_spatial_sa_name), roomCatAssertion);
												}
												else {
													log("cannot add RoomHumanAssertionPlaceProperty to WM: spatial SA name is unknown!");
												}
												//addToWorkingMemory(newDataID(), roomCatAssertion);
											} catch (AlreadyExistsOnWMException e) {
												logException(e);
												e.printStackTrace();
											}

										}

										/*
									// compare the set(!) of already known asserted room categories with the new set
									HashSet<String> prevKnownAssertedRoomCats = new HashSet<String>(Arrays.asList(_currRoom.assertedLabels)); // uncomment this
									// HashSet<String> prevKnownAssertedRoomCats = new HashSet<String>(); // comment this out
									if (!prevKnownAssertedRoomCats.equals(_assertedRoomCats)) {
										log("there are new asserted room categories not previosuly known.");
										HashSet<String> allKnownAssertedRoomsCats = new HashSet<String>();
										allKnownAssertedRoomsCats.addAll(prevKnownAssertedRoomCats);
										allKnownAssertedRoomsCats.addAll(_assertedRoomCats);
										String[] newAssertedLabels = allKnownAssertedRoomsCats.toArray(new String[allKnownAssertedRoomsCats.size()]);

										// if there is a new room category, we need to overwrite the room WME
										_currRoom.assertedLabels = newAssertedLabels; //uncomment this
										_currRoom.categories=new ProbabilityDistribution();
										// overwriting must be safe: lock + unlock WME
										debug("locking ComaRoom WME.");
										lockEntry(_wmp.address, WorkingMemoryPermissions.LOCKEDODR);
										try {
											debug("overwriting ComaRoom WME.");
											overwriteWorkingMemory(_wmp.address,_currRoom);
											debug("unlocking ComaRoom WME.");
											unlockEntry(_wmp.address);
											log("successfully overwritten ComaRoom WMD with new asserted labels.");
										} catch (ConsistencyException e) {
											log("There was a ConsistencyException when trying to overwrite the ComaRoom id="+_currRoom.roomId+" WME. " + e.getStackTrace());
											e.printStackTrace();
										} catch (PermissionException e) {
											log("There was a PermissionException when trying to overwrite the ComaRoom id="+_currRoom.roomId+" WME. " + e.getStackTrace());
											e.printStackTrace();
										}
									}	
										 */							
									} else {
										log("The ancestor points to a non-ComaRoom WME. Doing nothing.");
									}
								}						
							}
						} catch (ClassCastException cce) {
							log("Caught a ClassCastException. This means some assumption was not met. Doing nothing. Here's the transcript: " + cce.getStackTrace());
							logException(cce);
						}
					} else {
						debug("the belief content is not of type CondIndependentDistribs. Don't know what to do. Doing nothing.");
					}
				} else {
					debug("the attributed dBelief is not a human-asserted fact. Doing nothing.");
				}
			} else {
				debug("the added dBelief WME does not have an AttributedEpistemicStatus. Doing nothing.");
			}
		}


	/**
	 * This method is invoked whenever a new GroundedBelief is written to some WorkingMemory.
	 * 
	 * It is meant to handle cases when an object is found (e.g., by AVS etc).
	 * It currently ignores GroundedBeliefs about ComaRooms 
	 * (they're based on what is generated in here anyway).
	 * 
	 * @param _wmc
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	private void processAddedGBelief(WorkingMemoryChange _wmc) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		// get GroundedBelief from WM
		GroundedBelief _belief = getMemoryEntry(_wmc.address, GroundedBelief.class);
		log("processAddedGBelief() called: got a callback for an ADDED GroundedBelief");
		
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> gbProxy = 
			CASTIndependentFormulaDistributionsBelief.create(GroundedBelief.class, _belief);
		
		// check if it's a VisualObject:
        if (gbProxy.getType().equals(
                SimpleDiscreteTransferFunction
                        .getBeliefTypeFromCastType(VisualObject.class))) {
        	log("the ADDED GroundedBelief is about a VisualObject. Going to process it.");
        	
        	// first lock it
        	lockEntry(_wmc.address, WorkingMemoryPermissions.LOCKEDOD);
        	log("locked " + _wmc.address + " with LOCKEDOD");
        	
        	// process further
        	processVisObjGBelief(_belief);
        	
        	//add change filters
        	// add an OVERWRITE address filter
        	addChangeFilter(ChangeFilterFactory.createAddressFilter(_wmc.address,WorkingMemoryOperation.OVERWRITE), 
        			new WorkingMemoryChangeReceiver() {
    			public void workingMemoryChanged(WorkingMemoryChange _wmc) 
    			throws CASTException {
    				processOverwrittenVisObjGBelief(_wmc);
    			}
    		});
        	// add a DELETE address filter
			final String objInsName = "dora:" + ComaGBeliefHelper.getGBeliefComaIndividualName(_belief);
        	addChangeFilter(ChangeFilterFactory.createAddressFilter(_wmc.address,WorkingMemoryOperation.DELETE), 
        			new WorkingMemoryChangeReceiver() {
    			public void workingMemoryChanged(WorkingMemoryChange _wmc) 
    			throws CASTException {
    				// if GBelief is deleted, delete the corresponding instance from coma!
    				log("Got a callback for a DELETED VisualObject GroundedBelief. " +
    						"Going to delete the corresponding instance from coma ABox");
    		        synchronized (m_comareasoner) {
    		        	m_comareasoner.deleteInstance(objInsName);
    		        }
    		        log("deleted instance " + objInsName);}
    		});
        	
        	// finally, unlock the entry
        	try {
        		log("trying to unlock " + _wmc.address);
        		unlockEntry(_wmc.address);
				log("unlocked " + _wmc.address);
			} catch (ConsistencyException e) {
				log("could not unlock "+ _wmc.address);
				logException(e);
			}
			
			/*
			try {
				ReferenceGenerationRequest _fakeGREReq1 = new ReferenceGenerationRequest(_wmc.address, false, false, new LinkedList<String>());
				addToWorkingMemory(new WorkingMemoryAddress(newDataID(), getSubarchitectureID()), _fakeGREReq1);

				ReferenceGenerationRequest _fakeGREReq2 = new ReferenceGenerationRequest(_wmc.address, false, true, new LinkedList<String>());
				addToWorkingMemory(new WorkingMemoryAddress(newDataID(), getSubarchitectureID()), _fakeGREReq2);

				ReferenceGenerationRequest _fakeGREReq3 = new ReferenceGenerationRequest(_wmc.address, true, false, new LinkedList<String>());
				addToWorkingMemory(new WorkingMemoryAddress(newDataID(), getSubarchitectureID()), _fakeGREReq3);

				ReferenceGenerationRequest _fakeGREReq4 = new ReferenceGenerationRequest(_wmc.address, true, true, new LinkedList<String>());
				addToWorkingMemory(new WorkingMemoryAddress(newDataID(), getSubarchitectureID()), _fakeGREReq4);
				
				LinkedList<String> _omissionList = new LinkedList<String>();
				_omissionList.add("identity");
				ReferenceGenerationRequest _fakeGREReq5 = new ReferenceGenerationRequest(_wmc.address, true, false, _omissionList);
				addToWorkingMemory(new WorkingMemoryAddress(newDataID(), getSubarchitectureID()), _fakeGREReq5);

				ReferenceGenerationRequest _fakeGREReq6 = new ReferenceGenerationRequest(_wmc.address, true, true, _omissionList);
				addToWorkingMemory(new WorkingMemoryAddress(newDataID(), getSubarchitectureID()), _fakeGREReq6);
			} catch (AlreadyExistsOnWMException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			*/
			
        } else {
        	log("the GroundedBelief is not about a VisualObject... discarding it!");
        }
	}
	
	/**
	 * This method handles GroundedBeliefs about VisualObjects when they are added or overwritten...
	 * The method reads the GB and creates a corresponding coma ontology instance.
	 * This method is called from the processAddedGBelief() and processOverwrittenVisObjGBelief callback methods.
	 * 
	 * If the GroundedBelief is not about a VisualObject, the method returns immediately
	 * and no processing is done.
	 * 
	 * @param gbelief
	 * @throws UnknownSubarchitectureException 
	 * @throws DoesNotExistOnWMException 
	 */
	private void processVisObjGBelief(GroundedBelief gbelief) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> gbProxy = 
			CASTIndependentFormulaDistributionsBelief.create(GroundedBelief.class, gbelief);
		try {
		// check if it's a VisualObject:
        if (!(gbProxy.getType().equals(
                SimpleDiscreteTransferFunction
                        .getBeliefTypeFromCastType(VisualObject.class)))) {
        	log("the GroundedBelief is not about a VisualObject. Aborting!");
        	return;
        }

        // Step 1
        // create the individual for the GroundedBelief

        String objCatName = "dora:" + ComaHelper.firstCap(ComaGBeliefHelper.getGBeliefCategory(gbelief));
        String objInsName = "dora:" + ComaGBeliefHelper.getGBeliefComaIndividualName(gbelief);
        
		synchronized(m_comareasoner) {
			m_comareasoner.addInstance(objInsName, objCatName);
		}
		log("executed addInstance(" + objInsName + ", " + objCatName +")");
		
		// Step 2
		// create the relation with which the individual is spatially located
		// NB the related-to individual is NOT created explicitly (esp. no typing is done!)
		// it is just created implicitly by being in object position of the added relation!
		// this relies on the related-to object to be handled by the same ADD event callback mechanism!
		
		// get the most-likely related-to GroundedBelief
		WMPointer relateePtr = ComaGBeliefHelper.getGBeliefRelatee(gbelief);
		// get the belief it is related to (either another VO or a ComaRoom)
		if (relateePtr!=null) {
			GroundedBelief gbOfRelatedObjectInWM = getMemoryEntry(relateePtr.get().pointer, GroundedBelief.class);
			// get the coma individual name for it
			try {
				String relateeInsName = "dora:" + ComaGBeliefHelper.getGBeliefComaIndividualName(gbOfRelatedObjectInWM);
				String relationName = "dora:" + ComaGBeliefHelper.getGBeliefRelation(gbelief);
				synchronized (m_comareasoner) {
					m_comareasoner.addRelation(objInsName, relationName, relateeInsName);
				}
				log("executed addRelation(" + objInsName + ", " + relationName + ", " + relateeInsName + ")");
			} catch (de.dfki.lt.tr.beliefs.util.BeliefInvalidQueryException e) {
				logException(e);
			}
		}
		} catch (BeliefException e1) {
			logException(e1);
		}
		// this code is deprecated: relation to place is not maintained, and not needed
   		//m_comareasoner.addRelation(objInsName, "dora:observableFromPlace", placeInsName);
   		//log("executed addRelation( " + objInsName + ", dora:observableFromPlace, " + placeInsName +" )");
	}
	
	
	private void processOverwrittenVisObjGBelief(WorkingMemoryChange _wmc) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		// get GroundedBelief from WM
		GroundedBelief gbelief = getMemoryEntry(_wmc.address, GroundedBelief.class);
		log("processAddedGBelief() called: got a callback for an OVERWRITTEN GroundedBelief");
		
        // Step 1
        // create the individual for the GroundedBelief
        String objInsName = "dora:" + ComaGBeliefHelper.getGBeliefComaIndividualName(gbelief);

        // hmmmmmmm, perhaps simply delete the instance, and re-run the addedVisObj method
        if (!objInsName.equals("dora:")) {
        	synchronized (m_comareasoner) {
        		m_comareasoner.deleteInstance(objInsName);
        	}
        	log("deleted instance " + objInsName);
        }
        processVisObjGBelief(gbelief);        
	}
	
	
	
//	private boolean createObject(ObjectPlaceProperty _objProp) {
//		// establish the ontology member names
//		String category = "dora:" + ComaHelper.firstCap(((SpatialProperties.StringValue)_objProp.mapValue).value);
//		String placeIns = "dora:place"+_objProp.placeId;
//		String inRel = "dora:in";
//		String containsRel = "dora:contains";
//		
//		log("createObject of category " + category + " in place " + placeIns + " called.");
//		
//		// check whether the given place already contains an instance of the given category
//		String [] objsInPlace = m_comareasoner.getRelatedInstancesByRelation(placeIns, containsRel);
//		for (String obj : objsInPlace) {
//			if (obj.startsWith(":")) obj = "dora" + obj;
//			if (m_comareasoner.isInstanceOf(obj, category)) {
//				log("object of the given category already exists in the given place - doing nothing else.");
//				return false;
//				// if such an object exists, don't create a new instance!
//			}
//		}
//		
//		String objIns = "dora:object" + m_objectIndexCounter++; 
//		
//		log("going to add new instance " + objIns + " of category " + category);
//		m_comareasoner.addInstance(objIns, category);
//		log("going to add new relation " + objIns + " " + inRel + " " + placeIns);
//		m_comareasoner.addRelation(objIns, inRel, placeIns);
//		return true;
//	}

	private boolean processOverwrittenPlace(WorkingMemoryChange _wmc) {
		boolean _removeFilterAfterwards = false;
		debug("Got a callback for an OVERWRITTEN former Placeholder WME!");
		// try {
			// get the place from WM
			Place _newPlaceNode;
			try {
				_newPlaceNode = getMemoryEntry(_wmc.address, Place.class);
			} catch (DoesNotExistOnWMException e) {
				log("The overwritten Place WME at " + _wmc.address + " ceased to exist. Not taking any further steps. " +
						"Its deletion should be handled by the deletion change filter. Here's the full exception message: " + e.message);
				return true;
			} catch (UnknownSubarchitectureException e) {
				logException(e);
				e.printStackTrace();
				return true;
			}
			log("processOverwrittenPlace() called: Place ID = " + _newPlaceNode.id + " " +
			"Place status = " + (_newPlaceNode.status.equals(PlaceStatus.PLACEHOLDER) ? "PLACEHOLDER" : "TRUEPLACE"));

			// for the moment we are only interested in true places
			if (_newPlaceNode.status.equals(PlaceStatus.TRUEPLACE)) {
				log("create dora:Place instance " + "dora:place"+_newPlaceNode.id);
				synchronized (m_comareasoner) {
					m_comareasoner.addInstance("dora:place"+_newPlaceNode.id, "dora:Place");
				}
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
						try {
							synchronized (m_comareasoner) {
								ConnectivityPathProperty _currPendingPath = getMemoryEntry(_workingMemoryAddress, ConnectivityPathProperty.class);
								log("add relation to coma: " + "dora:place"+_currPendingPath.place1Id + " dora:adjacent " + "dora:place"+_currPendingPath.place2Id);
								m_comareasoner.addRelation("dora:place"+_currPendingPath.place1Id, "dora:adjacent", "dora:place"+_currPendingPath.place2Id);
							}
						} catch (DoesNotExistOnWMException e) {
							log("The ConnectivityPathProperty WME at " + _workingMemoryAddress + " ceased to exist. Continuing to cycle through the pending paths.");
						} catch (UnknownSubarchitectureException e) {
							logException(e);
						}						
					}
					// trigger room creation, splitting, merging, maintenance
					maintainRooms();
				}				
				_removeFilterAfterwards = true;
			}
		return _removeFilterAfterwards;
	}
	
	private void processDeletedPlace(long _deletedPlaceID) {
		synchronized (m_comareasoner) {
			log("processDeletedPlace() called: Got a callback for a DELETED Place with ID: " + _deletedPlaceID);
			boolean _successfullyDeleted = m_comareasoner.deleteInstance("dora:place"+_deletedPlaceID);
			if (_successfullyDeleted) log("successfully deleted " + "dora:place"+_deletedPlaceID);
			else log("There was an error deleting " + "dora:place"+_deletedPlaceID);

			// delete objects associated with that place
			_successfullyDeleted= m_comareasoner.deleteInstance("dora:object"+_deletedPlaceID);
			if (_successfullyDeleted) log("successfully deleted " + "dora:object"+_deletedPlaceID);
			else log("There was an error deleting " + "dora:object"+_deletedPlaceID);

			m_placeholders.remove(_deletedPlaceID);
		}
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
		// determine all places
		Queue<Long> _remainingPlaceIds;
		_remainingPlaceIds = new LinkedList<Long>();
		synchronized(m_comareasoner) {
			String[] _allPlaceIns = m_comareasoner.getAllInstances("dora:Place");
			for (String _placeIns : _allPlaceIns) {
				_remainingPlaceIds.add(Long.valueOf(_placeIns.replaceAll("\\D","")));
			}
			debug("remaining places: " + _remainingPlaceIds);
		}
		Collections.sort((List<Long>)_remainingPlaceIds);

		// get all rooms previously known from WM
		// get initial list for locking them
		List<CASTData<ComaRoom>> _knownRoomsOnWMinit;
		_knownRoomsOnWMinit = new LinkedList<CASTData<ComaRoom>>();
		int _count=0;
		getMemoryEntriesWithData(ComaRoom.class, _knownRoomsOnWMinit, _count);
		for (CASTData<ComaRoom> comaRoomWME : _knownRoomsOnWMinit) {
			try {
				lockEntry(comaRoomWME.getID(), WorkingMemoryPermissions.LOCKEDOD);
				log("successfully LOCKEDOD comaRoomWME "+ comaRoomWME.getID());
			} catch (DoesNotExistOnWMException e) {
				log("could not lock comaRoomWME "+ comaRoomWME.getID() + " -- does not exist on WM");
				logException(e);
				_knownRoomsOnWMinit.remove(comaRoomWME);
			}
		}
		// construct a new list with re-read entries after they were locked
		List<CASTData<ComaRoom>> _knownRoomsOnWM;
		_knownRoomsOnWM = new LinkedList<CASTData<ComaRoom>>();
		for (CASTData<ComaRoom> comaRoomWME : _knownRoomsOnWMinit) {
			try {
				_knownRoomsOnWM.add(getMemoryEntryWithData(comaRoomWME.getID(), ComaRoom.class));
			} catch (DoesNotExistOnWMException e) {
				log("error constructing set of known (and locked) ComaRoom WMEs -- this should never happen " +
						"because the entries were locked, and previously deleted ones were removed from the set...");
				logException(e);
			}
		}
		log("loaded and locked all room WMEs. no. of room WMEs: " + _knownRoomsOnWM.size());

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
				try {
					log("trying to delete the merged ComaRoom with room ID " + comaRoomWME.getData().roomId + "from WMA ID " + comaRoomWME.getID());
					deleteFromWorkingMemory(comaRoomWME.getID());
					log("successfully deleted obsolete ComaRoom with room ID " + comaRoomWME.getData().roomId + "from WMA ID " + comaRoomWME.getID());
				} catch (DoesNotExistOnWMException e) {
					log("could not delete ComaRoom with room ID " + comaRoomWME.getData().roomId + "from WMA ID " + comaRoomWME.getID() + " -- does not exist on WM!");
					logException(e);
				} catch (PermissionException e) {
					log("could not delete ComaRoom with room ID " + comaRoomWME.getData().roomId + "from WMA ID " + comaRoomWME.getID() + " -- permission exception!");
					logException(e);
				}
				synchronized (m_comareasoner) {
					m_comareasoner.deleteInstance("dora:room" + _currentRoomStruct.roomId);
					log("deleted instance dora:room" + _currentRoomStruct.roomId + " from the coma reasoner");
				}
			} else {
				// otherwise -- i.e., if the current room's seed place is not a Doorway 
				log("current room's seed is not a doorway. going to maintain.");
				// check whether that room has changed at all...
				boolean _hasChanged = false;

				Set<Long> _setOfPlaceIDsInTheSameRoom = new HashSet<Long>();
				synchronized (m_comareasoner) {
					// get places in the same room as the seed
					String[] _placesInTheSameRoom = 
							m_comareasoner.getRelatedInstancesByRelation(_seedPlaceInstance,"dora:sameRoomAs");
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
					}
				}

				// add the seed to the list of contained places to be written to WM!
				_setOfPlaceIDsInTheSameRoom.add(_seedPlaceId);
				synchronized (m_comareasoner) {
					m_comareasoner.addRelation(_seedPlaceInstance, "dora:constituentOfRoom", "dora:room" + _currentRoomStruct.roomId);
					log("added seed relation to the coma reasoner: " + _seedPlaceInstance + " dora:constituentOfRoom " + "dora:room" + _currentRoomStruct.roomId);
				}

				// discard the found places from the set of remaining places
				_remainingPlaceIds.removeAll(_setOfPlaceIDsInTheSameRoom);

				// now construct an array to be written to WM
				long[] _arrayOfPlaceIDsInTheSameRoom = new long[_setOfPlaceIDsInTheSameRoom.size()];
				int j=0;
				for (Long _currPlaceID: _setOfPlaceIDsInTheSameRoom) {
					_arrayOfPlaceIDsInTheSameRoom[j]=_currPlaceID.longValue();
					j++;
				}
				log("constructed arrayOfPlaceIDsInTheSameRoom with length = " + _arrayOfPlaceIDsInTheSameRoom.length 
						+": [" + Arrays.toString(_arrayOfPlaceIDsInTheSameRoom) +"]");

				// now check whether that has changed at all, 
				// else we don't have to create WM and binder traffic!
				Set<Long> _oldSetOfContainedPlaces = new HashSet<Long>();
				for (long _currKnownContainedPlaceID : _currentRoomStruct.containedPlaceIds) {
					_oldSetOfContainedPlaces.add(_currKnownContainedPlaceID);
				}
				if (!(_oldSetOfContainedPlaces.equals(_setOfPlaceIDsInTheSameRoom))) {
					_hasChanged = true;
				}
				log("Room has changed: " + (_hasChanged ? "TRUE" : "FALSE"));

				// new code: no longer contained places must no longer have 
				// a constituency relationship with their former rooms
				synchronized (m_comareasoner) {
					HashSet<Long> noLongerConstituentPlaces = ComaHelper.computeSetDifference(_oldSetOfContainedPlaces, _setOfPlaceIDsInTheSameRoom);
					for (Long _nlcPlace : noLongerConstituentPlaces) {
						m_comareasoner.deleteRelation("dora:place" + _nlcPlace,
								"dora:constituentOfRoom", 
								"dora:room" + _currentRoomStruct.roomId);
						log("deleted relation to the coma reasoner: " 
								+ "dora:place" + _nlcPlace 
								+ " dora:constituentOfRoom " 
								+ "dora:room" + _currentRoomStruct.roomId);
					}
				}
				log("after code block that deletes potential no-longer-contained places' constituency assertions.");

				// removed for Dora yr2 and yr3: room categories inferred by Andrzej
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
					log("room has changed! going to overwrite WM!");
					// update the contained places in the room struct
					_currentRoomStruct.containedPlaceIds=_arrayOfPlaceIDsInTheSameRoom;

					// new interface for Dora yr2: remove categories when room has changed
					// so that Andrzej can overwrite it
					_currentRoomStruct.categories=new ProbabilityDistribution();

					// now overwrite the existing room WME
					try {
						overwriteWorkingMemory(comaRoomWME.getID(),_currentRoomStruct);
						logRoom(_currentRoomStruct, "updated room WME. ", comaRoomWME.getID());
					} catch (DoesNotExistOnWMException e) {
						logRoom(_currentRoomStruct, "error overwriting comaRoomWME " + comaRoomWME.getID() + 
								" -- does not exist on WM!", comaRoomWME.getID());
						logException(e);
					} catch (ConsistencyException e) {
						logRoom(_currentRoomStruct, "error overwriting comaRoomWME " + comaRoomWME.getID() + 
								" -- WME inconsistent!", comaRoomWME.getID());
						logException(e);
					} catch (PermissionException e) {
						logRoom(_currentRoomStruct, "error overwriting comaRoomWME " + comaRoomWME.getID() + 
								" -- permission exception!", comaRoomWME.getID());
						logException(e);
					} 
					// 2010-09-13-YR2 maintainRoomProxy(_currentRoomStruct, comaRoomWME.getID());
				} else {
					logRoom(_currentRoomStruct, "did not update room WME or proxy. Room hasn't changed. ", comaRoomWME.getID());
				}
			}
			debug("remaining places: " + _remainingPlaceIds);
		} // end for each room loop
		log("end for each room loop - before each remaining place loop");

		// for each remaining place
		while (!_remainingPlaceIds.isEmpty()) {
			//			for (Long _remainingPlace : _remainingPlaceIds
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
				// ComaRoom _newRoom = new ComaRoom(m_roomIndexCounter++, _currentplaceInstance, new long[0], new String[0], new ProbabilityDistribution()); // uncomment this
				ComaRoom _newRoom = new ComaRoom(m_roomIndexCounter++, _currentplaceInstance, new long[0], new ProbabilityDistribution()); // comment this out

				synchronized (m_comareasoner) {
					// create new room on the reasoner
					m_comareasoner.addInstance("dora:room" + _newRoom.roomId, "dora:PhysicalRoom");
					log("created new instance " + "dora:room" + _newRoom.roomId +  " of concept dora:PhysicalRoom");
					m_comareasoner.addRelation("dora:room" + _newRoom.roomId, "dora:in", "dora:defaultScene");
				}

				// now initialize the room concepts
				// changed for Dora yr2!
				//_newRoom.concepts = m_comareasoner.getAllConcepts("dora:room" + _newRoom.roomId);
				_newRoom.categories = new ProbabilityDistribution();

				Set<Long> _setOfPlaceIDsInTheSameRoom = new HashSet<Long>();
				synchronized (m_comareasoner) {
					String[] _placesInTheSameRoom = 
							m_comareasoner.getRelatedInstancesByRelation(_currentplaceInstance,"dora:sameRoomAs");
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
				}
				// add the seed to the list of contained places to be written to WM!
				_setOfPlaceIDsInTheSameRoom.add(_remainingPlace);
				synchronized (m_comareasoner) {
					m_comareasoner.addRelation("dora:place" + _remainingPlace, "dora:constituentOfRoom", "dora:room" + _newRoom.roomId);
					log("added relation to the coma reasoner: dora:place" + _remainingPlace + " dora:constituentOfRoom " + "dora:room" + _newRoom.roomId);
				}

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
				try {
					addToWorkingMemory(_newRoomWMEID, _newRoom);
				} catch (AlreadyExistsOnWMException e) {
					log("could not add newRoom to WM " + " -- already exists on WM!");
					logException(e);
				}
				logRoom(_newRoom, "added new room WME. ", _newRoomWMEID);
				// 2010-09-13-YR2 maintainRoomProxy(_newRoom, _newRoomWMEID);
			} // end else create a new room for non-doorway seeds
			debug("remaining places: " + _remainingPlaceIds);				
		} // end for each remaining place loop
		log("end for each remaining place loop");
		for (CASTData<ComaRoom> comaRoomWME : _knownRoomsOnWM) {
			if (existsOnWorkingMemory(comaRoomWME.getID())) {
				try {
					log("trying to unlock ComaRoomWME " + comaRoomWME.getID());
					unlockEntry(comaRoomWME.getID());
					log("successfully unlocked ComaRoomWME " + comaRoomWME.getID());
				} catch (ConsistencyException e) {
					log("could not unlock ComaRoomWME " + comaRoomWME.getID() + " -- consistency exception!");
					logException(e);
				} catch (DoesNotExistOnWMException e) {
					log("could not unlock ComaRoomWME " + comaRoomWME.getID() + " -- does not exist on WM!");
					logException(e);
				}
			} else {
				log("ComaRoomWME " + comaRoomWME.getID() + " ceased to exist -- not attempting to unlock.");
			}
		}
		log("unlocked all ComaRoom WMEs.");
		logInstances("owl:Thing");
		
		if(this.mapLoadMode == true && this.mapLoadRoomsWritten==false) {
			log("Done with room maintenance while mapLoadMode == true && mapLoadRoomsWritten == false; going to report MapLoadStatus.roomsWritten=true");
		  	try {
				// We can mark the Coma rooms as
			  	// loaded now as well
			  	lockEntry(m_mapLoadStatusAddress,
				    WorkingMemoryPermissions.LOCKEDODR);
				MapLoadStatus _mlstatus = 
				  getMemoryEntry(m_mapLoadStatusAddress, MapLoadStatus.class);
				_mlstatus.roomsWritten = true;
				overwriteWorkingMemory(m_mapLoadStatusAddress, _mlstatus);
				log("successfully OVERWRITTEN WM with MapLoadStatus.roomsWritten=true");
				this.mapLoadRoomsWritten=true;
			}
			catch (DoesNotExistOnWMException e) {
				logException(e);
				e.printStackTrace();
			}
			catch (UnknownSubarchitectureException e) {
				logException(e);
				e.printStackTrace();
			}
			catch (ConsistencyException e) {
				logException(e);
				e.printStackTrace();
			}
			catch (PermissionException e) {
				logException(e);
				e.printStackTrace();
			}
		}
	}
	
	
	
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


	private void processAddedObjectProperty(WorkingMemoryChange _wmc) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		log("processAddedObjectProperty() called.");
		// get object from WM
		ObjectPlaceProperty _objProp = getMemoryEntry(_wmc.address, ObjectPlaceProperty.class);
		debug("got a callback for an ADDED ObjectPlaceProperty for " + 
				_objProp.category + " with relation "+ _objProp.relation.toString() + " to support object " +
				_objProp.supportObjectId + " of category " + _objProp.supportObjectCategory +
				" observed from a place with ID " + _objProp.placeId + 
				" with a probability of ");
		
		
		if (_objProp.inferred) {
			debug("the added ObjectPlaceProperty was inferred from conceptual and not observed by AVS. Doing nothing.");
			return;
		}
		
		DiscreteProbabilityDistribution dpd = (DiscreteProbabilityDistribution) _objProp.distribution;
		if (!((((BinaryValue) dpd.data[0].value).value == true && dpd.data[0].probability > 0.5)
				|| (((BinaryValue) dpd.data[1].value).value == true && dpd.data[1].probability > 0.5)))  {
			debug("The ObjectPlaceProperty object observation has a probability <= 0.5. Doing nothing.");
			return;
		}
		
		log("ready to add an object to the coma KB.");
		// TODO add object to coma KB!!!
		
		// _objProp.category; // cat of the obj
		// _objProp.distribution; // 
		// _objProp.inferred; // must be false, otherwise it is inferred by conceptual
		// _objProp.mapValue; // ignore max a posteriori
		// _objProp.mapValueReliable; // ignore max a posteriori
		// _objProp.placeId; // place from which it was seen
		// _objProp.relation; // enum := ON, INOBJECT, INROOM
		// _objProp.supportObjectCategory; // category of the supporting/related object or "" if INROOM
		// _objProp.supportObjectId; // unique id of the supporting object
		
		String objInsName = "dora:" + _objProp.category.toLowerCase() + _wmc.address.id.replace(":", "_");
		String objCatName = "dora:" + ComaHelper.firstCap(_objProp.category);
		
		String placeInsName = "dora:place" + _objProp.placeId;
		
		synchronized (m_comareasoner) {
			m_comareasoner.addInstance(objInsName, objCatName);
			log("executed addInstance( " + objInsName + ", " + objCatName +" )");
			m_comareasoner.addRelation(objInsName, "dora:observableFromPlace", placeInsName);
			log("executed addRelation( " + objInsName + ", dora:observableFromPlace, " + placeInsName +" )");

			// check if the object is immediately in the room or related via a supportObject
			if ((!_objProp.supportObjectCategory.equals("")) && 
					(_objProp.relation.equals(SpatialRelation.ON) || _objProp.relation.equals(SpatialRelation.INOBJECT))) {
				String suppobjInsName = "dora:" + _objProp.supportObjectCategory.toLowerCase() + _objProp.supportObjectId;
				String suppobjCatName = "dora:" + ComaHelper.firstCap(_objProp.supportObjectCategory);

				m_comareasoner.addInstance(suppobjInsName, suppobjCatName);
				m_comareasoner.addRelation(suppobjInsName, "dora:observableFromPlace", placeInsName);
				m_comareasoner.addRelation(
						objInsName, 
						(_objProp.relation.equals(SpatialRelation.ON) ? "dora:on" : "dora:in"), 
						suppobjInsName);
			}	
		}
	}

}
