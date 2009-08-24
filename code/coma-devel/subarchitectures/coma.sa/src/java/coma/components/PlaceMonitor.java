package coma.components;

import java.util.HashMap;
import java.util.Map;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Proxy;
import binder.autogen.featvalues.StringValue;
import binder.utils.ProbabilityDistributionUtils;

import comadata.ComaReasonerInterfacePrx;

import Ice.Identity;
import Ice.ObjectPrx;
import NavData.AEdge;
import NavData.FNode;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import cast.AlreadyExistsOnWMException;
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
	String m_bindingSA;
	//Map<String, String> m_placeID2proxyWMID;
	Map<Long,ComaPlace> m_placeID2ComaPlace;
	

	public void configure(Map<String, String> args) {
		log("configure() called");
		m_reasoner_id = new Identity();
		m_reasoner_id.name="";
		m_reasoner_id.category="ComaReasoner";

		if (args.containsKey("--reasoner-name")) {
			m_reasoner_id.name=args.get("--reasoner-name");
		}
		
		if (args.containsKey("-bsa")) {
			m_bindingSA=args.get("-bsa");
		} else if (args.containsKey("--binding-sa")) {
			m_bindingSA=args.get("--binding-sa");
		} else if (args.containsKey("--bsa")) {
			m_bindingSA=args.get("--bsa");
		}
		
		if (m_bindingSA!=null) {
//			m_placeID2proxyWMID = new HashMap<String, String>();
			m_placeID2ComaPlace = new HashMap<Long, ComaPlace>();
		}
		
	}
	
	
	public void start() {
		
		// register the monitoring change filters
		
		// this is the "proper" Place monitor
		// for the time being, it does not create any coma instances
		// instead the FNode monitor does this task
		// once the interface changes, the relevant code from there has to go in here!
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				log("Got a callback for an ADDed Place WME!");
				try {
					Place _newPlaceNode = getMemoryEntry(_wmc.address, Place.class);
					log("Place ID = " + _newPlaceNode.id);
					maintainPlaceProxy(_newPlaceNode);
//					log("Doing nothing else...");
					// propagate this to the coma ontology and the binder when possible!
					
					// old code:
//					m_comareasoner.addInstance("test:place"+_newPlaceNode.nodeId, "test:Place");
//					log("Added new Place instance. This is a list of all instances in the ABox");
//					for (String _currIns : m_comareasoner.getAllInstances("owl:Thing")) {
//						log("instance: " + _currIns);
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

		
		// this is the "proper" Place monitor
		// for the time being, it does not create any coma instances
		// instead the FNode monitor does this task
		// once the interface changes, the relevant code from there has to go in here!
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class, WorkingMemoryOperation.OVERWRITE), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				log("Got a callback for an OVERWRITTEN Place WME!");
				try {
					Place _placeNode = getMemoryEntry(_wmc.address, Place.class);
					log("Place ID = " + _placeNode.id);
					maintainPlaceProxy(_placeNode);
//					log("Doing nothing else...");
					// propagate this to the coma ontology and the binder when possible!
					
					// old code:
//					m_comareasoner.addInstance("test:place"+_newPlaceNode.nodeId, "test:Place");
//					log("Added new Place instance. This is a list of all instances in the ABox");
//					for (String _currIns : m_comareasoner.getAllInstances("owl:Thing")) {
//						log("instance: " + _currIns);
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
		
		// this is the "hacky" FNode monitor, 
		// which is used to get access to Place properties right away.
		// once the Place WMEs contain all relevant information, copy this code to the correct monitor above
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(FNode.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				log("Got a callback for an ADDed FNode!");
				try {
					FNode _newPlaceNode = getMemoryEntry(_wmc.address, FNode.class);
					log("FNode ID (node ID) = " + _newPlaceNode.nodeId);
					log("FNode status (gateway) = " + _newPlaceNode.gateway);
					
					// propagate this to the coma ontology and to the binder 
					// 1st coma
					m_comareasoner.addInstance("test:place"+_newPlaceNode.nodeId, "test:Place");
					log("Added new Place instance."); // This is a list of all instances in the ABox");
//					for (String _currIns : m_comareasoner.getAllInstances("owl:Thing")) {
//						log("instance: " + _currIns);
//					}
					// 2nd binder
//					boolean _isGateway = (_newPlaceNode.gateway==0 ? false: true);
//					createNewPlaceProxy(_newPlaceNode.nodeId, _isGateway);
					maintainPlaceProxy(_newPlaceNode);
					
				} catch (DoesNotExistOnWMException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (UnknownSubarchitectureException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			};
		});
		
		// this is the "hacky" FNode monitor, 
		// which is used to get access to Place properties right away.
		// once the Place WMEs contain all relevant information, copy this code to the correct monitor above
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(FNode.class, WorkingMemoryOperation.OVERWRITE), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				log("Got a callback for an OVERWRITTEN FNode!");
				try {
					FNode _fNode = getMemoryEntry(_wmc.address, FNode.class);
					log("FNode ID (node ID) = " + _fNode.nodeId);
					log("FNode status (gateway) = " + _fNode.gateway);
					
					// propagate this to the coma ontology and to the binder 
					// 1st coma
					m_comareasoner.addInstance("test:place"+_fNode.nodeId, "test:Place");
					log("Added new Place instance."); // This is a list of all instances in the ABox");
//					for (String _currIns : m_comareasoner.getAllInstances("owl:Thing")) {
//						log("instance: " + _currIns);
//					}
					// 2nd binder
//					boolean _isGateway = (_newPlaceNode.gateway==0 ? false: true);
//					createNewPlaceProxy(_newPlaceNode.nodeId, _isGateway);
					maintainPlaceProxy(_fNode);
					
				} catch (DoesNotExistOnWMException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (UnknownSubarchitectureException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			};
		});

		// temporaray ("hacky") monitor for Place connectivity
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(AEdge.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				log("Got a callback for an ADDed AEdge!");
				try {
					AEdge _newEdge = getMemoryEntry(_wmc.address, AEdge.class);
					log("Start Node ID = " + _newEdge.startNodeId);
					log("End Node ID = " + _newEdge.endNodeId);
					m_comareasoner.addRelation("test:place"+_newEdge.startNodeId, "test:adjacent", "test:place"+_newEdge.endNodeId);
					log("Added new Edge relation to coma ontology."); // This is a list of all room instances in the ABox");
//					for (String _currIns : m_comareasoner.getAllInstances("test:PhysicalRoom")) {
//						log("instance: " + _currIns);
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
		
		log("Initiating connection to server " + m_reasoner_id.category + "::" + m_reasoner_id.name + "...");
		ObjectPrx base = getObjectAdapter().createProxy(m_reasoner_id);
		m_comareasoner = comadata.ComaReasonerInterfacePrxHelper.checkedCast(base);
	}
	
	
	
	private void maintainPlaceProxy(Place _placeNode) {
		log("maintaining a Place proxy for a Place struct.");
		ComaPlace _comaPlace;
		if (!(m_placeID2ComaPlace.containsKey(_placeNode.id))) {
			_comaPlace = new ComaPlace(_placeNode.id);
		} else {
			_comaPlace = m_placeID2ComaPlace.get(_placeNode.id);
		}
		_comaPlace.setPlaceStatus(_placeNode.status);
		
		m_placeID2ComaPlace.put(_placeNode.id, _comaPlace);

		if (_comaPlace.m_proxyWMid==null) {
			log("no proxy WMA known...");
			createNewPlaceProxy(_comaPlace);
		} else {
			log("proxy WMA known!");
			updatePlaceProxy(_comaPlace);
		}
	}

	private void maintainPlaceProxy(FNode _placeNode) {
		log("maintaining a Place proxy for an FNode struct.");
		ComaPlace _comaPlace;
		if (!(m_placeID2ComaPlace.containsKey(_placeNode.nodeId))) {
			_comaPlace = new ComaPlace(_placeNode.nodeId);
		} else {
			_comaPlace = m_placeID2ComaPlace.get(_placeNode.nodeId);
		}
		_comaPlace.setGatewayStatus(_placeNode.gateway);

		m_placeID2ComaPlace.put(_placeNode.nodeId, _comaPlace);

		if (_comaPlace.m_proxyWMid==null) {
			log("no proxy WMA known...");
			createNewPlaceProxy(_comaPlace);
		} else {
			log("proxy WMA known!");
			updatePlaceProxy(_comaPlace);
		}
	}
	
	/**
	 * 
	 * @param _comaPlace
	 */
	public void createNewPlaceProxy(ComaPlace _comaPlace) {
		// check for presence of binder
		if (m_bindingSA==null) {
			log("No binder specified. Not creating any proxy...");
			return;
		}
		log("creating a new place proxy.");
		
		// create new proxy
		Proxy _newPlaceProxy = new Proxy();
		_newPlaceProxy.entityID = newDataID();
		_newPlaceProxy.subarchId = this.getSubarchitectureID();
		_newPlaceProxy.probExists = 1;
			
		_newPlaceProxy.features = new Feature[3];
		
		_newPlaceProxy.features[0] = new Feature();
		_newPlaceProxy.features[0].featlabel = "place_id";
		_newPlaceProxy.features[0].alternativeValues = new FeatureValue[1];
		_newPlaceProxy.features[0].alternativeValues[0] = new StringValue(1,_comaPlace.m_id.toString());
		
		_newPlaceProxy.features[1] = new Feature();
		_newPlaceProxy.features[1].featlabel = "place_type";
		_newPlaceProxy.features[1].alternativeValues = new FeatureValue[1];
		_newPlaceProxy.features[1].alternativeValues[0] = new StringValue(1, 
				(_comaPlace.getPlaceStatus()==PlaceStatus.TRUEPLACE ? "Place" : "Placeholder"));
			
		_newPlaceProxy.features[2] = new Feature();
		_newPlaceProxy.features[2].featlabel = "node_type";
		_newPlaceProxy.features[2].alternativeValues = new FeatureValue[1];
		_newPlaceProxy.features[2].alternativeValues[0] = new StringValue(1, 
				(_comaPlace.getGatewayStatus()==1 ? "Doorway" : (_comaPlace.getGatewayStatus()==-1 ? "unknown" :  "Free Node"))); 

		_newPlaceProxy.distribution = ProbabilityDistributionUtils.generateProbabilityDistribution(_newPlaceProxy);

		try {
//			addToWorkingMemory(new WorkingMemoryAddress(m_bindingSA,_newPlaceProxy.entityID), _newPlaceProxy);
			addToWorkingMemory(new WorkingMemoryAddress(_newPlaceProxy.entityID, m_bindingSA), _newPlaceProxy);
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.exit(0);
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.exit(0);
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.exit(0);
		}
		_comaPlace.setProxyWMid(_newPlaceProxy.entityID);
		m_placeID2ComaPlace.put(_comaPlace.m_id, _comaPlace);
//		m_placeID2proxyWMID.put(String.valueOf(_placeID), _newPlaceProxy.entityID);
		log("Successfully added a place proxy to the binder.");
		
	}
	
	public void updatePlaceProxy(ComaPlace _comaPlace) {
		// check for presence of binder
		if (m_bindingSA==null) {
			log("No binder specified. Not updating any proxy...");
			return;
		}
		log("updating an existing place proxy.");

		try {
			WorkingMemoryAddress _proxyWMA = new WorkingMemoryAddress(_comaPlace.getProxyWMid(), m_bindingSA);
			Proxy _placeProxy = getMemoryEntry(_proxyWMA, Proxy.class);
			
			for (int i = 0; i < _placeProxy.features.length; i++) {
				if (_placeProxy.features[i].featlabel.equals("place_type")) {
					_placeProxy.features[i].alternativeValues = new FeatureValue[1];
					_placeProxy.features[i].alternativeValues[0] = new StringValue(1, 
							(_comaPlace.getPlaceStatus()==PlaceStatus.TRUEPLACE ? "Place" : "Placeholder"));
					log("Feature: " + _placeProxy.features[i].featlabel + 
							" -- place status:" + (_comaPlace.getPlaceStatus()==PlaceStatus.TRUEPLACE ? "Place" : "Placeholder"));
				} else if (_placeProxy.features[i].featlabel.equals("node_type")) {
					_placeProxy.features[i].alternativeValues = new FeatureValue[1];
					_placeProxy.features[i].alternativeValues[0] = new StringValue(1, 
							(_comaPlace.getGatewayStatus()==1 ? "Doorway" : (_comaPlace.getGatewayStatus()==-1 ? "unknown" :  "Free Node")));
					log("Feature: " + _placeProxy.features[i].featlabel + 
							" -- gateway status:" + (_comaPlace.getGatewayStatus()==1 ? "Doorway" : (_comaPlace.getGatewayStatus()==-1 ? "unknown" :  "Free Node")));							
				} else if (_placeProxy.features[i].featlabel.equals("place_id")) {
					log("Place ID of the overwritten proxy: " + _placeProxy.features[i].featlabel);
				}
			}
			_placeProxy.distribution = ProbabilityDistributionUtils.generateProbabilityDistribution(_placeProxy);
			
			overwriteWorkingMemory(_proxyWMA, _placeProxy);
			
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (ConsistencyException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (PermissionException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		log("Successfully updated a place proxy on the binder.");
	}
	
	private class ComaPlace {
		private PlaceStatus m_placeStatus;
		private Long m_id;
		private short m_gateway;
		private String m_proxyWMid;
		
		public ComaPlace(Long _placeId) {
			m_id = _placeId;
			m_gateway = -1;
		}
		
		public boolean equals(ComaPlace _comaPlace) {
			return this.m_id.equals(_comaPlace.m_id);
		}
		
		public PlaceStatus getPlaceStatus() {
			return m_placeStatus;
		}
		public void setPlaceStatus(PlaceStatus _placeStatus) {
			m_placeStatus = _placeStatus;
		}
		
		public Long getPlaceId() {
			return m_id;
		}
		
		public short getGatewayStatus() {
			return m_gateway;
		}
		public void setGatewayStatus(short _gatewayStatus) {
			m_gateway = _gatewayStatus;
		}
		
		public String getProxyWMid() {
			return m_proxyWMid;
		}
		public void setProxyWMid(String _proxyWMid) {
			m_proxyWMid = _proxyWMid;
		}
	}
	
	
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