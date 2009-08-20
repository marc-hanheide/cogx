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
import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
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
	Map<String, String> m_placeID2proxyWMID;
	

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
			m_placeID2proxyWMID = new HashMap<String, String>();
		}
		
	}

	public void start() {
		
		// register the monitoring change filters
		
		// this is the "proper" Place monitor
		// for the time being, it does not create any behavior
		// instead the FNode monitor does this task
		// once the interface changes, the relevant code from there has to go in here!
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				log("Got a callback for an ADDed Place WME!");
				try {
					Place _newPlaceNode = getMemoryEntry(_wmc.address, Place.class);
					log("Place ID = " + _newPlaceNode.id);
					log("Doing nothing else...");
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
					log("Added new Place instance. This is a list of all instances in the ABox");
					for (String _currIns : m_comareasoner.getAllInstances("owl:Thing")) {
						log("instance: " + _currIns);
					}
					// 2nd binder
					boolean _isGateway = (_newPlaceNode.gateway==0 ? false: true);
					createNewPlaceProxy(_newPlaceNode.nodeId, _isGateway);
					
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
					log("Added new Edge relation. This is a list of all room instances in the ABox");
					for (String _currIns : m_comareasoner.getAllInstances("test:PhysicalRoom")) {
						log("instance: " + _currIns);
					}
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
	
	/**
	 * @todo THIS DOES NOT SUPPORT PLACEHOLDERS YET!
	 * 
	 * @param _placeID
	 * @param _isGateway
	 */
	public void createNewPlaceProxy(long _placeID, boolean _isGateway) {
		// check for presence of binder
		if (m_bindingSA==null) {
			log("No binder specified. Not creating any proxy...");
			return;
		}
		
		log("Warning: Placeholders not supported yet!");
		// create new proxy
		Proxy _newPlaceProxy = new Proxy();
		_newPlaceProxy.entityID = newDataID();
		_newPlaceProxy.subarchId = this.getSubarchitectureID();
		_newPlaceProxy.probExists = 1;
			
		_newPlaceProxy.features = new Feature[2];
		
		_newPlaceProxy.features[0] = new Feature();
		_newPlaceProxy.features[0].featlabel = "place_id";
		_newPlaceProxy.features[0].alternativeValues = new FeatureValue[1];
		_newPlaceProxy.features[0].alternativeValues[0] = new StringValue(1,String.valueOf(_placeID));
		
		_newPlaceProxy.features[1] = new Feature();
		_newPlaceProxy.features[1].featlabel = "place_type";
		_newPlaceProxy.features[1].alternativeValues = new FeatureValue[1];
		_newPlaceProxy.features[1].alternativeValues[0] = new StringValue(1, (_isGateway ? "Doorway" : "Place"));
			
		_newPlaceProxy.distribution = ProbabilityDistributionUtils.generateProbabilityDistribution(_newPlaceProxy);

		try {
			addToWorkingMemory(new WorkingMemoryAddress(m_bindingSA,_newPlaceProxy.entityID), _newPlaceProxy);
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
		m_placeID2proxyWMID.put(String.valueOf(_placeID), _newPlaceProxy.entityID);
		log("Successfully added a place proxy to the binder.");
		
	}
	
}