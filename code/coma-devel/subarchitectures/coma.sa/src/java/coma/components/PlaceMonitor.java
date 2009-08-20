package coma.components;

import java.util.Map;

import comadata.ComaReasonerInterfacePrx;

import Ice.Identity;
import Ice.ObjectPrx;
import NavData.AEdge;
import NavData.FNode;
import SpatialData.Place;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

public class PlaceMonitor extends ManagedComponent {

	Identity m_reasoner_id;
	ComaReasonerInterfacePrx m_comareasoner;

	public void configure(Map<String, String> args) {
		log("configure() called");
		m_reasoner_id = new Identity();
		m_reasoner_id.name="";
		m_reasoner_id.category="ComaReasoner";

		if (args.containsKey("--reasoner-name")) {
			m_reasoner_id.name=args.get("--reasoner-name");
		}
	}

	public void start() {
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(FNode.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				log("Got a callback for an ADDed FNode!");
				try {
					FNode _newPlaceNode = getMemoryEntry(_wmc.address, FNode.class);
					log("FNode ID (node ID) = " + _newPlaceNode.nodeId);
					log("FNode status (gateway) = " + _newPlaceNode.gateway);
					m_comareasoner.addInstance("test:place"+_newPlaceNode.nodeId, "test:Place");
					log("Added new Place instance. This is a list of all instances in the ABox");
					for (String _currIns : m_comareasoner.getAllInstances("owl:Thing")) {
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
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				log("Got a callback for an ADDed Place WME!");
				try {
					Place _newPlaceNode = getMemoryEntry(_wmc.address, Place.class);
					log("Place ID = " + _newPlaceNode.id);
					log("Doing nothing else...");
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
}