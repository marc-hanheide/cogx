package coma.components;

import java.util.HashMap;
import java.util.Properties;

import NavData.AEdge;
import NavData.Area;
import NavData.FNode;
import NavData.ONode;
import NavData.RobotPose;
import NavData.TopologicalRobotPos;
import NavData.VEdge;

import org.cognitivesystems.reasoner.base.OntologyMemberFactory;
import org.cognitivesystems.reasoner.base.ReasonerInstance;
import org.cognitivesystems.reasoner.base.ReasonerRelation;

import coma.aux.ComaFunctionWriter;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.core.data.CASTData;

public class ComaNavgraphMonitor extends PrivilegedManagedProcess {

	private ComaFunctionWriter m_funcWriter;
	private HashMap<WorkingMemoryAddress, Integer> m_wma2nodeID;
	private HashMap<WorkingMemoryAddress, Integer> m_wma2areaID;
	private HashMap<WorkingMemoryAddress, ReasonerRelation> m_wma2rel;
	private String m_ontologyNamespace;
	private String m_ontologySep;
	private OntologyMemberFactory m_oeMemberMaker;

	public ComaNavgraphMonitor(String _id) {
		super(_id);
        m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
        m_wma2nodeID = new HashMap<WorkingMemoryAddress, Integer>();
        m_wma2areaID = new HashMap<WorkingMemoryAddress, Integer>();
        m_wma2rel = new HashMap<WorkingMemoryAddress, ReasonerRelation>();
	}

    public void configure(Properties _config) {
    	super.configure(_config);
        
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
		this.m_oeMemberMaker = new OntologyMemberFactory(m_ontologyNamespace, m_ontologySep);
		m_funcWriter = new ComaFunctionWriter(this);	
    }
    	
    public void start() {
        super.start();
        log("Starting ComaNavgraphMonitor...");

        try {
        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(TopologicalRobotPos.class),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a general TopologicalRobotPos callback!");
                    }
                });
        	
        	
        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Area.class, 
        			WorkingMemoryOperation.ADD),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a new Area");
                    	processNewArea(_wmc);
                    }
                });
        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Area.class, 
        			WorkingMemoryOperation.DELETE),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a deleted Area");
                    	processDeletedArea(_wmc);
                    }
                });
        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Area.class, 
        			WorkingMemoryOperation.OVERWRITE),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a changed Area");
                    	processChangedArea(_wmc);
                    }
                });
  
        	
        	
        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(FNode.class, 
        			WorkingMemoryOperation.ADD),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a new FNode");
                    	processNewFNode(_wmc);
                    }
                });
        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(FNode.class, 
        			WorkingMemoryOperation.DELETE),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a deleted FNode");
                    	processDeletedFNode(_wmc);
                    }
                });
        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(FNode.class, 
        			WorkingMemoryOperation.OVERWRITE),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a changed FNode");
                    	processChangedFNode(_wmc);
                    }
                });
        	

        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(ONode.class, 
        			WorkingMemoryOperation.ADD),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a new ONode");
                    	processNewONode(_wmc);
                    }
                });
        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(ONode.class, 
        			WorkingMemoryOperation.DELETE),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a deleted ONode");
                    	processDeletedONode(_wmc);

                    }
                });
        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(ONode.class, 
        			WorkingMemoryOperation.OVERWRITE),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a changed ONode");
                    	processChangedONode(_wmc);
                    }
                });
        	
        	
        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(AEdge.class, 
        			WorkingMemoryOperation.ADD),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a new AEdge");
                    	processNewAEdge(_wmc);
                    }
                });
        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(AEdge.class, 
        			WorkingMemoryOperation.DELETE),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a deleted AEdge");
                    	processDeletedAEdge(_wmc);
                    }
                });
        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(AEdge.class, 
        			WorkingMemoryOperation.OVERWRITE),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a changed AEdge");
                    	processChangedAEdge(_wmc);
                    }
                });
        	
        	
        	
        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(VEdge.class, 
        			WorkingMemoryOperation.ADD),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a new VEdge");
                    	processNewVEdge(_wmc);
                    }
                });
        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(VEdge.class, 
        			WorkingMemoryOperation.DELETE),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a deleted VEdge");
                    	processDeletedVEdge(_wmc);
                    }
                });
        	addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(VEdge.class, 
        			WorkingMemoryOperation.OVERWRITE),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("I got a changed VEdge");
                    	processChangedVEdge(_wmc);
                    }
                });
        }
        catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException("Error! Could not register change filter. Aborting!");
        }
        log("done registering change filters for nav graph units.");
    }

	
    
    
	private void processNewArea(WorkingMemoryChange _wmc) {
        CASTData<?> wme;
		try {
			wme = getWorkingMemoryEntry(_wmc.m_address);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
		Area _currArea = (Area) wme.getData();

        log("Area: " );
        log("area ID " + _currArea.m_id);
        
        log("creating a new Area instance, and storing wma2areaID mapping -- what else?");
        m_funcWriter.addInstanceConceptAssertion(
        		new WorkingMemoryChangeReceiver() {
				  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				  		handleComaReasonerFunctionDone(_wmc);} }, 
        		m_oeMemberMaker.createInstance("area"+_currArea.m_id), 
        		m_oeMemberMaker.createConcept("Area"));

        m_wma2areaID.put(_wmc.m_address, _currArea.m_id);
	}
    
    
	private void processChangedArea(WorkingMemoryChange _wmc) {
        CASTData<?> wme;
		try {
			wme = getWorkingMemoryEntry(_wmc.m_address);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
		Area _currArea = (Area) wme.getData();

        log("Area: area ID " + _currArea.m_id);
        
        log("What am I supposed to do with a changed area?");
	}
	
	private void processDeletedArea(WorkingMemoryChange _wmc) {
        int _areaID = m_wma2areaID.remove(_wmc.m_address);
        
        log("deleting this Area instance -- what else?");
        m_funcWriter.deleteInstance(
        		new WorkingMemoryChangeReceiver() {
        			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
        				handleComaReasonerFunctionDone(_wmc);} },  
        		m_oeMemberMaker.createInstance("area"+_areaID));
        log("NEED TO DO SOMETHING MORE!!!! A deleted area means that some area merging was going on!");
        log("maybe this could be handled by the changed FNode callbacks...");
	}

    
    
	private void processNewFNode(WorkingMemoryChange _wmc) {
        CASTData<?> wme;
		try {
			wme = getWorkingMemoryEntry(_wmc.m_address);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
        FNode _currNode = (FNode) wme.getData();

        log("Fnode: node ID " + _currNode.m_nodeID + " - area ID " + _currNode.m_areaID+ 
        " - area type " + _currNode.m_areaType + " - gateway " + _currNode.m_gateway);
        
        log("creating a new FNode instance, and storing wma2nodeID mapping.");
        m_funcWriter.addInstanceConceptAssertion(
        		new WorkingMemoryChangeReceiver() {
				  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				  		handleComaReasonerFunctionDone(_wmc);} },  
        		m_oeMemberMaker.createInstance("node"+_currNode.m_nodeID), 
        		m_oeMemberMaker.createConcept((_currNode.m_gateway==1 ? "GatewayNode" : "PlaceNode")));

        log("creating a relation between the FNode and the Area it belongs to -- what else?");
        ReasonerInstance _nodeIns = m_oeMemberMaker.createInstance("node"+_currNode.m_nodeID);
        ReasonerInstance _areaIns = m_oeMemberMaker.createInstance("area"+_currNode.m_areaID);
        m_funcWriter.addInstanceInstanceRelation(
        		new WorkingMemoryChangeReceiver() {
				  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				  		handleComaReasonerFunctionDone(_wmc);} },  
        		_nodeIns, 
        		_areaIns, 
        		m_oeMemberMaker.createRelation("topoIncluded", _nodeIns, _areaIns));
        
        m_wma2nodeID.put(_wmc.m_address, _currNode.m_nodeID);
        
	}
    
    
	private void processChangedFNode(WorkingMemoryChange _wmc) {
        CASTData<?> wme;
		try {
			wme = getWorkingMemoryEntry(_wmc.m_address);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
        FNode _currNode = (FNode) wme.getData();

        log("Fnode: node ID " + _currNode.m_nodeID + " - area ID " + _currNode.m_areaID+ 
                " - area type " + _currNode.m_areaType + " - gateway " + _currNode.m_gateway);
        
        log("What am I supposed to do with a changed node?");
        // under the following conditions a node may change (25 June 2008, cf mail by Patric:)
        // * when a node is turned into a door (position change)
        // * when a door is removed (nodes change area id)
        
        // e.g. specify rdf:type = FNode and delete sub-type assertions, then re-specify sub-type
        // e.g. check area ID membership and react accordingly
        // e.g. check arey type and react!
	}
	
	private void processDeletedFNode(WorkingMemoryChange _wmc) {
        int _nodeID = m_wma2nodeID.remove(_wmc.m_address);
        
        log("deleting this FNode instance -- what else?");
        m_funcWriter.deleteInstance(
        		new WorkingMemoryChangeReceiver() {
				  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				  		handleComaReasonerFunctionDone(_wmc);} },  
        		m_oeMemberMaker.createInstance("node"+_nodeID));
        throw new RuntimeException("FNodes were supposed to not be deleted! So this breaks several assumptions. Check your code!");
        }


	
	
	private void processNewONode(WorkingMemoryChange _wmc) {
        CASTData<?> wme;
		try {
			wme = getWorkingMemoryEntry(_wmc.m_address);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
        ONode _currNode = (ONode) wme.getData();

        log("ONode: node ID " + _currNode.m_nodeID + 
        " - area ID " + _currNode.m_areaID+ 
        " - area type " + _currNode.m_areaType+
        " - category " + _currNode.m_category+
        " - object ID " + _currNode.m_objectID);
        
        log("storing wma2nodeID mapping -- what else?");
        log("not representing the object in coma - yet!!!");
        // put object into coma!!!
//        m_funcWriter.addInstanceConceptAssertion(null, 
//        		OntologyMemberFactory.createInstance("", "", "node"+_currNode.m_nodeID), 
//        		OntologyMemberFactory.createConcept("", "", (_currNode.m_gateway==1 ? "GatewayNode" : "PlaceNode")));

        m_wma2nodeID.put(_wmc.m_address, _currNode.m_nodeID);
        
	}
    
    
	private void processChangedONode(WorkingMemoryChange _wmc) {
        CASTData<?> wme;
		try {
			wme = getWorkingMemoryEntry(_wmc.m_address);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
        ONode _currNode = (ONode) wme.getData();

        log("ONode: node ID " + _currNode.m_nodeID + 
                " - area ID " + _currNode.m_areaID+ 
                " - area type " + _currNode.m_areaType+
                " - category " + _currNode.m_category+
                " - object ID " + _currNode.m_objectID);
        
        log("What am I supposed to do with a changed node?");
        // depends on how objects themselves are represented!
	}
	
	private void processDeletedONode(WorkingMemoryChange _wmc) {
        int _nodeID = m_wma2nodeID.remove(_wmc.m_address);
        
        log("nav.sa has deleted node #"+_nodeID);
        log("not doing anything with this ONode instance.... what am I supposed to do?");
//        m_funcWriter.deleteInstance(null, 
//        		OntologyMemberFactory.createInstance("", "", "node"+_nodeID));
	}

	
	
	
	
	private void processNewAEdge(WorkingMemoryChange _wmc) {
        CASTData<?> wme;
		try {
			wme = getWorkingMemoryEntry(_wmc.m_address);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
        AEdge _currEdge = (AEdge) wme.getData();

        log("AEdge: start node ID" + _currEdge.m_startNodeID + " - end node ID " + _currEdge.m_endNodeID);
        
        log("creating a new relation");
        ReasonerInstance _ins1 = m_oeMemberMaker.createInstance("node"+_currEdge.m_startNodeID);
        ReasonerInstance _ins2 = m_oeMemberMaker.createInstance("node"+_currEdge.m_endNodeID); 
        ReasonerRelation _rel = m_oeMemberMaker.createRelation("accessibleFrom", _ins1, _ins2); 
        m_funcWriter.addInstanceInstanceRelation(
        		new WorkingMemoryChangeReceiver() {
				  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				  		handleComaReasonerFunctionDone(_wmc);} },
				  		_ins1, _ins2, _rel);

        m_wma2rel.put(_wmc.m_address, _rel);
	}
    
    
	private void processChangedAEdge(WorkingMemoryChange _wmc) {
        CASTData<?> wme;
		try {
			wme = getWorkingMemoryEntry(_wmc.m_address);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
        AEdge _currEdge = (AEdge) wme.getData();

        log("AEdge: start node ID" + _currEdge.m_startNodeID + " - end node ID " + _currEdge.m_endNodeID);
        
        log("What am I supposed to do with a changed edge?");
        throw new RuntimeException("AEdges were supposed to not be changed! So this breaks several assumptions. Check your code!");
	}
	
	private void processDeletedAEdge(WorkingMemoryChange _wmc) {
        ReasonerRelation _rel = m_wma2rel.remove(_wmc.m_address);
        
        log("The following AEdge has been deleted on navWM: "+_rel);
        log("What am I supposed to do with a deleted edge?");
        // delete relation euqivalent does not exist.
//        m_funcWriter.deleteInstance(null, 
//        		OntologyMemberFactory.createInstance("", "", "node"+_nodeID));
        throw new RuntimeException("AEdges were supposed to not be deleted! So this breaks several assumptions. Check your code!");
	}

	

	
	private void processNewVEdge(WorkingMemoryChange _wmc) {
        CASTData<?> wme;
		try {
			wme = getWorkingMemoryEntry(_wmc.m_address);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
        VEdge _currEdge = (VEdge) wme.getData();

        log("VEdge: start node ID" + _currEdge.m_startNodeID + " - end node ID " + _currEdge.m_endNodeID);
        
        log("creating a new relation");
        ReasonerInstance _ins1 = m_oeMemberMaker.createInstance("node"+_currEdge.m_startNodeID);
        ReasonerInstance _ins2 = m_oeMemberMaker.createInstance("node"+_currEdge.m_endNodeID); 
        ReasonerRelation _rel = m_oeMemberMaker.createRelation("visibleFrom", _ins1, _ins2); 
        m_funcWriter.addInstanceInstanceRelation(
        		new WorkingMemoryChangeReceiver() {
				  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				  		handleComaReasonerFunctionDone(_wmc);} }, 
				  		_ins1, _ins2, _rel);

        m_wma2rel.put(_wmc.m_address, _rel);
        throw new RuntimeException("VEdges were supposed to not be used! So this breaks several assumptions. Check your code!");
	}
    
    
	private void processChangedVEdge(WorkingMemoryChange _wmc) {
        CASTData<?> wme;
		try {
			wme = getWorkingMemoryEntry(_wmc.m_address);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
        VEdge _currEdge = (VEdge) wme.getData();

        log("VEdge: start node ID" + _currEdge.m_startNodeID + " - end node ID " + _currEdge.m_endNodeID);
        
        log("What am I supposed to do with a changed edge?");
        throw new RuntimeException("VEdges were supposed to not be used! So this breaks several assumptions. Check your code!");
	}
	
	private void processDeletedVEdge(WorkingMemoryChange _wmc) {
        ReasonerRelation _rel = m_wma2rel.remove(_wmc.m_address);
        
        log("The following VEdge has been deleted on navWM: "+_rel);
        log("What am I supposed to do with a deleted edge?");
        // delete relation euqivalent does not exist.
//        m_funcWriter.deleteInstance(null, 
//        		OntologyMemberFactory.createInstance("", "", "node"+_nodeID));
        throw new RuntimeException("VEdges were supposed to not be used! So this breaks several assumptions. Check your code!");
	}

	
	public void handleComaReasonerFunctionDone(WorkingMemoryChange _wmc) {
		// I don't really have to do anything special;
		// just clean the WM
		m_funcWriter.cleanUpAfterFunctionEvaluated(_wmc.m_address);
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
