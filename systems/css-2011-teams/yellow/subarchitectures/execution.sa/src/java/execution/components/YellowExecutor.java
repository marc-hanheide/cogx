package execution.components;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

import SpatialData.ActionStatusYellow;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.GraspObjectTaskYellow;
import SpatialData.NavCommand;
import SpatialData.Place;
import SpatialData.Priority;
import SpatialData.StatusError;
import VisionData.DetectionCommand;
import VisionData.KinectPlanePopOut;
import VisionData.PlaneDetectionStatus;
import VisionData.VisualObject;
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
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.slice.ref.NominalReference;
import de.dfki.lt.tr.dialogue.slice.synthesize.SpokenOutputItem;


public class YellowExecutor extends ManagedComponent {
	
	private static final int TIME_TO_WAIT_TO_SETTLE = 5000;
	private static final int TIME_TO_WAIT_TO_INIT = 16500;
	
	
	int m_competitionTask = 3;
	String m_navsaID = "spatial.sa";
	String m_visionsaID = "vision.sa";
	String m_dsaID = "dialogue";
	// HashMap<String,Pose3> m_visualObjects;
	
	String[] m_objLabels;
	
	LinkedList<WorkingMemoryAddress> m_newObjs;	
	boolean m_newObjsDeteced = false;
	boolean m_grabbingFinished = false;

	boolean m_navCmdSuccess = false;
	
	HashMap<Long, String> m_placeStatus;
	HashMap<Long, Boolean> m_placeExplored;
	LinkedList<Long> m_placesQueue;

	private WorkingMemoryAddress m_currentNavCommandWMA = null;
	private boolean m_lastGraspActionResult = false;
	
	public void configure(Map<String, String> args) {
		log("configure() called");

		if (args.containsKey("--task")) {
			this.m_competitionTask = Integer.parseInt(args.get("--task"));
		}
		if (args.containsKey("--navsa")) {
			this.m_navsaID = args.get("--navsa");
		}
		if (args.containsKey("--visionsa")) {
			this.m_visionsaID = args.get("--visionsa");
		}
		if (args.containsKey("--dsa")) {
			this.m_dsaID = args.get("--dsa");
		}
		if (args.containsKey("--objects")) {
			this.m_objLabels=args.get("--objects").split(",");
		}
		//m_visualObjects = new HashMap<String, Pose3>();
		m_newObjs = new LinkedList<WorkingMemoryAddress>();
		m_placeExplored = new HashMap<Long, Boolean>();
		m_placesQueue = new LinkedList<Long>();
		m_placeStatus = new HashMap<Long, String>();
	}
	
	
	public void start() {
		log("start() called.");
		// register the monitoring change filters

		// visualObjects
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(VisualObject.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				onVisualObjectAdded(_wmc);
			};
		});
		
		// places
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				onPlaceCallback(_wmc);
			};
		});
		
	}
	
	@Override
	protected void runComponent() {
		say("Hello. I am putting myself to the fullest possible use, which is all I think that any conscious entity can ever hope to do."); 
		// I will collect the cornflakes for you. Stay here! Do not move! I will be back!");
		log("runComponent() called. waiting for some time (" + TIME_TO_WAIT_TO_INIT + "msec) to finish the init phase...");
		try {
			Thread.sleep(TIME_TO_WAIT_TO_INIT);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		log("done waiting.");
					
		switch (m_competitionTask) {
		case 1:
			log("Competition 1 not implemented!");
			break;
		case 2:
			log("Competition 2: Hand me the cereals.");
			handMeTheCereals();
			break;
		case 3:
			log("Competition 3: Cleaning the kitchen.");
			cleanUpTheKitchen();
			break;
		default:
			log("You have to specify a competition using the --task command line argument!");
			log("running visitAllPlaces() instead");
			visitAllPlacesBlocking();
			break;
		}
		say("I am done. My existence is futile now.");
	}
	
	private void handMeTheCereals() {
		log("handMeTheCererals() called.");
		
		while (isRunning()) {
			say("I am looking for cereal boxes!");
			/* 
			 * 1) look for objects
			 */
			dispatchDetectionCommand();
			while (isRunning()) {
				try {
					synchronized (this) {
						// if there is no pending task, continue the loop 
						if (!this.m_newObjsDeteced) {
							log("no new objects detected. waiting...");
							this.wait();
							continue;
						}
					}
					synchronized (this) {
						log("new detected objects. acknowledged.");
						this.m_newObjsDeteced = false;
					}
					// wait for another change
					log("wait for some time to let it settle.");
					Thread.sleep(TIME_TO_WAIT_TO_SETTLE);
					synchronized (this) {			
						// if there were no more changes
						if (this.m_newObjsDeteced) {
							log("ok. got fresh detections. not yet processing it.");
							continue;
						}
					}
					log("waited long enough. no more fresh object detections for a while. gonna process them!");
					this.lockComponent();
					WorkingMemoryAddress[] _newObjectsArray = new WorkingMemoryAddress[m_newObjs.size()];
					for (int i = 0; i < _newObjectsArray.length; i++) {
						_newObjectsArray[i] = m_newObjs.removeFirst();
					}
					/*
					 * found new objects, dispatch grabbing task!
					 */
					executeGrabbingTaskBlocking(_newObjectsArray);
					this.unlockComponent();
				}  catch (InterruptedException e) {
					logException(e);
				}
			}
	
		
			say("here you go!");
		}
	}


	private void cleanUpTheKitchen() {
		log("cleanUpTheKitchen() called.");
		kickStartMappingBlocking();
		
		// start planepopout detection!
		// planePopOut has detected a table surface
		KinectPlanePopOut planePopOutMonitorReadyWME = new KinectPlanePopOut(PlaneDetectionStatus.READY);
		WorkingMemoryAddress _kinectStatusWMA = new WorkingMemoryAddress(newDataID(), this.m_visionsaID);
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(KinectPlanePopOut.class,WorkingMemoryOperation.OVERWRITE), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				onKinectChanged(_wmc);
			};
		});
		try {
			addToWorkingMemory(_kinectStatusWMA, planePopOutMonitorReadyWME);
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
		while (isRunning()) {
			// 1) look around for objects
			say("I am looking for cereal boxes!");
			boolean _grabbingSuccess = false;
			WorkingMemoryAddress[] _newObjectsArray = lookForObjectsBlocking();
			if (_newObjectsArray.length>0) {
				log("found " + _newObjectsArray.length + " new objects. gonna issue grabbing task!");
				_grabbingSuccess = executeGrabbingTaskBlocking(_newObjectsArray);
			} else {
				log("no objects found. will go and search...");
				try {
					planePopOutMonitorReadyWME = getMemoryEntry(_kinectStatusWMA, KinectPlanePopOut.class);
					planePopOutMonitorReadyWME.status=PlaneDetectionStatus.READY;
					overwriteWorkingMemory(_kinectStatusWMA, planePopOutMonitorReadyWME);
				} catch (DoesNotExistOnWMException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (ConsistencyException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (PermissionException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (UnknownSubarchitectureException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		
			if (!_grabbingSuccess) {
				log("grasp was not successful. trying to look again!");
				continue;
			} else {
				log("grasp successful. going to the next place...");
				visitNextPlace();
				// now start over:
				continue;
			}
		}
	}


	private WorkingMemoryAddress[] lookForObjectsBlocking() {
			dispatchDetectionCommand();
			try {
				Thread.sleep(TIME_TO_WAIT_TO_SETTLE);
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
	//		boolean waitLonger = false;
	//		synchronized (this) {
	//			if (this.m_newObjsDeteced) {
	//				log("detected an object, but waiting for more...");
	//				// this.m_newObjsDeteced = false;
	//			} else {
	//				waitLonger = true;
	//			}
	//		}
	//		if (waitLonger) Thread.sleep(TIME_TO_WAIT_TO_SETTLE);
			while (isRunning()) {
				try {
					Thread.sleep(TIME_TO_WAIT_TO_SETTLE);
					synchronized (this) {
						if(!this.m_newObjsDeteced) {
							log("no new objects detected.");
							break;
						}
					}
					log("wait for some time to let it settle.");
					synchronized (this) {			
						if (this.m_newObjsDeteced) {
							log("ok. got fresh detections.");
							this.m_newObjsDeteced=false;
							continue;
						} else {
							break;
						}
					}
	 			} catch (InterruptedException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
			}
			log("waited long enough. no more fresh object detections for a while. gonna process them!");
			this.lockComponent();
			WorkingMemoryAddress[] _newObjectsArray = new WorkingMemoryAddress[m_newObjs.size()];
			for (int i = 0; i < _newObjectsArray.length; i++) {
				_newObjectsArray[i] = m_newObjs.removeFirst();
			}
			this.unlockComponent();
			return _newObjectsArray;
		}


	private boolean visitNextPlace() {
		Long _nextPlace = -1L;
		synchronized (this) {
			if (m_placesQueue.isEmpty()) {
				log("no next place to visit. exiting");
				return false;
			} else {
				_nextPlace = m_placesQueue.removeFirst();
				m_navCmdSuccess = false;
				notifyAll();
			}
		}
		log ("issuing new command for going to place " + _nextPlace);
		return gotoPlaceBlocking(_nextPlace);
	}
	
	private boolean kickStartMappingBlocking() {
		log("kickStartMapping() called");
		synchronized(this) {
			m_navCmdSuccess = false;
			this.notifyAll();
		}
		gotoXYABlocking(1.0, 0.0, 0.0);
		return true;
	}
	
	private void visitAllPlacesBlocking() {
		log("visitAllPlaces() called.");
		
		while (isRunning()) {
			synchronized (this) {
				try {
					if (m_placesQueue.isEmpty()) {
						log("no places yet");
						this.wait();
						continue;
					} else {
						break;
					}
				}
				catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}

		while (isRunning()) {
			synchronized (this) {
				if (m_placesQueue.isEmpty()) {
					log("no more places to visit. aborting.");
					break;
				}
			}
			visitNextPlace();
		}
	}
	
	private void dispatchDetectionCommand() {
		log("dispatchDetectionCommand() called.");
		DetectionCommand detCmd = new DetectionCommand(m_objLabels);
		try {
			addToWorkingMemory(newDataID(), detCmd);
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private boolean executeGrabbingTaskBlocking(WorkingMemoryAddress[] _newObjects) {
		log("executeGrabbingTaskBlocking("+_newObjects.length+") called.");
		GraspObjectTaskYellow _newTask = new GraspObjectTaskYellow();
		log("1");
		_newTask.status = ActionStatusYellow.PENDINGY;
		log("2");
		_newTask.targetObjects = _newObjects;
		log("3");
		WorkingMemoryAddress taskWMA = new WorkingMemoryAddress(newDataID(), m_navsaID);
		log("4 WMA = " + taskWMA.id + "::" + taskWMA.subarchitecture);
		debug("done constructing the GraspObjectTask struct.");
		try {
			log("5");
			addToWorkingMemory(taskWMA, _newTask);
			log("6");
			addChangeFilter(ChangeFilterFactory.createAddressFilter(taskWMA,WorkingMemoryOperation.OVERWRITE), 
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					log("WME changed for the GraspObjectTask struct.");
					try {
						log("GOTCB1");
						GraspObjectTaskYellow currTask = getMemoryEntry(_wmc.address, GraspObjectTaskYellow.class);
						log("GOTCB2");
						synchronized (this) {
							log("GOTCB3");
							if (currTask.status.equals(ActionStatusYellow.COMPLETEY)) {
								log("GOTCB4a");
								m_lastGraspActionResult = true;
							} else {
								log("GOTCB4b");
								m_lastGraspActionResult = false;
							}
							log("GOTCB5");
							m_grabbingFinished = true;
							this.notifyAll();
							log("GOTCB6");
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
			log("added GraspObjectTask to WM.");
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		log("1");
		while(isRunning()) {
			log("2");
			log("waiting for grabbing task to finish.");
			try {
				log("3");
				synchronized(this) {
					log("4");
					if (this.m_grabbingFinished) {
						log("grabbing finished!");
						m_grabbingFinished=false;
						break;
					}
					this.wait();
					continue;
				}
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		boolean taskSuccess = false;
		synchronized (this) {
			taskSuccess = m_lastGraspActionResult;
		}
		return taskSuccess;
	}
	
	
	
	private void say(String stringToSay) {
		log("say("+stringToSay+") called.");
		String soiadd = newDataID();
		WorkingMemoryAddress soiwma = new WorkingMemoryAddress(soiadd, m_dsaID);
		SpokenOutputItem greeting = new SpokenOutputItem(newDataID(), stringToSay, "", new NominalReference("", new dFormula(0)));
		try {
			addToWorkingMemory(soiwma, greeting);
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		//addChangeFilter(ChangeFilterFactory.createAddressFilter(new WorkingMemoryAddress(soiadd, this.getSubarchitectureID())), 
		//		new WorkingMemoryChangeReceiver() {
		//	public void workingMemoryChanged(WorkingMemoryChange _wmc) 
		//	throws CASTException {
		//		log(_wmc.operation);
		//	}
		//});
	}
	
	private boolean gotoXYABlocking(double x, double y, double theta) {
		WorkingMemoryAddress navadd = new WorkingMemoryAddress(newDataID(), this.m_navsaID);
		NavCommand nc = createNavCommand(x, y, theta);
		nc.cmd=CommandType.GOTOPOSITION;
		synchronized (this) {
			this.m_currentNavCommandWMA = navadd;
			this.notifyAll();
		}
		try {		
			addChangeFilter(ChangeFilterFactory.createAddressFilter(navadd), 
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					synchronized (this) {
						m_navCmdSuccess = true;
						m_currentNavCommandWMA = null;
						notifyAll();
					}
				};
			});
			
			addToWorkingMemory(navadd, nc);
		} catch (AlreadyExistsOnWMException e1) {
			logException(e1);
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		// wait for completion
		while (isRunning()) {
			log("wait for nav command completion.");
			try {
				synchronized (this) {
					if (m_navCmdSuccess) {
						m_navCmdSuccess = false;
						break;
					} else {
						this.wait();
						continue;
					}
				}
			}catch (InterruptedException e) {
				logException(e);
			}
		}
		return true;
	}
	
	private boolean gotoPlaceBlocking(long placeID) {
		WorkingMemoryAddress navadd = new WorkingMemoryAddress(newDataID(), this.m_navsaID);
		NavCommand nc = new NavCommand(CommandType.GOTOPLACE,
				Priority.NORMAL, null, null, null, null, null, StatusError.NONE,
				Completion.COMMANDPENDING);
		long[] targetPlaces = {placeID};
		nc.destId=targetPlaces;
		nc.tolerance=new double[3];
		nc.tolerance[0]=0.1;
		nc.tolerance[1]=0.1;
		nc.tolerance[2]=Math.PI*10.0/180.0;
		synchronized (this) {
			this.m_currentNavCommandWMA = navadd;
			this.notifyAll();
		}
		try {		
			addChangeFilter(ChangeFilterFactory.createAddressFilter(navadd), 
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					synchronized (this) {
						m_navCmdSuccess = true;
						m_currentNavCommandWMA = null;
						notifyAll();
					}
				};
			});
			
			addToWorkingMemory(navadd, nc);
		} catch (AlreadyExistsOnWMException e1) {
			logException(e1);
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// wait for completion
		while (isRunning()) {
			log("wait for nav command completion.");
			try {
				synchronized (this) {
					if (m_navCmdSuccess) {
						m_navCmdSuccess = false;
						break;
					} else {
						this.wait();
						continue;
					}
				}
			}catch (InterruptedException e) {
				logException(e);
			}
		}
		return true;	
	}
	
	private NavCommand createNavCommand(double x, double y, double theta) {
		NavCommand nc = new NavCommand(CommandType.GOTOPOSITION,
				Priority.NORMAL, null, null, null, null, null, StatusError.NONE,
				Completion.COMMANDPENDING);
		nc.pose=new double[3];
		nc.pose[0]=x;
		nc.pose[1]=y;
		nc.pose[2]=Math.PI*theta/180;

		nc.tolerance=new double[3];
		nc.tolerance[0]=0.1;
		nc.tolerance[1]=0.1;
		nc.tolerance[2]=Math.PI*10.0/180.0;
		
		nc.destId=new long[1];
		nc.destId[0]=0L;
		nc.distance=new double[0];
		nc.angle=new double[0];
		return nc;
	}
		
	/* private void onVisualObjectChanged(WorkingMemoryChange _wmc) {
		VisualObject _newVisObj = null; 
		try {
			_newVisObj = getMemoryEntry(_wmc.address, VisualObject.class);
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		if (_newVisObj==null) return;
		else {
			Pose3 objPose = _newVisObj.pose;
			Pose3 oldPose = m_visualObjects.get(_wmc.address.id);
			
			//mathlib.Functions.
			
			log("overwritten object at " + mathlib.Functions.toString(objPose));
			m_visualObjects.put(_wmc.address.id, objPose);
		}	
	} */
	
	
	/* private void onGotoTaskChangeEvent(WorkingMemoryChange _wmc) {
		log("onGotoTaskChangeEvent() received with WMO " + _wmc.operation.toString());
		if ((_wmc.operation.equals(WorkingMemoryOperation.OVERWRITE))) {
			NavCommand nc = null;
			try {
				nc = getMemoryEntry(_wmc.address, NavCommand.class);
			} catch (DoesNotExistOnWMException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (UnknownSubarchitectureException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			log("onGotoTaskChangeEvent() received with completion status " + nc.comp.toString());
			synchronized (this) {
				if (nc.comp.equals(Completion.COMMANDSUCCEEDED) || 
						nc.comp.equals(Completion.COMMANDFAILED)) m_navCmdSuccess=true;
				this.notifyAll();
			}
			// m_navCmdSuccess=true;
		} else return;
	
		//if (_newVisObj==null) return;
		//else {
			//Pose3 objPose = _newVisObj.pose;
			//Pose3 oldPose = m_visualObjects.get(_wmc.address.id);
			
			//mathlib.Functions.
			
		//	log("overwritten object at " + mathlib.Functions.toString(objPose));
		//	m_visualObjects.put(_wmc.address.id, objPose);
		//}	
	} */


	private void onVisualObjectAdded(WorkingMemoryChange _wmc) {
		log("onVisualObjectAdded("+_wmc.address.toString()+") called.");
		synchronized(this) {
			log("oVOA1");
			m_newObjs.add(_wmc.address);
			m_newObjsDeteced = true;
			this.notifyAll();
		}
		say("I found an object!");
	}	
	
	private void onKinectChanged(WorkingMemoryChange _wmc) {
		log("onKinectCallback("+_wmc.address.toString()+") called.");
		KinectPlanePopOut _kinectStatus = null;
		try {
			_kinectStatus = getMemoryEntry(_wmc.address, KinectPlanePopOut.class);
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		if (_kinectStatus.status.equals(PlaneDetectionStatus.STOPROBOT)) {
			log("STOOOOOP! Found a table!");
			synchronized (this) {
				if (m_currentNavCommandWMA==null) {
					log("we do not have to do anything because we currently do not have a nav command...");
					return;
				} else {
					try {
						NavCommand _currentNavCommand = getMemoryEntry(m_currentNavCommandWMA, NavCommand.class);
						_currentNavCommand.comp = Completion.COMMANDABORTED;
						overwriteWorkingMemory(m_currentNavCommandWMA, _currentNavCommand);
						log("sending COMMANDABORTED to current nav command!");
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
					say("I found a table!");
					notifyAll();
				}
			}
		} 
	}
	
	private void onPlaceCallback(WorkingMemoryChange _wmc) {
		log("onPlaceCallback("+_wmc.address.toString()+") with WMO "+_wmc.operation.toString()+"called.");
		Place _place = null;
		try {
			_place = getMemoryEntry(_wmc.address, Place.class);
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		if (_place==null) {
			log("there was an error reading the Place from WM. doing nothing and exiting the callback method...");
			return;
		}
			
		synchronized(this) {
			if (_wmc.operation.equals(WorkingMemoryOperation.ADD)) {
				log("received a new place " + "id: "+ _place.id +", status: " + _place.status.toString());
				m_placeExplored.put(_place.id, false);
				m_placeStatus.put(_place.id, _place.status.toString());
				m_placesQueue.add(_place.id);
			} else if (_wmc.operation.equals(WorkingMemoryOperation.OVERWRITE)) {
				log("received an overwritten place " + "id: "+ _place.id +", status: " + _place.status.toString());
				m_placeStatus.put(_place.id, _place.status.toString());				
			} else if (_wmc.operation.equals(WorkingMemoryOperation.DELETE)) {
				log("received a deleted place " + "id: "+ _place.id +", status: " + _place.status.toString());
				m_placeStatus.remove(_place.id);
				m_placeExplored.remove(_place.id);
			} 
			this.notifyAll();
		}
	}
}
