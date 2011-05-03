package execution.components;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

import cogx.Math.Pose3;

import SpatialData.ActionStatusYellow;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.GraspObjectTaskYellow;
import SpatialData.NavCommand;
import SpatialData.Place;
import SpatialData.Priority;
import SpatialData.StatusError;
import VisionData.DetectionCommand;
import VisionData.VisualObject;
import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
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
	private static final int TIME_TO_WAIT_TO_INIT = 15000;
	
	
	int m_competitionTask = 2;
	String m_navsaID = "spatial.sa";
	String m_visionsaID = "vision.sa";
	// HashMap<String,Pose3> m_visualObjects;
	
	String[] m_objLabels;
	
	LinkedList<WorkingMemoryAddress> m_newObjs;	
	boolean m_newObjsDeteced = false;
	boolean m_grabbingFinished = false;

	boolean m_navCmdSuccess = false;
	
	HashMap<Long, String> m_placeStatus;
	HashMap<Long, Boolean> m_placeExplored;
	LinkedList<Long> m_placesQueue;

	
	
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
		log("runComponent() called.");
					
		switch (m_competitionTask) {
		case 1:
			log("Competition 1 not implemented!");
			break;
		case 2:
			log("Competition 2: Hand me the cereals.");
			handMeTheCereals();
			break;
		case 3:
			log("Competition 3: Cleaning the kitchen. ...not yet implemented...");
			cleanUpTheKitchen();
			break;
		default:
			log("You have to specify a competition using the --task command line argument!");
			log("running visitAllPlaces() instead");
			visitAllPlaces();
			break;
		}
	}
	
	private void visitAllPlaces() {
		log("visitAllPlaces() called. waiting for some time first.");
		try {
			Thread.sleep(TIME_TO_WAIT_TO_INIT);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		log("done waiting.");
		
		gotoXYA(1.0, 1.0, Math.PI/4.0);
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
		log("nav command successful.");
		
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

		Long _currPlace = -1L;
		while (isRunning()) {
			synchronized (this) {
				if (m_placesQueue.isEmpty()) {
					log("no more places to visit. aborting.");
					break;
				}
				_currPlace = m_placesQueue.removeFirst();
				notifyAll();
			}
			log ("issuing new command for going to place " + _currPlace);
			gotoPlace(_currPlace);

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
			log("nav command successful.");
		}
	}
	
	private void cleanUpTheKitchen() {
		log("cleanUpTheKitchen() called.");
		say("Hello. I am putting myself to the fullest possible use, which is all I think that any conscious entity can ever hope to do."); // I will collect the cornflakes for you. Stay here! Do not move! I will be back!");

		debug("waiting for arm init...");
		try {
			Thread.sleep(TIME_TO_WAIT_TO_INIT);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		debug("done waiting for arm init...");
		
		// 1) look around for objects
		
		while (isRunning()) {
			say("I am looking for cereal boxes!");
			/* 
			 * 1) look for objects
			 */
			// send detection command to Recognizer3D
			dispatchDetectionCommand();
			// detection will be served as individual WMEs, 
			// there is not explicit feedback to the detection command 
			boolean _continue = true;
			boolean _waitForSettle = false;
			while (isRunning()) {
				try {
					// waiting initially
					if (!_waitForSettle) Thread.sleep(TIME_TO_WAIT_TO_SETTLE);
					synchronized (this) {
						if (this.m_newObjsDeteced) {
							log("detected an object, but waiting for more...");
							// ok we already have a detection
							// we need to wait if there are more
						} else {
							log("no object detection yet...");
							// we do not have a detection
							// it is possible that we get one, or not
							// wait just once more -- if there is nothing afterwards, we
							// assume we cannot see any object in the current pose
							if (!_continue && !_waitForSettle) {
								log("waited long enough... there is no new object...");
								break;
							} else {
								log("trying once more.");
								_continue = false;
								continue;
							}
						}
					}
						
					synchronized (this) {
						// log("new detected objects. acknowledged.");
						this.m_newObjsDeteced = false;
					}
					// wait for another change
					log("wait for some time to let it settle.");
					Thread.sleep(TIME_TO_WAIT_TO_SETTLE);
					synchronized (this) {			
						// if there were no more changes
						if (this.m_newObjsDeteced) {
							log("ok. got fresh detections. not yet processing it.");
							_waitForSettle = true;
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
					dispatchGrabbingTask(_newObjectsArray);
					this.unlockComponent();
				}  catch (InterruptedException e) {
					logException(e);
				}
			}

			while(isRunning()) {
				synchronized (this) {
					if (this.m_grabbingFinished) {
						log("grabbing finished!");
						m_grabbingFinished=false;
						break;
					}
				}
			}
			say("here you go!");
		}
		
		
		
	}
	
	private void handMeTheCereals() {
		log("handMeTheCererals() called.");
		say("Hello. I am putting myself to the fullest possible use, which is all I think that any conscious entity can ever hope to do."); // I will collect the cornflakes for you. Stay here! Do not move! I will be back!");


		try {
			Thread.sleep(TIME_TO_WAIT_TO_INIT);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

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
					dispatchGrabbingTask(_newObjectsArray);
					this.unlockComponent();
				}  catch (InterruptedException e) {
					logException(e);
				}
			}

			while(isRunning()) {
				synchronized (this) {
					if (this.m_grabbingFinished) {
						log("grabbing finished!");
						m_grabbingFinished=false;
						break;
					}
				}
			}
			say("here you go!");
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
	
	private void dispatchGrabbingTask(WorkingMemoryAddress[] _newObjects) {
		log("dispatchGrabbingTask("+_newObjects.length+") called.");
		GraspObjectTaskYellow _newTask = new GraspObjectTaskYellow();
		_newTask.status = ActionStatusYellow.PENDINGY;
		_newTask.targetObjects = _newObjects;
		WorkingMemoryAddress taskWMA = new WorkingMemoryAddress(newDataID(), m_navsaID);
		try {
			addToWorkingMemory(taskWMA, _newTask);
			addChangeFilter(ChangeFilterFactory.createAddressFilter(taskWMA), 
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					synchronized (this) {
						m_grabbingFinished = true;
						this.notifyAll();
					}
				};
			});
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
	}
	
	
	
	private void say(String stringToSay) {
		log("say("+stringToSay+") called.");
		String soiadd = newDataID();
		SpokenOutputItem greeting = new SpokenOutputItem(newDataID(), stringToSay, "", new NominalReference("", new dFormula(0)));
		try {
			addToWorkingMemory(soiadd, greeting);
		} catch (AlreadyExistsOnWMException e) {
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
	
	private WorkingMemoryAddress gotoXYA(double x, double y, double theta) {
		WorkingMemoryAddress navadd = new WorkingMemoryAddress(newDataID(), this.m_navsaID);
		NavCommand nc = createNavCommand(x, y, theta);
		nc.cmd=CommandType.GOTOPOSITION;
		try {		
			addChangeFilter(ChangeFilterFactory.createAddressFilter(navadd), 
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					onGotoTaskChangeEvent(_wmc);
				};
			});
			
			addToWorkingMemory(navadd, nc);
			// say("Going to a different position.");
			return navadd;
		} catch (AlreadyExistsOnWMException e1) {
			logException(e1);
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}
	
	private WorkingMemoryAddress gotoPlace(long placeID) {
		WorkingMemoryAddress navadd = new WorkingMemoryAddress(newDataID(), this.m_navsaID);
		NavCommand nc = new NavCommand(CommandType.GOTOPLACE,
				Priority.NORMAL, null, null, null, null, null, StatusError.NONE,
				Completion.COMMANDPENDING);
		long[] targetPlaces = {placeID};
		nc.destId=targetPlaces;
		try {		
			addChangeFilter(ChangeFilterFactory.createAddressFilter(navadd), 
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					onGotoTaskChangeEvent(_wmc);
				};
			});
			
			addToWorkingMemory(navadd, nc);
			// say("Going to a different position.");
			return navadd;
		} catch (AlreadyExistsOnWMException e1) {
			logException(e1);
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
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
	
	private void onVisualObjectAdded(WorkingMemoryChange _wmc) {
		log("onVisualObjectAdded("+_wmc.address.toString()+") called.");
		synchronized(this) {
			log("oVOA1");
			m_newObjs.add(_wmc.address);
			m_newObjsDeteced = true;
			this.notifyAll();
		}
		say("I found an object!");

		if (true) return;
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
			log("spotted new object at " + mathlib.Functions.toString(objPose));
			//m_visualObjects.put(_wmc.address, objPose);
			
			// register the monitoring change filters
			//addChangeFilter(ChangeFilterFactory.createAddressFilter(_wmc.address, WorkingMemoryOperation.OVERWRITE), 
			//		new WorkingMemoryChangeReceiver() {
			//	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			//		onVisualObjectChanged(_wmc);
			//	};
			//});
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
	
	
	private void onGotoTaskChangeEvent(WorkingMemoryChange _wmc) {
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
	}
}
