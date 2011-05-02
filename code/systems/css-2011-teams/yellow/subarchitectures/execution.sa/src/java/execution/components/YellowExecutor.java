package execution.components;

import java.util.LinkedList;
import java.util.Map;

import cogx.Math.Pose3;

import SpatialData.ActionStatusYellow;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.GraspObjectTaskYellow;
import SpatialData.NavCommand;
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
	}
	
	
	public void start() {
		log("start() called.");
		
		// register the monitoring change filters
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(VisualObject.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				onVisualObjectAdded(_wmc);
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
			// cleanTheKitchen();
			break;
		default:
			log("You have to specify a competition using the --task command line argument!");
			break;
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
		if (!(_wmc.operation.equals(WorkingMemoryOperation.OVERWRITE))) {
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
			//nc.comp
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
