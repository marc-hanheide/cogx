/**
 * 
 */
package vision.components;

import java.util.HashMap;
import java.util.Map;

import mathlib.Functions;
import vision.VisionUtils;
import NavData.RobotPose2d;
import Video.CameraParameters;
import Video.CameraParametersWrapper;
import VisionData.DetectionCommand;
import VisionData.Post3DObject;
import VisionData.VisualObject;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import castutils.castextensions.WMEventQueue;
import cogx.Math.Matrix33;
import cogx.Math.Pose3;
import cogx.Math.Vector3;

/**
 * @author marc
 * 
 */
public class FakeDetector extends ManagedComponent implements
		WorkingMemoryChangeReceiver {

	private WMEventQueue recognitionCommandEvents = new WMEventQueue();
	private CameraParameters currentCameraParameters;
	private Map<String, Post3DObject> knowObjects = new HashMap<String, Post3DObject>();
	private RobotPose2d currentRobotPose;

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> _config) {
		Pose3 pose = new Pose3(new Vector3(0, 0, 0), new Matrix33(1, 0, 0, 0,
				1, 0, 0, 0, 1));
		knowObjects.put("table", new Post3DObject("table", pose));
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		while (isRunning()) {
			try {
				WorkingMemoryChange event = recognitionCommandEvents.take();
				log("received detection command: " + CASTUtils.toString(event));
				DetectionCommand dc = getMemoryEntry(event.address,
						DetectionCommand.class);
				for (String label : dc.labels) {
					if (knowObjects.containsKey(label)) {
						log("checking detection of known object " + label);
						Post3DObject fakeDetection = knowObjects.get(label);
						Pose3 objPose = checkVisibility(fakeDetection);
						if (objPose != null) {
							log("fake object " + label
									+ " is visible. submitting to WM.");
							Post3DObject postCmd = new Post3DObject(label,
									objPose);
							addToWorkingMemory(newDataID(), postCmd);
						}
					}
				}
			} catch (CASTException e) {
				logException(e);
			} catch (InterruptedException e) {
				logException(e);
			}
		}
	}

	private Pose3 checkVisibility(Post3DObject fakeDetection)
			throws InterruptedException {
		RobotPose2d robotPose = getCurrentRobotPose();
		log("checkVisibility: current robotPose is " + robotPose.x + ", "
				+ robotPose.y);
		Pose3 robPose3D = Functions.pose3FromEuler(new Vector3(robotPose.x,
				robotPose.y, 0.0), 0.0, 0.0, robotPose.theta);
		log("checkVisibility: robot in world coords "
				+ Functions.toString(robPose3D));
		Pose3 objInRobotCoord = Functions.transformInverse(robPose3D,
				fakeDetection.pose);
		log("checkVisibility: object in robot coords is "
				+ Functions.toString(objInRobotCoord));
		log("checkVisibility: getCurrentCameraParameters().pose "
				+ Functions.toString(getCurrentCameraParameters().pose));
		if (VisionUtils.isVisible(getCurrentCameraParameters(),
				objInRobotCoord.pos)) {
			return objInRobotCoord;
		} else {
			return null;
		}
	}

	private synchronized RobotPose2d getCurrentRobotPose()
			throws InterruptedException {
		while (currentRobotPose == null) {
			wait();
		}
		return currentRobotPose;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				DetectionCommand.class, WorkingMemoryOperation.ADD),
				recognitionCommandEvents);

		addChangeFilter(ChangeFilterFactory
				.createGlobalTypeFilter(RobotPose2d.class), this);

		addChangeFilter(ChangeFilterFactory
				.createGlobalTypeFilter(CameraParametersWrapper.class), this);

	}

	@Override
	public void workingMemoryChanged(WorkingMemoryChange arg0)
			throws CASTException {
		if (arg0.operation == WorkingMemoryOperation.DELETE)
			return;
		if (arg0.type.equals(CASTUtils.typeName(RobotPose2d.class))) {
			// new RobotPose
			RobotPose2d robotPose = getMemoryEntry(arg0.address,
					RobotPose2d.class);
			setCurrentRobotPose(robotPose);
		} else {
			// new CameraParams
			CameraParametersWrapper cpw = getMemoryEntry(arg0.address,
					CameraParametersWrapper.class);
			setCurrentCameraParameters(cpw.cam);
		}
	}

	private synchronized void setCurrentRobotPose(RobotPose2d robotPose) {
		currentRobotPose = robotPose;
		notifyAll();
	}

	protected synchronized void setCurrentCameraParameters(CameraParameters cam) {
		currentCameraParameters = cam;
		notifyAll();
	}

	protected synchronized CameraParameters getCurrentCameraParameters()
			throws InterruptedException {
		while (currentCameraParameters == null) {
			wait();
		}
		return currentCameraParameters;
	}

}
