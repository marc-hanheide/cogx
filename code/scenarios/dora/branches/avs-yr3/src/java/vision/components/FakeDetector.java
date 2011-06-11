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
	private Map<String, VisualObject> knowObjects = new HashMap<String, VisualObject>();
	private RobotPose2d currentRobotPose;

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> _config) {
		// TODO Auto-generated method stub
		super.configure(_config);
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
				DetectionCommand dc = getMemoryEntry(event.address,
						DetectionCommand.class);
				for (String label : dc.labels) {
					if (knowObjects.containsKey(label)) {
						VisualObject fakeDetection = knowObjects.get(label);
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

	private Pose3 checkVisibility(VisualObject fakeDetection)
			throws InterruptedException {
		RobotPose2d robotPose = getCurrentRobotPose();
		Pose3 robPose3D = Functions.pose3FromEuler(new Vector3(robotPose.x,
				robotPose.y, 0.0), 0.0, 0.0, robotPose.theta);
		Pose3 objInRobotCoord = Functions.transformInverse(robPose3D,
				fakeDetection.pose);
		if (VisionUtils.isVisible(getCurrentCameraParameters(),
				objInRobotCoord.pos)) {
			return objInRobotCoord;
		} else {
			return null;
		}
	}

	private synchronized RobotPose2d getCurrentRobotPose()
			throws InterruptedException {
		if (currentRobotPose == null) {
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

		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(RobotPose2d.class),
				this);

		addChangeFilter(
				ChangeFilterFactory
						.createGlobalTypeFilter(CameraParametersWrapper.class),
				this);

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
		if (currentCameraParameters == null) {
			wait();
		}
		return currentCameraParameters;
	}

}
