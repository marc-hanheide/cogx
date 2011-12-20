/**
 * 
 */
package vision.components;

import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

import mathlib.Functions;
import vision.VisionUtils;
import NavData.RobotPose2d;
import Video.CameraParameters;
import Video.CameraParametersWrapper;
import VisionData.DetectionCommand;
import VisionData.Face;
import VisionData.GeometryModel;
import VisionData.Post3DObject;
import VisionData.Recognizer3DCommand;
import VisionData.Recognizer3DCommandType;
import VisionData.Vertex;
import VisionData.VisualObject;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import castutils.castextensions.WMEventQueue;
import cogx.Math.Pose3;
import cogx.Math.Vector2;
import cogx.Math.Vector3;

/**
 * report VisualObject detections for pre-defined objects
 * 
 * <pre>
 *  --geometry-hack 
 *  --pos-table "4.0, -0.0, 0.0, 0.0" --dimensions-table "1.0, 2.0, 0.55" --visibility-area-table "1.0, -4.0, 4.0, -1.0"
 * </pre>
 * 
 * @author marc
 * 
 */

public class FakeDetector extends ManagedComponent implements
		WorkingMemoryChangeReceiver {
	private static final String POS_CONFIG_PREFIX = "--pos-";
	private static final int POS_CONFIG_PREFIX_LENGTH = POS_CONFIG_PREFIX
			.length();
	private static final String DIMENSION_CONFIG_PREFIX = "--dimensions-";
	private static final int DIMENSION_CONFIG_PREFIX_LENGTH = DIMENSION_CONFIG_PREFIX
			.length();
	private static final String VISIBLEFROM_CONFIG_PREFIX = "--visibility-area-";
	private static final int VISIBLEFROM_CONFIG_PREFIX_LENGTH = VISIBLEFROM_CONFIG_PREFIX
			.length();

	// FIXME: these values should come from the CameraParameters read from WM,
	// but those are currently just wrong, so we copied values from the calib
	// file directly here:
//	private static double PREDEF_CAM_FX = 816.208435;
//	private static double PREDEF_CAM_FY = 817.011902;
//	private static double PREDEF_CAM_CX = 300.978394;
//	private static double PREDEF_CAM_CY = 251.706039;
	 // a rough estimate for the Kinect
  private static double PREDEF_CAM_FX = 530;
  private static double PREDEF_CAM_FY = 530;
  private static double PREDEF_CAM_CX = 313;
  private static double PREDEF_CAM_CY = 255;
	
	private static int PREDEF_CAM_WIDTH = 640;
	private static int PREDEF_CAM_HEIGHT = 480;

	private WMEventQueue recognitionCommandEvents = new WMEventQueue();
	private CameraParameters currentCameraParameters;
	private Map<String, Pose3> knowObjects = new HashMap<String, Pose3>();
	private Map<String, Vector3> knownGeometry = new HashMap<String, Vector3>();
	private Map<String, String> label2IdMap = new HashMap<String, String>();
	private RobotPose2d currentRobotPose;
	private boolean geometryHackEnabled = false;
	private Map<String, Vector<Double>> knownVisibilityArea = new HashMap<String, Vector<Double>>();

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> _config) {
		// Pose3 pose = new Pose3(new Vector3(0, 0, 0), new Matrix33(1, 0, 0, 0,
		// 1, 0, 0, 0, 1));
		// knowObjects.put("table", pose);

		if (_config.containsKey("--geometry-hack")) {
			println("geometry hack is enabled... the GeometryModel is a BAD hack now to work with the ObjectRelationManager properly");
			geometryHackEnabled = true;
		}

		for (String key : _config.keySet()) {
			if (key.startsWith(POS_CONFIG_PREFIX)) {
				configurePosition(_config, key);
			}
			if (key.startsWith(DIMENSION_CONFIG_PREFIX)) {
				configureDimensions(_config, key);
			}
			if (key.startsWith(VISIBLEFROM_CONFIG_PREFIX)) {
				configureVisibleArea(_config, key);
			}
		}

	}

	private void configurePosition(Map<String, String> _config, String key) {
		String label = key.substring(POS_CONFIG_PREFIX_LENGTH);

		String coordsJoined = _config.get(key);
		String[] labels = coordsJoined.split(",");

		assert (labels.length >= 4);
		double x = Double.parseDouble(labels[0]);
		double y = Double.parseDouble(labels[1]);
		double z = Double.parseDouble(labels[2]);
		double theta = Double.parseDouble(labels[3]) * Math.PI / 180.0;
		println("predefined position configured: " + label + " " + x + " " + y
				+ " " + z + " " + theta);
		Pose3 pose = Functions
				.pose3FromEuler(new Vector3(x, y, z), 0, 0, theta);
		knowObjects.put(label, pose);
	}

	private void configureDimensions(Map<String, String> _config, String key) {
		String label = key.substring(DIMENSION_CONFIG_PREFIX_LENGTH);

		String coordsJoined = _config.get(key);
		String[] labels = coordsJoined.split(",");

		assert (labels.length >= 3);

		double dx = Double.parseDouble(labels[0]);
		double dy = Double.parseDouble(labels[1]);
		double dz = Double.parseDouble(labels[2]);
		println("dimensions are provided: " + label + " " + dx + " " + dy + " "
				+ dz);
		knownGeometry.put(label, new Vector3(dx, dy, dz));

	}

	private void configureVisibleArea(Map<String, String> _config, String key) {
		String label = key.substring(VISIBLEFROM_CONFIG_PREFIX_LENGTH);

		String coordsJoined = _config.get(key);
		String[] labels = coordsJoined.split(",");

		assert (labels.length >= 4);

		Vector<Double> geom = new Vector<Double>();
		geom.add(Double.parseDouble(labels[0]));
		geom.add(Double.parseDouble(labels[1]));
		geom.add(Double.parseDouble(labels[2]));
		geom.add(Double.parseDouble(labels[3]));
		knownVisibilityArea.put(label, geom);

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
				String[] labels;
				DetectionCommand dc = null;
				Recognizer3DCommand rc = null;
				if (event.type.equals(CASTUtils
						.typeName(DetectionCommand.class))) {

					dc = getMemoryEntry(event.address, DetectionCommand.class);
					labels = dc.labels;
				} else {
					rc = getMemoryEntry(event.address,
							Recognizer3DCommand.class);
					if (rc.cmd == Recognizer3DCommandType.RECOGNIZE)
						labels = new String[] { rc.label };
					else
						continue;
				}
				for (String label : labels) {
					if (knowObjects.containsKey(label)) {
						log("checking detection of known object " + label);
						Pose3 fakeObjectPoseInWorldCS = knowObjects.get(label);
						Pose3 objPose = checkVisibility(
								fakeObjectPoseInWorldCS, label);
						if (objPose != null) {
							println("fake object " + label
									+ " is visible. submitting to WM.");
							if (geometryHackEnabled) {
								submitHackedVisualObjectToWM(label,
										fakeObjectPoseInWorldCS, true);
							} else {
								submitVisualObjectToWM(label, objPose, true);
							}
						} else {
							println("fake object " + label
									+ " is NOT visible. submitting to WM.");
							if (geometryHackEnabled) {
//								submitHackedVisualObjectToWM(label, Functions
//										.pose3FromEuler(new Vector3(0, 0, 0),
//												0, 0, 0), false);
								println("here we don't submit in case of failure");
							} else {
								submitVisualObjectToWM(label, Functions
										.pose3FromEuler(new Vector3(0, 0, 0),
												0, 0, 0), false);
							}

						}
						if (dc != null) {
							overwriteWorkingMemory(event.address, dc);
						}
						if (rc != null) {
							rc.confidence = (objPose == null) ? 0.0 : 1.0;
							overwriteWorkingMemory(event.address, rc);
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

	private void submitHackedVisualObjectToWM(String label,
			Pose3 fakeObjectPoseInWorldCS, boolean b)
			throws AlreadyExistsOnWMException, DoesNotExistOnWMException,
			ConsistencyException, PermissionException {

		VisualObject visualObject;

		String oldId = label2IdMap.get(label);
		if (oldId == null)
			visualObject = VisionUtils.newVisualObject();
		else
			visualObject = getMemoryEntry(oldId, VisualObject.class);

		visualObject.pose = fakeObjectPoseInWorldCS;
		visualObject.detectionConfidence = (b ? 1.0 : 0.0);

		visualObject.model = new GeometryModel(new Vertex[1], new Face[0]);
		Vector3 predefDimensions = knownGeometry.get(label);
		if (predefDimensions == null)
			getLogger()
					.error(
							"couldn't find predefined dimension which is a prerequisite for the geometry hack.");
		Vertex vert = new Vertex(predefDimensions, new Vector3(), new Vector2());
		visualObject.model.vertices[0] = vert;
		visualObject.identLabels = new String[] { label, "unknown" };
		visualObject.identDistrib = new double[] {
				visualObject.detectionConfidence,
				1.0 - visualObject.detectionConfidence };
		if (oldId == null) {
			String id = newDataID();
			label2IdMap.put(label, id);
			addToWorkingMemory(id, visualObject);
		} else {
			overwriteWorkingMemory(oldId, visualObject);
		}
	}

	private void submitVisualObjectToWM(String label, Pose3 objPose,
			boolean positiveDetection) throws AlreadyExistsOnWMException {
		Post3DObject postCmd = new Post3DObject(label, objPose,
				positiveDetection);
		addToWorkingMemory(newDataID(), postCmd);
	}

	private Pose3 checkVisibility(Pose3 objInWorldCS, String label)
			throws InterruptedException {
		RobotPose2d robotPose = getCurrentRobotPose();
		log("checkVisibility: current robotPose is " + robotPose.x + ", "
				+ robotPose.y);
		Vector<Double> visibilityArea = knownVisibilityArea.get(label);
		if (visibilityArea != null) {
			double minx = visibilityArea.get(0);
			double miny = visibilityArea.get(1);
			double maxx = visibilityArea.get(2);
			double maxy = visibilityArea.get(3);
			println("");
			if (robotPose.x < minx || robotPose.x > maxx || robotPose.y < miny
					|| robotPose.y > maxy) {
				println("object " + label + " cannot be visible from position "
						+ robotPose.x + ", " + robotPose.y);
				return null;
			}
		}
		Pose3 robPose3D = Functions.pose3FromEuler(new Vector3(robotPose.x,
				robotPose.y, 0.0), 0.0, 0.0, robotPose.theta);
		log("checkVisibility: robot in world coords "
				+ Functions.toString(robPose3D));
		Pose3 objInRobotCoord = Functions.transformInverse(robPose3D,
				objInWorldCS);
		Vector3 objCheckPos = (Vector3) objInRobotCoord.pos.clone();
		// FIXME: this should really be read from geometry model...
		objCheckPos.z += 0.225;
		log("checkVisibility: object in robot coords is "
				+ Functions.toString(objInRobotCoord));
		log("checkVisibility: getCurrentCameraParameters().pose "
				+ Functions.toString(getCurrentCameraParameters().pose));
		if (VisionUtils.isVisible(getCurrentCameraParameters(), objCheckPos)) {
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
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				Recognizer3DCommand.class, WorkingMemoryOperation.ADD),
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
		debug("relevant information changed: " + arg0.type);
		if (arg0.type.equals(CASTUtils.typeName(RobotPose2d.class))) {
			// new RobotPose
			RobotPose2d robotPose = getMemoryEntry(arg0.address,
					RobotPose2d.class);
			debug("new robot pose: x=" + robotPose.x + " y=" + robotPose.y);

			setCurrentRobotPose(robotPose);
		} else {
			// new CameraParams
			CameraParametersWrapper cpw = getMemoryEntry(arg0.address,
					CameraParametersWrapper.class);
			debug("new camera information: fx=" + cpw.cam.fx + " fy="
					+ cpw.cam.fy + "pose=" + Functions.toString(cpw.cam.pose));
			setCurrentCameraParameters(cpw.cam);
		}
	}

	private synchronized void setCurrentRobotPose(RobotPose2d robotPose) {
		currentRobotPose = robotPose;
		notifyAll();
	}

	protected synchronized void setCurrentCameraParameters(CameraParameters cam) {
		currentCameraParameters = cam;
		// FIXME: the following should not be necessary, but currently is...
		currentCameraParameters.cx = PREDEF_CAM_CX;
		currentCameraParameters.cy = PREDEF_CAM_CY;
		currentCameraParameters.fx = PREDEF_CAM_FX;
		currentCameraParameters.fy = PREDEF_CAM_FY;
		currentCameraParameters.width = PREDEF_CAM_WIDTH;
		currentCameraParameters.height = PREDEF_CAM_HEIGHT;
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
