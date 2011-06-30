/**
 * 
 */
package vision.components;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.util.Map;
import java.util.Map.Entry;

import ptz.PTZCompletion;
import ptz.PTZPose;
import ptz.SetPTZPoseCommand;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialData.Priority;
import SpatialData.StatusError;
import VisionData.DetectionCommand;
import VisionData.VisualObject;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEventQueue;

/**
 * run an experiment with the robot to estimate an observation model of the
 * object detector. The robot circles a centre point step-wise and always looks
 * towards the centre point at given radii. The detector is then trigger n times
 * and the result is stored in a file that be loaded into matlab.
 * 
 * @author marc
 * 
 */
public class ObservationModelExperiment extends ManagedComponent {

	private static final double CAMERA_POS_Z = 1.37;

	public class DetectionMeasurement {
		double angle;

		double confidence;
		double distance;
		int detectionSeq;
		int setting;

		public DetectionMeasurement(int setting, double angle, double distance,
				int detectionSeq, double confidence) {
			this.setting = setting;
			this.angle = angle;
			this.distance = distance;
			this.confidence = confidence;
			this.detectionSeq = detectionSeq;
		}

		@Override
		public String toString() {
			return setting + ", " + angle + ", " + distance + ", "
					+ detectionSeq + ", " + confidence;
		}

	}

	private static final int DEFAULT_ANGLE_STEP = 45;

	private static final String DEFAULT_FILENAME = "observation-model.txt";

	private static final String[] DEFAULT_LABELS = new String[] { "cerealbox" };

	private static final double DEFAULT_NUMBER_DETECTIONS = 10;

	private static final double DEFAULT_RADIUS_START = 0.7;

	private static final double DEFAULT_RADIUS_STEP = 0.5;
	private static final double DEFAULT_RADIUS_STOP = 1.0;
	private static final String SET_PREFIX = "--set-";

	private static final double DEFAULT_OBJ_POS_Z = 0.9;
	private static double[] TOLERANCE = new double[] { 0.05, 0.05,
			Math.PI * 5 / 180 };

	double angleStep = DEFAULT_ANGLE_STEP;

	double[] centre = new double[] { 1.0, 0.0 };

	private String fileName = DEFAULT_FILENAME;
	private double numberDetection = DEFAULT_NUMBER_DETECTIONS;
	private String[] objectLabels = DEFAULT_LABELS;
	PrintWriter out;

	double radiusStart = DEFAULT_RADIUS_START;

	double radiusStep = DEFAULT_RADIUS_STEP;

	double radiusStop = DEFAULT_RADIUS_STOP;

	double objectPosZ = DEFAULT_OBJ_POS_Z;

	@Override
	protected void configure(Map<String, String> config) {
		for (Entry<String, String> e : config.entrySet()) {
			if (!e.getKey().startsWith(SET_PREFIX))
				continue;
			String fieldName = e.getKey().substring(SET_PREFIX.length());
			try {
				Field f = this.getClass().getDeclaredField(fieldName);
				f.setDouble(this, Double.parseDouble(e.getValue()));
				println("configured " + fieldName + "=" + e.getValue());
			} catch (SecurityException e1) {
				logException(e1);
			} catch (NoSuchFieldException e1) {
				getLogger().warn("no such field: " + fieldName);
			} catch (NumberFormatException e1) {
				logException(e1);
			} catch (IllegalArgumentException e1) {
				logException(e1);
			} catch (IllegalAccessException e1) {
				logException(e1);
			}
		}
		if (config.containsKey("--label"))
			objectLabels = new String[] { config.get("--label") };

	}

	private VisualObject executeDetection() throws AlreadyExistsOnWMException,
			InterruptedException, DoesNotExistOnWMException,
			UnknownSubarchitectureException, SubarchitectureComponentException {
		WMEventQueue queue = new WMEventQueue();
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				VisualObject.class, WorkingMemoryOperation.ADD), queue);
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				VisualObject.class, WorkingMemoryOperation.OVERWRITE), queue);
		addToWorkingMemory(new WorkingMemoryAddress(newDataID(), "vision.sa"),
				new DetectionCommand(objectLabels));
		WorkingMemoryChange ev = queue.take();
		VisualObject nc = getMemoryEntry(ev.address, VisualObject.class);
		removeChangeFilter(queue);

		return nc;
	}

	private Completion executeNavCommand(NavCommand navCommand)
			throws CASTException, InterruptedException {
		String id = newDataID();
		WMEventQueue queue = new WMEventQueue();
		addChangeFilter(ChangeFilterFactory.createIDFilter(id), queue);
		addToWorkingMemory(id, navCommand);
		Completion completion = Completion.COMMANDFAILED;
		while (isRunning()) {
			WorkingMemoryChange ev = queue.take();
			if (ev.operation == WorkingMemoryOperation.OVERWRITE) {
				NavCommand nc = getMemoryEntry(id, NavCommand.class);
				completion = nc.comp;
				if (completion == Completion.COMMANDPENDING
						|| completion == Completion.COMMANDINPROGRESS)
					continue;
				else
					break;
			}
		}
		removeChangeFilter(queue);
		return completion;
	}

	private NavCommand generateNavCommand(double currentRadius,
			double currentAngle) {
		double x = centre[0] + Math.cos(currentAngle) * currentRadius;
		double y = centre[1] + Math.sin(currentAngle) * currentRadius;
		double theta = currentAngle - Math.PI;
		double[] pose = new double[] { x, y, theta };
		NavCommand navCommand = new NavCommand(CommandType.GOTOPOSITION,
				Priority.NORMAL, new long[0], pose, new double[0],
				new double[0], TOLERANCE, StatusError.NONE,
				Completion.COMMANDPENDING);
		return navCommand;
	}

	@Override
	protected void runComponent() {
		sleepComponent(5000);

		int setting = 0;
		for (double currentRadius = radiusStart; currentRadius <= radiusStop; currentRadius += radiusStep) {

			setPTZ(currentRadius);
			sleepComponent(2000);

			for (double currentAngle = -Math.PI; currentAngle < Math.PI; currentAngle += (angleStep
					* Math.PI / 180.0)) {
				try {
					if (!isRunning())
						return;
					println("current angle is: " + currentAngle * 180 / Math.PI);

					NavCommand navCommand = generateNavCommand(currentRadius,
							currentAngle);
					println("new goal pose: " + navCommand.pose[0] + ", "
							+ navCommand.pose[1] + ", " + navCommand.pose[2]);
					Completion comp = executeNavCommand(navCommand);
					println("navCommand completed with " + comp.toString());
					if (comp != Completion.COMMANDSUCCEEDED) {
						getLogger()
								.warn("failed to navigate to goal. Will try the next one.");
						continue;
					}
					println("reached goal. time to run the detector");

					for (int i = 0; i < (int) numberDetection; i++) {
						VisualObject nc = executeDetection();
						println("run " + i + ": object found with conf="
								+ nc.identDistrib[0]);

						DetectionMeasurement r = new DetectionMeasurement(
								setting, currentAngle, currentRadius, i,
								nc.identDistrib[0]);

						out.println(r.toString());
						out.flush();
					}

				} catch (CASTException e) {
					logException(e);
				} catch (InterruptedException e) {
					logException(e);
					return;
				}
				setting++;
			}
		}
	}

	private void setPTZ(double currentRadius) {
		SetPTZPoseCommand spc = new SetPTZPoseCommand(new PTZPose(0.0,
				Math.atan((objectPosZ - CAMERA_POS_Z) / currentRadius), 1),
				PTZCompletion.COMPINIT);
		try {
			addToWorkingMemory(newDataID(), spc);
		} catch (AlreadyExistsOnWMException e1) {
			logException(e1);
		}
	}

	@Override
	protected void start() {
		try {

			out = new PrintWriter(new File(fileName));
		} catch (FileNotFoundException e) {
			logException(e);
			throw new RuntimeException(e);
		}
	}

	@Override
	protected void stop() {
		out.close();
	}

}
