/**
 * 
 */
package vision.components;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Map;

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
 * @author cogx
 * 
 */
public class ObservationModelExperiment extends ManagedComponent {

	public class Reading {
		public Reading(int setting, double angle, double distance, double confidence) {
			this.setting= setting;
			this.angle = angle;
			this.distance = distance;
			this.confidence = confidence;
		}

		int setting;
		double angle;
		double distance;
		double confidence;

		@Override
		public String toString() {
			return setting + ", " + angle + ", " + distance + ", " + confidence;
		}

	}

	private static final String DEFAULT_FILENAME = "observation-model.txt";

	private String fileName = DEFAULT_FILENAME;

	PrintWriter out;

	private static double[] TOLERANCE = new double[] { 0.1, 0.1,
			Math.PI * 5 / 180 };

	private static final double DEFAULT_RADIUS_START = 0.6;
	private static final double DEFAULT_RADIUS_STOP = 1.0;
	private static final double DEFAULT_RADIUS_STEP = 0.5;
	private static final int DEFAULT_ANGLE_STEP = 45;

	private static final String[] DEFAULT_LABELS = new String[] { "cerealbox" };

	double radiusStart = DEFAULT_RADIUS_START;
	double radiusStop = DEFAULT_RADIUS_STOP;
	double radiusStep = DEFAULT_RADIUS_STEP;
	double angleStep = Math.PI * DEFAULT_ANGLE_STEP / 180.0;

	double[] centre = new double[] { 1.0, 0.0 };

	private String[] objectLabels = DEFAULT_LABELS;

	@Override
	protected void configure(Map<String, String> config) {
		// TODO Auto-generated method stub
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
	protected void runComponent() {
		sleepComponent(5000);
		int setting=0;
		for (double currentRadius = radiusStart; currentRadius <= radiusStop; currentRadius += radiusStep) {
			for (double currentAngle = -Math.PI; currentAngle < Math.PI; currentAngle += angleStep) {
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
								.warn(
										"failed to navigate to goal. Will try the next one.");
						continue;
					}
					println("reached goal. time to run the detector");

					VisualObject nc = executeDetection();
					println("visual object: " + nc.identDistrib[0]);

					Reading r = new Reading(setting, currentAngle, currentRadius,
							nc.identDistrib[0]);
					out.println(r.toString());
					out.flush();
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
	protected void stop() {
		out.close();
	}

}
