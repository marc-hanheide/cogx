/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package castutils.experimentation;

import NavData.RobotPose2d;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * 
 * @author dunes
 */
public class DistanceMonitor extends ManagedComponent implements
		WorkingMemoryChangeReceiver {

	/**
	 * how frequently the distance the robot has moved is logged (in seconds)
	 */
	private static final long UPDATE_FREQUENCY = 10;
	/**
	 * how frequently the distance the robot has moved is logged (in metres)
	 */
	private static final double UPDATE_DISTANCE = 0.5;
	private volatile RobotPose2d prevPose = null;
	private volatile double distance = 0;
	private long nextUpdateTime = 0;
	private double nextUpdateDistance = Float.MIN_VALUE;
	private int locked = 0;

	@Override
	public void start() {

		// log("Distance Monitor started");

		addChangeFilter(ChangeFilterFactory.createTypeFilter(RobotPose2d.class,
				WorkingMemoryOperation.ADD), this);

		addChangeFilter(ChangeFilterFactory.createTypeFilter(RobotPose2d.class,
				WorkingMemoryOperation.OVERWRITE), this);

		addChangeFilter(ChangeFilterFactory.createTypeFilter(RobotPose2d.class,
				WorkingMemoryOperation.DELETE), this);
	}

	public synchronized void workingMemoryChanged(WorkingMemoryChange wmc)
			throws CASTException {

		synchronized (this) {
			if (locked != 0) {
				// log("could not obtain lock");
				return;
			} else {
				locked++;
				// log("obtained lock: "+locked);
			}
		}

		try {
			switch (wmc.operation) {
			case ADD:
				// log("new robot pose added");
				initPose(wmc);
				break;
			case OVERWRITE:
				changePose(wmc);
				break;
			case DELETE:
				throw new CASTException(
						"RobotPose2d instance unexpectedly deleted");
			default:
				throw new CASTException(
						"unexpected memory change operation in DistanceMonitor");
			}
		} finally {
			synchronized (this) {
				locked--;
				// log("released lock: "+locked);
			}
		}
	}

	private void changePose(WorkingMemoryChange change)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {

		RobotPose2d newPose = getMemoryEntry(change.address, RobotPose2d.class);

		newPose(newPose, change.timestamp);
	}

	private void initPose(WorkingMemoryChange change)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {

		if (this.prevPose != null) {
			throw new IllegalStateException("Pose already exists");
		}

		RobotPose2d newPose = getMemoryEntry(change.address, RobotPose2d.class);
		prevPose = newPose;

		newPose(newPose, change.timestamp);
	}

	private void newPose(RobotPose2d pose, CASTTime time) {

		double deltaX = pose.x - prevPose.x;
		double deltaY = pose.y - prevPose.y;

		double moved = Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));

		distance += moved;

		update(distance, moved, pose.x, pose.y, time);

		prevPose = pose;

	}

	private void update(double distance, double moved, double x, double y,
			CASTTime time) {

		if (time.s < nextUpdateTime && distance < nextUpdateDistance) {
			// println("not moved far enough: "+distance
			// +" @ time: "+CASTUtils.toString(time));
			return;
		}

		nextUpdateDistance = distance + UPDATE_DISTANCE;
		nextUpdateTime = time.s + UPDATE_FREQUENCY;

		// Use XMLTag for output
		getLogger().info(
				"<DISTANCE time=\"" + time.s + "." + time.us / 1000
						+ "\" distance=\"" + distance + "\" />");
	}
}
