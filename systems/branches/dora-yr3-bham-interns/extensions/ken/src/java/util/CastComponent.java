package util;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.Vector;

import javax.swing.Timer;

import NavData.FNode;
import NavData.RobotPose2d;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialData.Priority;
import SpatialData.StatusError;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEventQueue;
import exploration.PathTimes;
import exploration.PathTimesWrapper;

/**
 * abstract superclass for classes who want to travel a graph
 * 
 * @author kenslaptop
 * 
 */
public abstract class CastComponent extends ManagedComponent {

	protected Vector<PathTimes> pathTimes; // the connections between nodes as
	// well as the times to travel them
	protected Vector<FNode> nodes; // the nodes of a map
	protected double x;// the robot's x co-ordinate
	protected double y;// the robot's y co-ordinate
	protected Timer timer;
	protected int pos;// the node the robot is currently on
	protected boolean start;
	protected double heading;
	protected boolean loaded;

	/**
	 * announces that this component is running
	 */
	@Override
	public void runComponent() {
		println(getClass().getName() + " is running ");

	}

	/**
	 * goes to the given place (where the place is the nodeId of an FNode/Place
	 * 
	 * @param place
	 */
	public Completion goTo(int place) {

		NavCommand nav = createNavCommand(place);
		

		try {
			println("heading to " + nav.destId[0]);
			Completion comp = executeNavCommand(nav);
			if (comp == Completion.COMMANDSUCCEEDED) {
				pos = place;
			}
			return comp;

		} catch (CASTException e) {
			println("error");
			println(e);
			e.printStackTrace();
		} catch (InterruptedException e) {
			println("error");
			println(e);
			e.printStackTrace();
		}
		return null;
	}

	/**
	 * create a NavCommand to send the robot to the provided node modified from
	 * TourGiver
	 * 
	 * @param io
	 * @return
	 */
	protected NavCommand createNavCommand(int nodeId) {
		return new NavCommand(CommandType.GOTOPLACE, Priority.NORMAL,
				new long[] { nodeId }, new double[0], new double[0],
				new double[0], new double[0], StatusError.UNKNOWN,
				Completion.COMMANDPENDING);
	}

	/**
	 * code copied from TourGiver the robot will execute the provided navCommand
	 * 
	 * @param navCommand
	 * @return
	 * @throws CASTException
	 * @throws InterruptedException
	 */
	protected Completion executeNavCommand(NavCommand navCommand)
			throws CASTException, InterruptedException {
		String id = newDataID();
		WMEventQueue queue = new WMEventQueue();
		addChangeFilter(ChangeFilterFactory.createIDFilter(id), queue);
		addToWorkingMemory(id, "spatial.sa", navCommand);
		Completion completion = Completion.COMMANDFAILED;
		while (isRunning()) {
			WorkingMemoryChange ev = queue.take();
			if (ev.operation == WorkingMemoryOperation.OVERWRITE) {
				NavCommand nc = getMemoryEntry(ev.address, NavCommand.class);
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

	/**
	 * create a NavCommand to turn the robot the provided amount modified from
	 * TurnAndLookExecutor
	 * 
	 * @param degrees
	 *            : number of degrees to turn (in radians)
	 * @return
	 */
	public NavCommand generateTurn(double degrees) {
		NavCommand cmd = new NavCommand(CommandType.TURN, Priority.NORMAL,
				new long[0], new double[0], new double[] { -degrees },
				new double[0], new double[0], StatusError.UNKNOWN,
				Completion.COMMANDPENDING);

		return cmd;
	}

	/**
	 * add an FNode filter this will look at working memory and report if any
	 * FNodes are added
	 */
	public void addFNodeFilter() {
		println("change filter added");
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(FNode.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {

				try {
					FNode f = getMemoryEntry(_wmc.address, FNode.class);
					nodes.add(f);

					timer.restart();

				} catch (DoesNotExistOnWMException e) {

					println("error");
					println(e.getMessage());
					e.printStackTrace();
				} catch (UnknownSubarchitectureException e) {

					println("error");
					println(e.getMessage());
					e.printStackTrace();
				}
			}

		});
	}

	/**
	 * add a Pose filter this will look at working memory and report if the
	 * robot's pose is modified
	 */
	protected void addPoseFilter() {
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				RobotPose2d.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {

						try {
							RobotPose2d p = getMemoryEntry(_wmc.address,
									RobotPose2d.class);
							heading = -p.theta;
							if (start) {
								x = p.x;
								y = p.y;
								start = false;
								println("position updated");
							}

						} catch (DoesNotExistOnWMException e) {

							println("error");
							println(e.getMessage());
							e.printStackTrace();
						} catch (UnknownSubarchitectureException e) {

							println("error");
							println(e.getMessage());
							e.printStackTrace();
						}
					}

				});

	}

	/**
	 * loads a set of path times from file
	 * 
	 * @throws FileNotFoundException
	 */
	public void load() throws FileNotFoundException {
		try {
			ObjectInputStream in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream("timings.txt")));

			pathTimes = ((PathTimesWrapper) (in.readObject())).getPathTimes();
			printPathTimes();
			in.close();
		} catch (FileNotFoundException e) {
			println("unable to find file, load failed");
			throw e;

		} catch (IOException e) {
			println(e);
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			println(e);
			e.printStackTrace();

		}
		loaded = true;
		println("load finished");

	}

	/**
	 * prints out the paths and their times to the command line
	 */
	public void printPathTimes() {
		for (PathTimes times : pathTimes) {
			println(times);
		}
	}

	/**
	 * given an x and a y coordinate, as well as Vector of nodes, will find the
	 * node closest to the (x,y) coordinate
	 * 
	 * @param x
	 * @param y
	 * @param nodes
	 * @return
	 */
	public int getClosestNode(double x, double y, Vector<FNode> nodes) {
		double dist = Double.MAX_VALUE;
		int closest = 0;
		for (int i = 0; i < nodes.size(); i++) {
			double xComp = Math.pow(nodes.get(i).x - x, 2);
			double yComp = Math.pow(nodes.get(i).y - y, 2);
			double current = Math.sqrt(xComp + yComp);
			if (current < dist) {
				dist = current;
				closest = i;
			}
		}
		return closest;
	}

}
