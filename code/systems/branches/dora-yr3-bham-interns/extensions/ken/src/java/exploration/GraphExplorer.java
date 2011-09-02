package exploration;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.Line2D;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.Date;
import java.util.Vector;

import javax.swing.Timer;

import NavData.FNode;
import NavData.RobotPose2d;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialData.Priority;
import SpatialData.StatusError;
import SpatialProperties.PathProperty;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEventQueue;


/**
 * class that will explore a network/graph
 * @author ken
 *
 */
public class GraphExplorer extends ManagedComponent implements ActionListener {

	private String begin = "begin";
	private String running = "running";
	private String idle = "idle";

	private Timer timer;
	private Vector<FNode> nodes;
	private Vector<PathProperty> tempPaths;
	private Vector<PathTimes> pathTimes;
	private int pos; // the node the robot is currently on
	private boolean loaded;
	private double heading; // the direction in which the robot is heading

	public GraphExplorer() {
		timer = new Timer(2000, this);
		tempPaths = new Vector<PathProperty>();
		nodes = new Vector<FNode>();
		pathTimes = new Vector<PathTimes>();
		pos = 0;
		loaded = false;
		heading = 0;
	}

	public void runComponent() {
		println("explorer is running");
	}

	/**
	 * adds in filters and begins the timer
	 */
	public void start() {
		try {
			load();
		} catch (FileNotFoundException e) {
			println("file not found");
		}

		addFNodeFilter();
		addPathFilter();
		addPoseFilter();
		timer.restart();
		timer.setActionCommand(begin);

	}

	/**
	 * informs the robot that the filters have collected all they can
	 */
	public void begin() {
		timer.stop();
		timer.setActionCommand(running);
		println("number of nodes found " + nodes.size());
		println("number of paths found " + tempPaths.size());
		convertToPathTimes();

		simpleRun();

		printPathTimes();

		timer.setActionCommand(idle);

	}

	/**
	 * converts the PathProperties into PathTimes removes duplicate entries
	 * (from paths being generated both ways) can be used multiple times to add
	 * in any new paths found
	 */
	public void convertToPathTimes() {
		for (PathProperty path : tempPaths) {
			if (path.place1Id < path.place2Id) {

				if (loaded) {
					if (!pathAlreadyExists((int) path.place1Id,
							(int) path.place2Id)) {
						pathTimes.add(new PathTimes((int) path.place1Id,
								(int) path.place2Id));
					}
				} else {
					pathTimes.add(new PathTimes((int) path.place1Id,
							(int) path.place2Id));
				}
			}
		}
		tempPaths.clear();
		loaded = false;
		println("path times compiled, there are " + pathTimes.size());
	}

	/**
	 * return true if there exists a path between the two specified points
	 * 
	 * @param a
	 * @param b
	 * @return
	 */
	public boolean pathAlreadyExists(int a, int b) {
		for (PathTimes path : pathTimes) {
			if ((path.getA() == a && path.getB() == b)
					|| (path.getA() == b && path.getB() == a)) {
				return true;
			}
		}
		return false;
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
	 * runs through the provided lists of paths and saves them
	 * 
	 * @param paths
	 */
	public void runPaths(Vector<PathTimes> paths) {
		int index = 0;
		for (PathTimes path : paths) {
			int a;
			int b;
			if (TourFinder.distFromPaths(paths, pos, path.getA()) < TourFinder
					.distFromPaths(paths, pos, path.getB())) {
				// getA() is closer
				a = path.getA();
				b = path.getB();

			} else {// getB() is closer
				a = path.getB();
				b = path.getA();
			}
			if (pos != a) {
				goTo(a);

			}
			turnToNode(b);
			PathRun newPath = new PathRun(new Date(System.currentTimeMillis()));
			goTo(b);
			newPath.setTimeTaken(new Date(System.currentTimeMillis()));
			path.add(newPath);
			index++;
			println(index + " out of " + pathTimes.size() + " completed");
		}
		save();
	}

	/**
	 * runs through each path in the order they came in
	 */
	public void naiveRun() {
		runPaths(pathTimes);
	}

	/**
	 * uses the PathFinder class to attempt to find a better path to run than
	 * simply how the paths were found
	 */
	public void simpleRun() {
		runPaths(new TourFinder(pathTimes, pos).getBestPath());
	}

	/**
	 * what to do when a timer goes off ActionEvents used to differentiate
	 * between what should be done
	 */
	@Override
	public void actionPerformed(ActionEvent e) {
		if (e.getActionCommand().equals(begin)) {
			begin();
		}
		if (e.getActionCommand().equals(idle)) {
			if (tempPaths.size() > 0) {
				convertToPathTimes();
				save();
			}
		}

	}

	/**
	 * goes to the given place (where the place is the nodeId of an FNode/Place
	 * 
	 * @param place
	 */
	public void goTo(int place) {

		NavCommand nav = createNavCommand(place);
		pos = place;

		try {
			println("heading to " + nav.destId[0]);

			executeNavCommand(nav);

		} catch (CASTException e) {
			println("error");
			println(e);
			e.printStackTrace();
		} catch (InterruptedException e) {
			println("error");
			println(e);
			e.printStackTrace();
		}
	}

	/**
	 * create a NavCommand to send the robot to the provided node modified from
	 * TourGiver
	 * 
	 * @param io
	 * @return
	 */
	public static NavCommand createNavCommand(int nodeId) {
		return new NavCommand(CommandType.GOTOPLACE, Priority.NORMAL,
				new long[] { nodeId }, new double[0], new double[0],
				new double[0], new double[0], StatusError.UNKNOWN,
				Completion.COMMANDPENDING);
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
	 * code copied from TourGiver the robot will execute the provided navCommand
	 * 
	 * @param navCommand
	 * @return
	 * @throws CASTException
	 * @throws InterruptedException
	 */
	private Completion executeNavCommand(NavCommand navCommand)
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
	 * given the index of a FNode, will turn the robot towards it
	 * 
	 * @param index
	 */
	public void turnToNode(int index) {

		double amt = getTurnAmount(index);
		NavCommand cmd = generateTurn(amt);

		try {
			executeNavCommand(cmd);

		} catch (CASTException e) {
			println(e);
			e.printStackTrace();
		} catch (InterruptedException e) {
			println(e);
			e.printStackTrace();
		}
	}

	/**
	 * given the index of a node, will return how much the robot needs to turn
	 * to be facing it
	 * 
	 * @param index
	 * @return
	 */
	public double getTurnAmount(int index) {

		if (heading > Math.PI) {
			heading = heading - 2 * Math.PI;
		}
		double amt = requiredHeading(index) - heading;
		if (amt > Math.PI) {
			amt = amt - 2 * Math.PI;
		}

		return amt;
	}

	/**
	 * given the index of a FNode, compute the heading from the current FNode
	 * 
	 * @param index
	 * @return
	 */
	public double requiredHeading(int index) {
		double x0, x1, x2, y0, y1, y2;
		x0 = nodes.get(pos).x;
		y0 = nodes.get(pos).y;
		x1 = x0 + 1;// along the x axis is heading 0 in this coordinate frame
		y1 = y0;
		x2 = nodes.get(index).x;
		y2 = nodes.get(index).y;
		Line2D.Double a = new Line2D.Double(x0, y0, x1, y1);
		Line2D.Double b = new Line2D.Double(x0, y0, x2, y2);
		return angleBetween2Lines(a, b);

	}

	/**
	 * gives the angle between two lines
	 * 
	 * from http://stackoverflow.com/questions/3365171/calculating-the-angle-
	 * between-two-lines-without-having-to-calculate-the-slope-j/3366569#3366569
	 * 
	 * @param line1
	 * @param line2
	 * @return
	 */
	private double angleBetween2Lines(Line2D line1, Line2D line2) {
		double angle1 = Math.atan2(line1.getY1() - line1.getY2(), line1.getX1()
				- line1.getX2());
		double angle2 = Math.atan2(line2.getY1() - line2.getY2(), line2.getX1()
				- line2.getX2());
		return angle1 - angle2;
	}

	/**
	 * add an FNode filter this will look at working memory and report if any
	 * FNodes are added
	 */
	public void addFNodeFilter() {
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
	 * add a Path filter this will look at working memory and report if any
	 * Paths are added
	 */
	private void addPathFilter() {

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				PathProperty.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {

						try {
							PathProperty p = getMemoryEntry(_wmc.address,
									PathProperty.class);
							tempPaths.add(p);

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
	private void addPoseFilter() {
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				RobotPose2d.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {

						try {
							RobotPose2d p = getMemoryEntry(_wmc.address,
									RobotPose2d.class);
							heading = -p.theta;
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
	 * saves a set of path times to file
	 */
	public void save() {
		ObjectOutputStream out;
		try {
			File file = new File("timings.txt");
			file.delete();
			out = new ObjectOutputStream(new BufferedOutputStream(
					new FileOutputStream("timings.txt")));
			PathTimesWrapper wrap = new PathTimesWrapper(pathTimes);
			out.writeObject(wrap);
			out.close();
			println("saved");
		} catch (FileNotFoundException e) {
			println(e);
			e.printStackTrace();
		} catch (IOException e) {
			println(e);
			e.printStackTrace();
		}

	}

}
