package exploration;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.Line2D;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.Date;
import java.util.Vector;

import javax.swing.Timer;

import util.CastComponent;
import NavData.FNode;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialProperties.PathProperty;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import clustering.ProbGenerator;

/**
 * class that will explore a network/graph
 * 
 * 
 * @author ken
 * 
 */
public class IntelligentGraphExplorer extends CastComponent implements
		ActionListener {

	private String begin = "begin";
	private String running = "running";
	private String idle = "idle";
	private Vector<PathProperty> tempPaths;
	private ArrayList<Integer> bounds;
	private ArrayList<ArrayList<Integer>> toInvestigate;// contains the
	// startTimes for any
	// times to investigate
	private ArrayList<ArrayList<Integer>> lowPaths;// contains the paths that
	// have only been visited a
	// few times
	private int nextTime;
	private int minPath = 2;

	private boolean includeDay = false;

	public IntelligentGraphExplorer() {
		timer = new Timer(10000, this);
		tempPaths = new Vector<PathProperty>();
		nodes = new Vector<FNode>();
		pathTimes = new Vector<PathTimes>();
		pos = 0;
		loaded = false;
		heading = 0;
		start = true;
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

		if (nodes.size() < 1) {
			println("graph is not compiled, ken grab the controller ");
			return;
		}

		println("number of nodes found " + nodes.size());
		println("number of paths found " + tempPaths.size());
		convertToPathTimes();
		System.out.println("heading to closest node");
		goTo(getClosestNode(x, y, nodes));

		while (true) {
			generateToInvestigate();

			nextTime();

			System.out.println("next time is " + nextTime);
			System.out.println("size of nodes " + nodes.size());
			System.out
					.println("to investigate size is " + toInvestigate.size());
			System.out.println("to investigate " + toInvestigate);

			System.out.println("at first node");
			System.out
					.println("about to investigate any paths we have too few runs for");
			runUntil(nextTime, lowPaths);
			if (timeInMins() < nextTime) {
				System.out
						.println("about to investifate any paths we have high entropy for");
				runUntil(nextTime, toInvestigate);
			}
			System.out.println("finished run until");
			printPathTimes();

		}

	}

	/**
	 * generates a list of paths that need to be investigated
	 */
	private void generateToInvestigate() {
int today = new Date(System.currentTimeMillis()).getDay();
		toInvestigate = new ArrayList<ArrayList<Integer>>();
		lowPaths = new ArrayList<ArrayList<Integer>>();
		for (int i = 0; i < pathTimes.size(); i++) {
			ProbGenerator p = new ProbGenerator(i, false, false, includeDay,today);
		
				toInvestigate.add(p.getTimesToInvestigate());
				lowPaths.add(p.getPathsLessThan(minPath));
						
			
			// System.out.println("from prob generator " + temp);
			if (i == 0) {
				bounds = p.getBounds(true);
			}
		}
	}

	/**
	 * sets next time to the next time interval also returns that value
	 * 
	 * @return
	 */
	private int nextTime() {
		int curr = timeInMins();
		System.out.println("current time is " + curr);
		for (int time : bounds) {
			if (time > curr) {
				nextTime = time;
				break;
			}
		}
		return nextTime;
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
		save();
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
	 * runs through the provided lists of paths and saves them
	 * 
	 * 
	 * @param paths
	 */
	public void runPathsUntil(Vector<PathTimes> paths,
			Vector<PathTimes> visited, Vector<PathTimes> blocked, int time)
			throws PathRunFailure {

		if (paths.size() == 0) {
			println("we should stop and wait I guess");
			long sleepTime = (((nextTime - timeInMins()) * 60) - seconds()) * 1000;
			println("sleeping for " + sleepTime);
			sleepComponent(sleepTime);

		}

		for (PathTimes path : paths) {

			int a;
			int b;
			if (timeInMins() >= time) {// in the event of things going weird on
				// monday, it's probably this line
				println("time is up");
				return;
			}
			if (TourFinder.distFromPaths(pathTimes, pos, path.getA()) < TourFinder
					.distFromPaths(pathTimes, pos, path.getB())) {
				// getA() is closer
				a = path.getA();
				b = path.getB();

			} else {// getB() is closer
				a = path.getB();
				b = path.getA();
			}

			if (pos != a) {
				goTo(pos);
				Vector<PathTimes> pT = TourFinder.route(pos, a, pathTimes,
						blocked);
				runPathsUntil(pT, visited, blocked, time);

			}

			turnToNode(b);
			println("facing next node, commencing path run");
			PathRun newPath = new PathRun(new Date(System.currentTimeMillis()));
			Completion comp = goTo(b);
			path.add(newPath);

			if (!visited.contains(path)) {
				visited.add(path);
			}
			if (comp == Completion.COMMANDSUCCEEDED) {
				newPath.setTimeTaken(new Date(System.currentTimeMillis()));
				save();
			} else {
				blocked.add(path);
				save();
				throw new PathRunFailure(visited, blocked, " path run failed");
			}

			println(visited.size() + " out of " + pathTimes.size()
					+ " completed");
		}
		save();
	}

	/**
	 * uses the PathFinder class to attempt to find a better path to run than
	 * simply how the paths were found
	 */
	public void runUntil(int time,
			ArrayList<ArrayList<Integer>> pathsToInvestigate) {

		int posInTime = 0;
		for (int i = 0; i < bounds.size(); i++) {
			if (bounds.get(i) == nextTime) {
				posInTime = bounds.get(i - 1);
			}
		}
		Vector<PathTimes> pathsNotToInvestigate = new Vector<PathTimes>();

		for (int i = 0; i < pathTimes.size(); i++) {
			if (!contains(pathsToInvestigate.get(i), posInTime)) {

				pathsNotToInvestigate.add(pathTimes.get(i));
			}
		}

		System.out.println(" not to investigate is " + pathsNotToInvestigate);

		TourFinder tF = new TourFinder(pathTimes, pos);
		Vector<PathTimes> times = tF.getBestPath(pos);

		Vector<PathTimes> blocked = new Vector<PathTimes>();
		Vector<PathTimes> visited = pathsNotToInvestigate;
		boolean success = false;

		times = tF.generatePath(times, blocked, visited, pos);
		System.out.println("path generated?");
		System.out.println("path is size " + times.size());
		System.out.println("path is " + times);
		while (!success) {
			println("path times length " + times.size());
			success = true;
			try {
				runPathsUntil(times, visited, blocked, time);
			} catch (PathRunFailure e) {
				success = false;
				times = tF.generatePath(times, blocked, visited, pos);
				println("problem in route, trying new one");
				println("i think I'm at position " + pos);
				println("oh don't blame me, you coded it");
				println("blocked " + blocked.size());
				println("visited " + visited.size());
			}
		}
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

	public int timeInMins() {

		return (new Date(System.currentTimeMillis())).getMinutes() + 60
				* (new Date(System.currentTimeMillis())).getHours();
	}

	public int seconds() {
		return new Date(System.currentTimeMillis()).getSeconds();
	}

	public static boolean contains(ArrayList<Integer> list, Integer in) {
		for (Integer l : list) {
			// System.out.println(" l value is " + l + " & in value is " + in);
			if (l.intValue() == in.intValue()) {

				return true;
			}
		}
		return false;
	}

	public static void main(String[] args) {

		ArrayList<Integer> list = new ArrayList<Integer>();

		System.out.println("false " + contains(list, 2));
		list.add(new Integer(2));
		System.out.println("true " + contains(list, 2));
	}

}
