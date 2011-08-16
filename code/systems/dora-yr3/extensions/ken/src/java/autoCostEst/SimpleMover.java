package autoCostEst;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import javax.swing.Timer;

import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialData.Place;
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

public class SimpleMover extends ManagedComponent implements ActionListener {

	private List<PathProperty> paths;
	private List<PathsToPlace> pathsToPlaces;
	private List<PathsFromPlace> pathsFromPlaces;
	private List<ConnectedPath> connectedPaths;
	private Timings[] timings;
	private int currentPos;
	private Timer timer;
	private boolean ready;

	public SimpleMover() {
		// places = new Vector<Place>();
		paths = new Vector<PathProperty>();
		pathsToPlaces = new Vector<PathsToPlace>();
		pathsFromPlaces = new Vector<PathsFromPlace>();
		connectedPaths = new Vector<ConnectedPath>();
		currentPos = 0;
		timer = new Timer(1000, this);
		timer.setActionCommand("begin");
		ready = false;

	}

	@Override
	protected void runComponent() {
		println("Simple mover is running :-)");

	}

	@Override
	public void start() {
		try {
			load();
			ready = true;
			timer.restart();
		} catch (FileNotFoundException e) {
			println("file not found, adding path filter");
			addPathFilter();
		}

	}

	/**
	 * adds a working memory change filter checking for paths then adds then to
	 * the list of paths
	 */
	public void addPathFilter() {

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				PathProperty.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {

						try {
							PathProperty e = getMemoryEntry(_wmc.address,
									PathProperty.class);
							addToPathToPlaces(e);
							addToPathFromPlaces(e);
							paths.add(e);

							println("paths found: " + paths.size());
							timer.restart();
							ready = true;
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
	 * send the robot to the given the place
	 * 
	 * @param destId
	 */
	public void goTo(Place place) {

		NavCommand nav = createNavCommand(Integer.valueOf((int) place.id));

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
	 * goes to the given place
	 * 
	 * @param place
	 */
	public void goTo(int place) {

		NavCommand nav = createNavCommand(place);

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
	 * stolen from TourGiver
	 * 
	 * @param io
	 * @return
	 */
	private NavCommand createNavCommand(Integer io) {
		return new NavCommand(CommandType.GOTOPLACE, Priority.NORMAL,
				new long[] { io.longValue() }, new double[0], new double[0],
				new double[0], new double[0], StatusError.UNKNOWN,
				Completion.COMMANDPENDING);
	}

	/**
	 * stolen from TourGiver
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

	private void goThroughPaths() {
		int nextPath = 0;
		try {
			nextPath = findNextPath();
			while (true) {
				while (nextPath != Integer.MAX_VALUE) {

					runPath(nextPath);
					nextPath = findNextPath();

				}
				// if no paths within this time period are found, look for some
				// in
				// the next
				nextPath = checkForFuturePaths();

				if (nextPath != Integer.MAX_VALUE) {// if one is found
					// go to it's start position
					println("heading to "
							+ timings[nextPath].getPrev()
							+ " in preparation for checking a route in the next time period");
					goTo((int) timings[nextPath].getPrev());

				}
				int timeTillNextPeriod = Timings.timeTillNextPeriod() * 1000;

				// println("saving");
				// save();
				println("waiting " + timeTillNextPeriod / 1000
						+ " seconds till next period");

				sleepComponent(timeTillNextPeriod);

			}
		} catch (NullPointerException e) {
			println(e);
			println("next path is " + nextPath);
			println("Timings legnth is " + timings.length);
			throw e;
		}

	}

	private int checkForFuturePaths() {

		int timeTillNextPeriod = Timings.timeTillNextPeriod();
		ArrayList<Integer> visited = new ArrayList<Integer>();
		ArrayList<Integer> toVisit = new ArrayList<Integer>();
		ArrayList<Integer> storage = new ArrayList<Integer>();
		toVisit.add(new Integer(currentPos));
		int count = 0;
		while (!toVisit.isEmpty()) {

			for (Integer node : toVisit) {

				for (int i = 0; i < timings.length; i++) {
					try {
						// if the path we are looking at, starts at the node
						// being
						// considered
						if (node == timings[i].getPrev()) {
							// need to look at the particular time here, with
							// added
							// increments
							if (timings[i]
									.getTimingIn((timeTillNextPeriod + 1) * 60) == 0) {// if
								// the
								// node
								// hasn't
								// been
								// visited
								// yet
								return i;
							} else {// if not, add to places to be considered
								// next
								storage.add(new Integer((int) (timings[i]
										.getEndPoint())));
							}
						}
					} catch (NullPointerException e) {
						println(e);
						println("place in loop " + i);
						println("timings size " + timings.length);
						throw e;
					}
				}

			}

			for (Integer i : toVisit) {// move anything in toVisit into visited
				visited.add(i);
			}
			toVisit = new ArrayList<Integer>();
			// look through those nodes at the next depth level of search
			for (Integer stor : storage) {
				boolean seen = false;
				for (Integer visit : visited) {
					if (stor.equals(visit)) {
						seen = true;
					}
				}
				if (!seen) {
					toVisit.add(stor);
				}
			}

			count++;
		}
		println("no path found");
		return Integer.MAX_VALUE;
	}

	/**
	 * the one currently in use
	 * 
	 * @param index
	 */
	private void runPath(int index) {
		ConnectedPath p = connectedPaths.get(index);
		println("path about to be run");

		println("about to travel to " + p.getPrevPath().place1Id);
		goTo((int) p.getPrevPath().place1Id);
		println("about to travel to " + p.getPrevPath().place2Id
				+ " to prepare for measuring");
		goTo((int) p.getPrevPath().place2Id);
		println("arrived at " + p.getPath().place1Id);
		println("about to travel to " + p.getPath().place2Id + " from "
				+ p.getPath().place1Id + " to measure time taken");
		long time = System.currentTimeMillis();
		goTo((int) p.getPath().place2Id);

		int timeTaken = (int) (System.currentTimeMillis() - time);
		timings[index].setCurrentTiming(timeTaken);

		println("time taken was " + timeTaken);
		currentPos = (int) p.getPath().place2Id;
	}

	/**
	 * given a path, will add it to the appropriate PathsToPlace
	 * 
	 * @param path
	 */
	private void addToPathToPlaces(PathProperty path) {
		boolean flag = false;
		for (int i = 0; i < pathsToPlaces.size(); i++) {
			if (pathsToPlaces.get(i).getPlaceID() == path.place2Id) {
				flag = true;
				pathsToPlaces.get(i).add(path);
			}

		}
		if (!flag) {
			PathsToPlace p = new PathsToPlace((int) path.place2Id);
			p.add(path);
			pathsToPlaces.add(p);
		}
	}

	/**
	 * given a path, will add it to the appropriate PathsFromPlace
	 * 
	 * @param path
	 */
	private void addToPathFromPlaces(PathProperty path) {
		boolean flag = false;
		for (int i = 0; i < pathsFromPlaces.size(); i++) {

			if (pathsFromPlaces.get(i).getPlaceID() == path.place1Id) {
				flag = true;
				pathsFromPlaces.get(i).add(path);
			}

		}
		if (!flag) {

			PathsFromPlace p = new PathsFromPlace((int) path.place1Id);
			p.add(path);
			pathsFromPlaces.add(p);
		}

	}

	private void generatePathsFromTimings() {
		for (Timings time : timings) {
			PathProperty p1 = new PathProperty(time.getPrev(), time
					.getStartPoint(), null, null, true);
			PathProperty p2 = new PathProperty(time.getStartPoint(), time
					.getEndPoint(), null, null, true);
			connectedPaths.add(new ConnectedPath(p2, p1));
		}

	}

	private void generateConnectedPaths() {

		println("begin generating connected paths");
		for (int i = 0; i < pathsToPlaces.size(); i++) {
			for (int j = 0; j < pathsFromPlaces.size(); j++) {

				if (pathsToPlaces.get(i).getPlaceID() == pathsFromPlaces.get(j)
						.getPlaceID()) {
					Vector<PathProperty> pTp = pathsToPlaces.get(i).getPaths();
					Vector<PathProperty> pFp = pathsFromPlaces.get(j)
							.getPaths();
					for (PathProperty p1 : pTp) {
						for (PathProperty p2 : pFp) {
							if (p1.place1Id != p2.place2Id) {
								connectedPaths.add(new ConnectedPath(p2, p1));
							}
						}
					}

				}
			}

		}
		timings = new Timings[connectedPaths.size()];
		for (int i = 0; i < connectedPaths.size(); i++) {
			timings[i] = new Timings(
					connectedPaths.get(i).getPrevPath().place1Id,
					connectedPaths.get(i).getPath().place1Id, connectedPaths
							.get(i).getPath().place2Id);
		}
		println("finish generating connected paths");
		println(connectedPaths.size() + " were found");

	}

	private int findNextPath() {

		// generate empty list of nodes to check
		ArrayList<Integer> visited = new ArrayList<Integer>();
		ArrayList<Integer> toVisit = new ArrayList<Integer>();
		ArrayList<Integer> storage = new ArrayList<Integer>();
		toVisit.add(new Integer(currentPos));
		int count = 0;
		while (!toVisit.isEmpty()) {

			for (Integer node : toVisit) {

				for (int i = 0; i < timings.length; i++) {

					// if the path we are looking at, starts at the node being
					// considered
					if (node == timings[i].getPrev()) {
						// need to look at the particular time here, with added
						// increments
						if (timings[i].getTimingIn(count * 10) == 0) {// if the
							// node
							// hasn't
							// been
							// visited
							// yet

							return i;
						} else {// if not, add to places to be considered next
							storage.add(new Integer((int) (timings[i]
									.getEndPoint())));
						}
					}

				}

			}

			for (Integer i : toVisit) {// move anything in toVisit into visited
				visited.add(i);
			}
			toVisit = new ArrayList<Integer>();
			// look through those nodes at the next depth level of search
			for (Integer stor : storage) {
				boolean seen = false;
				for (Integer visit : visited) {
					if (stor.equals(visit)) {
						seen = true;
					}
				}
				if (!seen) {
					toVisit.add(stor);
				}
			}

			count++;
		}

		return Integer.MAX_VALUE;
	}

	public void run() {

		if (ready) {
			println("running");
			timer.stop();
			timer.setActionCommand("save");
			timer.setDelay(5 * 60 * 1000);
			timer.restart();
			try {
				int num = timings.length;
				generatePathsFromTimings();
			} catch (NullPointerException e) {
				generateConnectedPaths();
			}
			save();
			goThroughPaths();

		}
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		if (e.getActionCommand().equals("begin")) {

			run();
		} else {
			if (e.getActionCommand().equals("save")) {
				save();
			}
		}

	}

	public void load() throws FileNotFoundException {

		try {
			ObjectInputStream in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream("timings.txt")));

			timings = ((TimingsWrapper) (in.readObject())).getTimings();
			println(timings);
			in.close();
		} catch (FileNotFoundException e) {
			println("unable to find file, load failed");
			throw e;

		} catch (IOException e) {
			// TODO Auto-generated catch block
			println(e);
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			println(e);
			e.printStackTrace();

		}
		println("load finished");
	}

	public void save() {
		ObjectOutputStream out;
		try {
			File file = new File("timings.txt");
			file.delete();
			out = new ObjectOutputStream(new BufferedOutputStream(
					new FileOutputStream("timings.txt")));
			TimingsWrapper wrap = new TimingsWrapper(timings);
			out.writeObject(wrap);
			out.close();
			println("saved");
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block

			e.printStackTrace();
		}

	}

}
