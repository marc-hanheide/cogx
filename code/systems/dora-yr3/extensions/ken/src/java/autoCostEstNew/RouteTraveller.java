package autoCostEstNew;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.Vector;

import javax.swing.Timer;

import NavData.FNode;
import SpatialData.Completion;
import SpatialData.NavCommand;
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
 * class that will travel the fastest route over a network
 * 
 * @author ken
 * 
 */
public class RouteTraveller extends ManagedComponent implements ActionListener {

	private Vector<PathTimes> pathTimes;
	private Vector<FNode> nodes;
	private Timer timer;
	private int start;
	private int end;

	@Override
	public void runComponent() {
		println("traveller is running");
	}

	public RouteTraveller() {
		this(0, 30);
	}

	public RouteTraveller(int start, int end) {

		this.start = start;
		this.end = end;
	}

	@Override
	public void start() {
		try {
			load();
			addFNodeFilter();
			timer = new Timer(2000, this);

		} catch (FileNotFoundException e) {
			println("Could not find file");
			println(e);
			println(e.getStackTrace());
			println("exiting");
			System.exit(0);
		}

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
	 * add an FNode filter this will look at working memory and report if any
	 * FNodes are added
	 */
	public void addFNodeFilter() {
		println("change filter added");
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(FNode.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				println("change seen");
				try {
					FNode f = getMemoryEntry(_wmc.address, FNode.class);
					nodes.add(f);

					timer.restart();
					println("end change");
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
	private void load() throws FileNotFoundException {
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

		println("load finished");

	}

	@Override
	public void actionPerformed(ActionEvent arg0) {
		println("timer finished");
		timer.stop();
		RouteFinder p = new RouteFinder(pathTimes);
		Vector<Integer> route = p.getRoute(start, end);
		println("route is: ");
		for (Integer i : route) {
			println(i);
		}
		runRoute(route);
	}

	private void runRoute(Vector<Integer> route) {
		for (Integer node : route) {
			goTo(node);
		}

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

	private void goTo(Integer node) {
		try {
			NavCommand cmd = GraphExplorer.createNavCommand(node);
			println("heading to " + node);
			executeNavCommand(cmd);
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

}
