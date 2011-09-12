package navigation;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.FileNotFoundException;
import java.util.Vector;

import javax.swing.Timer;

import util.CastComponent;
import NavData.FNode;
import SpatialData.NavCommand;
import cast.CASTException;

/**
 * class that will travel the fastest route over a network
 * 
 * @author ken
 * 
 */
public class RouteTraveller extends CastComponent implements ActionListener {

	private int dest;
	
	/**
	 * creates a new Route traveller heading to a pre-set destination
	 */
	public RouteTraveller() {
		this(13);

	}

	/**
	 * creates a new Route traveller heading to a specified destination
	 * @param dest
	 */
	public RouteTraveller(int dest) {
		nodes = new Vector<FNode>();
		start = true;
		this.dest = dest;
	}

	/**
	 * loads the pathTimes as well as sets the memory filters
	 * and starts the timer
	 */
	@Override
	public void start() {
		try {
			load();
			addFNodeFilter();
			addPoseFilter();
			timer = new Timer(10000, this);

		} catch (FileNotFoundException e) {
			println("Could not find file");
			println(e);
			println(e.getStackTrace());
			println("exiting");
			System.exit(0);
		}

	}


	/**
	 * this is what is run once the timer has finished
	 * this means that the paths+nodes have been compiled from working memory
	 * now it is time to run around the graph
	 */
	@Override
	public void actionPerformed(ActionEvent arg0) {
		println("timer finished");
		timer.stop();
		RouteFinder p = new RouteFinder(pathTimes, nodes);

		int closest = getClosestNode(x, y, nodes);
		goTo(closest);

		println("about to do turn to measure time taken");
		long time = System.currentTimeMillis();
		turn(1);
		AStarNode.setRadTurn(System.currentTimeMillis() - time);

		println("about to search routes");
		Vector<Integer> route = p.getRoute(closest, dest);
		println("route is: ");
		for (Integer i : route) {
			println(i);
		}
		runRoute(route);
		println("destination reached");
	}

	/**
	 * given a route (composed of a vector of Integers),
	 *  run through it
	 * @param route
	 */
	private void runRoute(Vector<Integer> route) {
		for (Integer node : route) {
			goTo(node.intValue());
		}

	}

	/**
	 * turn the specified amount (in radians)
	 * @param amt
	 */
	public void turn(double amt) {
		NavCommand cmd = generateTurn(amt);
		try {
			executeNavCommand(cmd);
		} catch (CASTException e) {
			println("problem turning");
			println(e);
			e.printStackTrace();
		} catch (InterruptedException e) {
			println("problem turning");
			println(e);
			e.printStackTrace();
		}
	}



}
