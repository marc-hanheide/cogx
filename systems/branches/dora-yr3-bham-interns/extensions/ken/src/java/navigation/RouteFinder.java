package navigation;

import java.util.Vector;

import exploration.PathTimes;


import NavData.FNode;

/**
 * class for finding the quickest path over a route
 * 
 * @author ken
 * 
 */
public class RouteFinder {

	private Vector<PathTimes> pathTimes;
	private Vector<FNode> nodes;

	public RouteFinder(Vector<PathTimes> pathTimes, Vector<FNode> nodes) {
		this.pathTimes = pathTimes;
		this.nodes=nodes;
	}

	/**
	 * converts the stored pathTimes into a Vector of paths each having the most
	 * likely time as their cost value
	 * 
	 * @return
	 */
	public Vector<Path> convertToSingleTime() {
		Vector<Path> returnV = new Vector<Path>();
		PathSelector p = new NaivePathSelector();
		for (PathTimes path : pathTimes) {
			returnV.add(new Path(path.getA(), path.getB(), (int) p
					.computeTime(path)));
		}
		return returnV;
	}

	public Vector<Integer> getRoute(int startPoint, int endPoint) {
		Vector<Path> paths = convertToSingleTime();
		return AStarSearch.search(nodes, paths, startPoint, endPoint);
	}

}
