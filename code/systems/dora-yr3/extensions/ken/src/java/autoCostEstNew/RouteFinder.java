package autoCostEstNew;

import java.util.Vector;

/**
 * class for finding the quickest path over a route
 * 
 * @author ken
 * 
 */
public class RouteFinder {

	private Vector<PathTimes> pathTimes;

	public RouteFinder(Vector<PathTimes> pathTimes) {
		this.pathTimes = pathTimes;
	}

	/**
	 * converts the stored pathTimes into a Vector of integers
	 * 
	 * @return
	 */
	public Vector<Integer> convertToSingleTime() {
		Vector<Integer> returnV = new Vector<Integer>();
		PathSelector p = new NaivePathSelector();
		for (PathTimes path : pathTimes) {
			returnV.add(new Integer((int) p.computeTime(path)));
		}
		return returnV;
	}

	public Vector<Integer> getRoute(int start, int end) {
		// TODO Auto-generated method stub
		return null;
	}

}
