package autoCostEstNew;

import java.util.Vector;

/**
 * class for finding tours of a network
 * @author ken
 *
 */
public class TourFinder {

	private Vector<PathTimes> standard;
	private Vector<Vector<PathTimes>> others;
	private int startPos;

	public TourFinder(Vector<PathTimes> pathTimes, int startPos) {
		this.startPos = startPos;
		standard = copy(pathTimes);
		others = new Vector<Vector<PathTimes>>();
		Vector<PathTimes> temp = copy(pathTimes);
		others.add(generateSimplePath(temp, startPos));
		temp = copy(pathTimes);
		others.add(generateRandomPath(temp, startPos));
	}

	/**
	 * copy the supplied vector
	 * 
	 * @param pathTimes
	 * @return
	 */
	public Vector<PathTimes> copy(Vector<PathTimes> pathTimes) {
		Vector<PathTimes> temp = new Vector<PathTimes>();
		for (PathTimes path : pathTimes) {
			temp.add(path);
		}
		return temp;

	}

	/**
	 * 
	 * @return the best of all the paths (i.e the one that is shortest)
	 */
	public Vector<PathTimes> getBestPath() {
		int value = evaluatePath(standard, startPos);
		int best = -1;
		int count = 0;
		for (Vector<PathTimes> pathTimes : others) {
			int current = evaluatePath(pathTimes, startPos);

			if (current < value) {
				
				best = count;
				value = current;
				
			}
			count++;
		}

		if (best == -1) {
			return standard;
		} else {
			return others.get(best);
		}

	}

	/**
	 * return all paths connected to the specified position
	 * 
	 * @param paths
	 * @param pos
	 * @return
	 */
	public Vector<PathTimes> getPathsFromNode(Vector<PathTimes> paths, int pos) {
		Vector<PathTimes> temp = new Vector<PathTimes>();
		for (PathTimes path : paths) {
			if ((path.getA() == pos) || (path.getB() == pos)) {
				temp.add(path);
			}
		}
		return temp;
	}

	/**
	 * returns a vector where each indice contains a vector listing every path
	 * to and from the node with the id of the indice
	 * 
	 * @param paths
	 * @return
	 */
	public Vector<Vector<PathTimes>> convertToNodes(Vector<PathTimes> paths) {
		Vector<Vector<PathTimes>> nodes = new Vector<Vector<PathTimes>>();
		for (int i = 0; i < paths.size(); i++) {
			nodes.add(getPathsFromNode(paths, i));
		}

		return nodes;
	}

	/**
	 * generates a very simple path from the provided Vector makes the path with
	 * as few detours as possible
	 * 
	 * @param pathTimes
	 * @param startPos
	 * @return
	 */
	public Vector<PathTimes> generateSimplePath(Vector<PathTimes> pathTimes,
			int startPos) {

		Vector<PathTimes> simple = new Vector<PathTimes>();

		Vector<Vector<PathTimes>> pathFromNodes = convertToNodes(pathTimes);

		Vector<Integer> toVisit = new Vector<Integer>();
		Vector<Integer> visited = new Vector<Integer>();
		Vector<Integer> temp = new Vector<Integer>();
		toVisit.add(new Integer(startPos));
		int count = 0;
		int foundNum = -1;

		while ((simple.size() < pathTimes.size()) && count < 100) {

			// for every node we are currently looking from
			for (Integer node : toVisit) {

				if (foundNum != -1) {
					break;
				}
				// check all the paths from that node
				for (PathTimes path : pathFromNodes.get(node)) {

					if (foundNum != -1) {
						break;
					}
					// if we haven't already added this to our path, then add it
					if (!simple.contains(path)) {
						simple.add(path);
						if (path.getA() == node) {

							foundNum = path.getB();

						} else {// so path.getB() = node

							foundNum = path.getA();

						}

						break;
					} else {// if we've already seen this path, add the node it
						// connects to
						if (path.getA() == node) {

							temp.add(path.getB());

						} else {// so path.getB() = node

							temp.add(path.getA());

						}
					}
				}

			}
			if (foundNum != -1) {// we found another path in this iteration
				toVisit.clear();
				visited.clear();
				temp.clear();
				toVisit.add(foundNum);
				foundNum = -1;
			} else {// we still need to look for another path

				// check through each node visited and add any new ones to it
				for (Integer seen : toVisit) {

					if (!simple.contains(seen)) {
						visited.add(seen);
					}
				}

				toVisit.clear();
				// check through each node connected to a seen node
				for (Integer node : temp) {

					if (!visited.contains(node)) {
						// if it's new, look at it in the next cycle
						toVisit.add(node);
					}
				}

				temp.clear();

			}
			count++;
		}

		return simple;
	}

	public Vector<PathTimes> generateRandomPath(Vector<PathTimes> pathTimes,
			int startPos) {
		Vector<PathTimes> temp = new Vector<PathTimes>();
		while (pathTimes.size() > 0) {
			double ran = Math.random();
			int pos = (int) (pathTimes.size() * ran);
			temp.add(pathTimes.remove(pos));
		}
		return temp;
	}

	/**
	 * states how many extra paths need to be run to go through the given path
	 * 
	 * @return
	 */
	public int evaluatePath(Vector<PathTimes> pathTimes, int startPos) {
		int cost = 0;
		for (PathTimes path : pathTimes) {
			cost += distFromPaths(pathTimes, startPos, path.getA());
			startPos = path.getB();
		}

		return cost;
	}

	/**
	 * returns the number of nodes traversed through the list of path times from
	 * the start position till the end one
	 * 
	 * @param pathTimes
	 * @param startPos
	 * @param endPos
	 * @return
	 */
	public static int distFromPaths(Vector<PathTimes> pathTimes, int startPos,
			int endPos) {
		Vector<Integer> visited = new Vector<Integer>();
		Vector<Integer> toVisit = new Vector<Integer>();
		Vector<Integer> temp = new Vector<Integer>();
		toVisit.add(new Integer(startPos));
		int cost = 0;
		while (!toVisit.contains(endPos)) {
			for (PathTimes path : pathTimes) {
				if (toVisit.contains(path.getA())) {

					if (!visited.contains(path.getB())) {
						temp.add(path.getB());
					}

				} else {
					if (toVisit.contains(path.getB())) {

						if (!visited.contains(path.getA())) {
							temp.add(path.getA());
						}

					}
				}
			}
			toVisit.clear();
			for (Integer in : temp) {
				toVisit.add(in);
			}
			temp.clear();
			cost++;
		}

		return cost;
	}
}
