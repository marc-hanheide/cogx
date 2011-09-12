package exploration;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.Vector;

/**
 * class for finding tours of a network
 * 
 * @author ken
 * 
 */
public class TourFinder {

	private Vector<PathTimes> standard;
	private Vector<Vector<PathTimes>> others;
	private int startPos;

	public static void main(String[] args) {
		Vector<PathTimes> pathTimes = new Vector<PathTimes>();
		try {
			ObjectInputStream in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream("timings.txt")));

			pathTimes = ((PathTimesWrapper) (in.readObject())).getPathTimes();

			in.close();
		} catch (FileNotFoundException e) {
			System.out.println("unable to find file, load failed");

		} catch (IOException e) {
			System.out.println(e);
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			System.out.println(e);
			e.printStackTrace();

		}
		System.out.println("Path times");
		int index = 0;
		for (PathTimes pT : pathTimes) {
			System.out.println("num " + index + " " + pT.getA() + " "
					+ pT.getB());
			index++;
		}
		TourFinder tF = new TourFinder(pathTimes, 0);
		// Vector<PathTimes> times = tF.getBestPath();
		Vector<PathTimes> travelled = new Vector<PathTimes>();
		Vector<PathTimes> blocked = new Vector<PathTimes>();
		blocked.add(pathTimes.get(10));
		blocked.add(pathTimes.get(17));
		blocked.add(pathTimes.get(42));
		for (int i = 0; i < 10; i++) {
			travelled.add(pathTimes.get(i));
		}

		Vector<PathTimes> times = tF.generateSimplePath(pathTimes, blocked,
				travelled, 0);
		System.out.println("times " + times.size());
		System.out.println("pathTimes " + pathTimes.size());
		for (PathTimes pT : times) {
			System.out.println(pT.getA() + " " + pT.getB());

		}

	}

	public TourFinder(Vector<PathTimes> pathTimes, int startPos) {
		this.startPos = startPos;
		standard = copy(pathTimes);
		others = new Vector<Vector<PathTimes>>();
		Vector<PathTimes> temp = copy(pathTimes);
		others.add(generateSimplePath(temp, startPos));
		// temp = copy(pathTimes);
		// others.add(generateRandomPath(temp, startPos));
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
	 * @return the best of all the paths (i.e the one that is shortest) there is
	 *         an error somewhere, so I'm just returning the computed one
	 */
	public Vector<PathTimes> getBestPath() {

		return others.get(0);
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

		while ((simple.size() < pathTimes.size())) {

			// foundNum=-1;
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
		// System.out.println("path times " + pathTimes.size());
		// System.out.println("simple " + simple.size());

		// for (PathTimes pT : simple) {
		// System.out.println(pT.getA() + " " + pT.getB());
		//
		// }
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

	/**
	 * generates a very simple path from the provided Vector makes the path with
	 * as few detours as possible
	 * 
	 * the same as the other generate simplePath, however it will find a route
	 * during run time, after some routes have been run and some are found to be
	 * blocked
	 * 
	 * @param pathTimes
	 * @param startPos
	 * @return
	 */
	public Vector<PathTimes> generateSimplePath(Vector<PathTimes> pathTimes,
			Vector<PathTimes> blocked, Vector<PathTimes> travelled, int startPos) {
		Vector<PathTimes> pathTimesC = copy(pathTimes);
		Vector<PathTimes> simple = new Vector<PathTimes>();
		for (PathTimes p : blocked) {// scrub where we can look of any blocked
			// nodes
			pathTimesC.remove(p);
		}

		Vector<Vector<PathTimes>> pathFromNodes = convertToNodes(pathTimesC);

		Vector<Integer> toVisit = new Vector<Integer>();
		Vector<Integer> visited = new Vector<Integer>();
		Vector<Integer> temp = new Vector<Integer>();
		toVisit.add(new Integer(startPos));

		int count = 0;
		int foundNum = -1;

		while ((simple.size() < pathTimesC.size())) {

			// foundNum=-1;
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
					if (!simple.contains(path) && !travelled.contains(path)) {
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
			if ( toVisit.isEmpty()) {//there is no where left to look
				
				System.out.println("incomplete path has been generated");
				return simple;
			}
			count++;

		}
		// System.out.println("path times " + pathTimes.size());
		// System.out.println("simple " + simple.size());

		// for (PathTimes pT : simple) {
		// System.out.println(pT.getA() + " " + pT.getB());
		//
		// }
		return simple;
	}

}
