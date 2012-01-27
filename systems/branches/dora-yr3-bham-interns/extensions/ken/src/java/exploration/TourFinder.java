package exploration;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.Vector;

import gA.BlockedGeneticAlgorithm;

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
		// System.out.println("Path times");
		// int index = 0;
		// for (PathTimes pT : pathTimes) {
		// System.out.println("num " + index + " " + pT.getA() + " "
		// + pT.getB());
		// index++;
		// }
		TourFinder tF = new TourFinder(pathTimes, 0);
		Vector<PathTimes> pTa = tF.getBestPath(0);
		Vector<PathTimes> blocked = new Vector<PathTimes>();
		Vector<PathTimes> visited = new Vector<PathTimes>();
		visited.add(pathTimes.get(0));
		visited.add(pathTimes.get(1));
		visited.add(pathTimes.get(2));
		visited.add(pathTimes.get(3));
		visited.add(pathTimes.get(4));
		pathTimes.remove(4);
		System.out.println("moo " + blocked);
		Vector<PathTimes> pT = tF.generatePath(pathTimes, blocked, visited, 0);

		for (int i = 0; i < pT.size(); i++) {
			System.out.println(pT.get(i).getA() + " " + pT.get(i).getB() + " "
					+ pTa.get(i).getA() + " " + pTa.get(i).getB() + " ");
			System.out.println(pT.get(i).getA() == pTa.get(i).getA());
			System.out.println(pT.get(i).getB() == pTa.get(i).getB());
		}
		System.out.println("old version " + evaluatePath(pTa, 0));
		System.out
				.println("old version " + evaluateIncompletePath(pTa, pTa, 0));

		System.out.println("new version " + evaluatePath(pT, 0));
		System.out.println("new version " + evaluateIncompletePath(pT, pT, 0));
		System.out.println(pT.size() + " " + pTa.size());

	}

	public TourFinder(Vector<PathTimes> pathTimes, int startPos) {
		this.startPos = startPos;
		standard = copy(pathTimes);
		others = new Vector<Vector<PathTimes>>();
		// Vector<PathTimes> temp = copy(pathTimes);

		Vector<PathTimes> travelled = new Vector<PathTimes>();
		Vector<PathTimes> simple = generateSimplePath(pathTimes, startPos);
		others.add(simple);
		BlockedGeneticAlgorithm GA = new BlockedGeneticAlgorithm(copy(simple),
				travelled, 200, 60, startPos);
		others.add(GA.getBest());

	}

	/**
	 * copy the supplied vector
	 * 
	 * @param pathTimes
	 * @return
	 */
	public static Vector<PathTimes> copy(Vector<PathTimes> pathTimes) {
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
	public Vector<PathTimes> getBestPath(int pos) {

		Vector<PathTimes> bestPath = standard;
		int best = evaluatePath(standard, pos);
		System.out.println("others size is " + others.size());
		for (Vector<PathTimes> path : others) {
			int current = evaluatePath(path, pos);

			if (current < best) {
				System.out.println("new best found");
				System.out.println("old best " + best + " & new best is "
						+ current);
				best = current;
				bestPath = path;

			}
		}

		return bestPath;
	}

	/**
	 * return all paths connected to the specified position
	 * 
	 * @param paths
	 * @param pos
	 * @return
	 */
	public static Vector<PathTimes> getPathsFromNode(Vector<PathTimes> paths,
			int pos) {
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
	public static Vector<Vector<PathTimes>> convertToNodes(
			Vector<PathTimes> paths) {
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
		System.out.println("random path generated");
		return temp;
	}

	/**
	 * states how many extra paths need to be run to go through the given path
	 * 
	 * @return
	 */
	public static int evaluatePath(Vector<PathTimes> pathTimes, int startPos) {

		int cost = 0;
		for (PathTimes path : pathTimes) {
			if (startPos == path.getA()) {
				cost += distFromPaths(pathTimes, startPos, path.getA());
				startPos = path.getB();
			} else {
				if (startPos == path.getB()) {
					cost += distFromPaths(pathTimes, startPos, path.getB());
					startPos = path.getA();
				} else {
					int distA = distFromPaths(pathTimes, startPos, path.getA());
					int distB = distFromPaths(pathTimes, startPos, path.getB());
					if (distA < distB) {
						cost += distA;
						startPos = path.getB();
					} else {
						cost += distB;
						startPos = path.getA();
					}
				}
			}

		}

		return cost;
	}

	/**
	 * states how many extra paths need to be run to go through the given path
	 * 
	 * @return
	 */
	public static int evaluateIncompletePath(Vector<PathTimes> paths,
			Vector<PathTimes> wholeGraph, int startPos) {
		
		int cost = 0;
		for (PathTimes path : paths) {

			if (startPos == path.getA()) {
				cost += distFromPaths(wholeGraph, startPos, path.getA());
				startPos = path.getB();
			} else {
				if (startPos == path.getB()) {
					cost += distFromPaths(wholeGraph, startPos, path.getB());
					startPos = path.getA();
				} else {
					int distA = distFromPaths(wholeGraph, startPos, path.getA());
					int distB = distFromPaths(wholeGraph, startPos, path.getB());
					if (distA < distB) {
						cost += distA;
						startPos = path.getB();
					} else {
						cost += distB;
						startPos = path.getA();
					}
				}
			}
			
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
	 * returns the number of nodes traversed through the list of path times from
	 * the start position till the end one
	 * 
	 * @param pathTimes
	 * @param startPos
	 * @param endPos
	 * @return
	 */
	public static int distFromPaths(Vector<PathTimes> pathTimes,
			Vector<PathTimes> blocked, int startPos, int endPos) {
		Vector<Integer> visited = new Vector<Integer>();
		Vector<Integer> toVisit = new Vector<Integer>();
		Vector<Integer> temp = new Vector<Integer>();
		Vector<PathTimes> pathTimesC = copy(pathTimes);
		for (PathTimes pT : blocked) {
			pathTimesC.remove(pT);
		}
		toVisit.add(new Integer(startPos));
		int cost = 0;
		while (!toVisit.contains(endPos)) {
			for (PathTimes path : pathTimesC) {
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

	public Vector<PathTimes> generatePath(Vector<PathTimes> pathTimes,
			Vector<PathTimes> blocked, Vector<PathTimes> travelled, int startPos) {

		Vector<PathTimes> temp = generateSimplePath(pathTimes, blocked,
				travelled, startPos);

		BlockedGeneticAlgorithm gA = new BlockedGeneticAlgorithm(copy(temp),
				travelled, 200, 60, startPos);
		Vector<PathTimes> evolved = gA.getBest();
		int gACost = evaluatePath(evolved, startPos);
		int otherCost = evaluatePath(temp, startPos);
		System.out.println("GA cost is " + gACost);
		System.out.println("other cost is " + otherCost);
		if (otherCost > gACost) {
			System.out.println("evolved");
			return evolved;
		} else {
			return temp;
		}

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
			if (toVisit.isEmpty()) {// there is no where left to look

				System.out.println("incomplete path has been generated");
				return simple;
			}
			count++;

		}

		return simple;
	}

	/**
	 * given a start and end point, will look over a provided list of paths for
	 * a route ignores any paths that are listed as blocked this is not the
	 * fastest route time wise, but the first one found
	 * 
	 * @param from
	 * @param to
	 * @param pathTimes
	 * @param blocked
	 * @return
	 */
	public static Vector<PathTimes> route(int from, int to,
			Vector<PathTimes> pathTimes, Vector<PathTimes> blocked) {
		Vector<Integer> toVisit = new Vector<Integer>();
		Vector<Integer> visitedNode = new Vector<Integer>();
		Vector<PathTimes> visited = new Vector<PathTimes>();
		Vector<PathTimes> pathTimesC = copy(pathTimes);
		Vector<Integer> temp = new Vector<Integer>();
		for (PathTimes bloc : blocked) {
			pathTimesC.remove(bloc);
		}
		Vector<Vector<PathTimes>> pathFromNodes = convertToNodes(pathTimesC);
		boolean found = false;
		toVisit.add(from);

		while (!found) {

			for (Integer node : toVisit) {

				if (found) {
					break;
				}
				if (!visitedNode.contains(node)) {
					visitedNode.add(node);
				}
				for (PathTimes path : pathFromNodes.get(node)) {
					if (found) {
						break;
					}
					if (!visited.contains(path)) {

						if (!visitedNode.contains(path.getA())) {
							temp.add(path.getA());
							visited.add(path);
							if (path.getA() == to) {
								found = true;

							}
						}

						if (!visitedNode.contains(path.getB())) {
							temp.add(path.getB());
							visited.add(path);
							if (path.getB() == to) {
								found = true;

							}
						}

					}
				}

			}
			toVisit.clear();
			for (Integer node : temp) {
				if (!visitedNode.contains(node)) {
					toVisit.add(node);
				}
			}

		}

		Vector<PathTimes> route = new Vector<PathTimes>();
		int pos = to;
		while (pos != from) {
			for (PathTimes p : visited) {
				if (p.getA() == pos) {
					pos = p.getB();
					route.insertElementAt(p, 0);
					break;
				}
				if (p.getB() == pos) {
					pos = p.getA();
					route.insertElementAt(p, 0);
					break;
				}

			}
		}

		return route;
	}
}
