/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package navigation;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.Vector;

import NavData.FNode;
import displays.GraphicalDisplay;
import exploration.PathTimes;
import exploration.PathTimesWrapper;

/**
 * class for performing A* search on a Vector of Paths
 * 
 * @author ken
 */
public class AStarSearch {

	public static void main(String[] args) {
		Vector<PathTimes> pathTimes = new Vector<PathTimes>();
		try {
			ObjectInputStream in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream("timings.txt")));

			pathTimes = ((PathTimesWrapper) (in.readObject())).getPathTimes();

			in.close();
			BufferedReader f = new BufferedReader(
					new FileReader("tmpmap.graph"));
			String first = f.readLine();
			int length = Integer.valueOf(String.valueOf(first.toCharArray(), 6,
					first.length() - 6));
			// System.out.println(length);
			Vector<FNode> nodes = new Vector<FNode>();

			while (nodes.size() < length) {
				nodes.add(util.misc.string2FNode(f.readLine()));

				// System.out.println("node " + nodes.lastElement().nodeId
				// + " pos " + nodes.lastElement().x + ", "
				// + nodes.lastElement().y);
			}
			// int[] pos = { 0, 51, 50, 25 };

			// for (int time = 600; time < 750; time += 50) {
			// AStarNode.setRadTurn(5000);

			int start = 25;
			int end = 39;
			int time = 650;

			Vector<Path> p = util.misc.convertToSingleTime(pathTimes,
					new NaivePathSelector(), time, false, 1);
			Vector<Path> p2 = util.misc.convertToSingleTime(pathTimes,
					new IntelligentPathSelector(), time, false, 1);
			// 710 and any other time, 5-->10
			// for (Path pa : p) {
			// System.out.println(pa.getA() + " to " + pa.getB()
			// + " costs " + pa.getCost());
			// }
			// System.out.println("graph is " + p);

			Vector<Integer> path = AStarSearch.search(nodes, p, start, end);
			Vector<Integer> path2 = AStarSearch.search(nodes, p2, start, end);

			System.out.println("naive path is " + eval(path, p2));
			 System.out.println(path);
			System.out.println("better path is " + eval(path2, p2));
			 System.out.println(path2);
//for(int i=0;i<p.size();i++){
//	System.out.println(p.get(i).getA()+ " & "+ p.get(i).getB());
//	System.out.println(p.get(i).getCost());
//	System.out.println(p2.get(i).getCost());
//	System.out.println();
//}
			 
			// }
		} catch (FileNotFoundException e) {
			System.out.println("unable to find file, load failed");

		} catch (IOException e) {
			System.out.println(e);
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			System.out.println(e);
			e.printStackTrace();

		}

	}

	public static int eval(Vector<Integer> path, Vector<Path> map) {

		int sum = 0;
		for (int i = 1; i < path.size(); i++) {
			for (int j = 0; j < map.size(); j++) {
				if ((path.get(i) == map.get(j).getA() && path.get(i - 1) == map
						.get(j).getB())
						|| (path.get(i - 1) == map.get(j).getA() && path.get(i) == map
								.get(j).getB())) {
					sum += map.get(j).getCost();
				}
			}

		}
		return sum;
	}

	public Path getPath(int a, int b, Vector<Path> paths) {
		for (Path path : paths) {
			if ((path.getA() == a && path.getB() == b)
					|| (path.getA() == b && path.getB() == a)) {
				return path;
			}
		}
		return null;
	}

	public static Vector<Integer> search(Vector<FNode> nodes,
			Vector<Path> paths, int startPoint, int endPoint) {
		System.out.println("performing search between " + startPoint + " & "
				+ endPoint);
		Vector<PathsFromNode> PFN = PathsFromNode.getPathsFromNodes(paths,
				nodes);
		Vector<Integer> nodesVisited = new Vector<Integer>();
		Vector<AStarNode> toVisit = new Vector<AStarNode>();
		Vector<AStarNode> pathsVisited = new Vector<AStarNode>();

		// System.out.println("net work is "+ paths);

		// nodesVisited.add(new Integer(startPoint));

		toVisit
				.add(new AStarNode(0, new Path(startPoint, startPoint, 0),
						nodes.get(startPoint), nodes.get(startPoint), nodes
								.get(startPoint)));

		// for (Path path : PFN.get(startPoint).getPaths()) {
		// if(path.getA()==startPoint){
		// toVisit.add(new AStarNode(0, path, nodes.get(path.getA()), nodes
		// .get(path.getB()), nodes.get(endPoint)));
		// }else{
		// toVisit.add(new AStarNode(0, path, nodes.get(path.getB()), nodes
		// .get(path.getA()), nodes.get(endPoint)));
		// }
		// }

		boolean found = false;
		while (!found) {
			System.out.println();
			// System.out.println("tovisit is currently "+toVisit.size());
			// for(AStarNode node:toVisit){
			// System.out.println(node.getPrevNode()+" "+node.getNode());
			// }
			// find next best pos
			double best = Double.MAX_VALUE;
			int bestPos = 0;
			for (int i = 0; i < toVisit.size(); i++) {

				if (toVisit.get(i).getHeuristic() < best) {
					bestPos = i;
					best = toVisit.get(i).getHeuristic();
				}

			}

			int pos = toVisit.get(bestPos).getNode();
			best = toVisit.get(bestPos).getCostToNode();// we want to use the
			// cost to that node, not it's heuristic
			// for further calculations
			System.out.println("position is " + pos);
			System.out
					.println("edge was " + toVisit.get(bestPos).getPrevNode());
			pathsVisited.add(toVisit.remove(bestPos));
			Vector<Path> temp = PFN.get(pos).getPaths();
			for (Path path : temp) {
				if (path.getA() == pos) {
					if (!nodesVisited.contains(new Integer(path.getB()))) {
						System.out.println("Visited " + nodesVisited);
						System.out.println("in b " + path.getA() + " "
								+ path.getB());
						toVisit.add(new AStarNode(best, path, nodes
								.get(pathsVisited.get(pathsVisited.size() - 1)
										.getPrevNode()),
								nodes.get(path.getA()), nodes.get(path.getB()),
								nodes.get(endPoint)));
						nodesVisited.add(new Integer(path.getA()));
						if (path.getB() == endPoint) {
							found = true;
							pathsVisited
									.add(toVisit.remove(toVisit.size() - 1));

						}
					}
				} else {
					if (path.getB() == pos) {
						if (!nodesVisited.contains(new Integer(path.getA()))) {
							System.out.println("Visited " + nodesVisited);
							System.out.println("in a " + path.getA() + " "
									+ path.getB());
							toVisit.add(new AStarNode(best, path, nodes
									.get(pathsVisited.get(
											pathsVisited.size() - 1)
											.getPrevNode()), nodes.get(path
									.getB()), nodes.get(path.getA()), nodes
									.get(endPoint)));
							nodesVisited.add(new Integer(path.getB()));
							if (path.getA() == endPoint) {
								found = true;
								pathsVisited.add(toVisit
										.remove(toVisit.size() - 1));

							}
						}
					}
				}
			}

		}

		for (AStarNode node : pathsVisited) {
			System.out.println("from " + node.getPrevNode() + " to "
					+ node.getNode());
			System.out.println("cost " + node.getCostToNode());

		}

		// go through pathsVisited to find each path
		int pos = endPoint;
		Vector<Integer> path = new Vector<Integer>();
		path.add(new Integer(endPoint));
		while (pos != startPoint) {
			for (AStarNode node : pathsVisited) {
				if (node.getNode() == pos) {
					pos = node.getPrevNode();
					path.add(0, new Integer(node.getPrevNode()));

				}
			}
		}

		return path;

	}

}
