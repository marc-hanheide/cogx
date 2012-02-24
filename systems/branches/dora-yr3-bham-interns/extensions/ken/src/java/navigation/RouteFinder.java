package navigation;

import java.util.Date;
import java.util.Vector;

import NavData.FNode;
import exploration.PathRun;
import exploration.PathTimes;

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
		this.nodes = nodes;
	}

//	public static void main(String[] args) {
//		Vector<FNode> nodes = new Vector<FNode>();
//		Vector<PathTimes> path = new Vector<PathTimes>();
//
//		nodes.add(node(0, 2, 2));
//		nodes.add(node(1, 1, 1));
//		nodes.add(node(2, 3, 1));
//		nodes.add(node(3, 1, 4));
//		nodes.add(node(4, 1, 7));
//		nodes.add(node(5, 2, 9));
//		nodes.add(node(6, 3, 6));
//		nodes.add(node(7, 4, 11));
//		nodes.add(node(8, 0, 5));
//		nodes.add(node(9, 2, 9));
//		nodes.add(node(10, 0, 9));
//		nodes.add(node(11, 2, 11));
//		nodes.add(node(12, 0, 12));
//		nodes.add(node(13, 3, 4));
//
//		path.add(path(0, 1, 4*1400));
//		path.add(path(0, 2, 4*1400));
//		path.add(path(0, 3, 4*2230));
//		path.add(path(1, 2, 4*2000));
//		path.add(path(1, 3, 4*3000));
//		path.add(path(2, 13, 4*3000));
//		path.add(path(3, 13, 4*2000));
//		path.add(path(13, 6, 4*2000));
//		path.add(path(6, 5, 4*3100));
//		path.add(path(11, 5, 4*2000));
//		path.add(path(8, 3, 4*1400));
//		path.add(path(8, 4, 4*2230));
//		path.add(path(4, 3, 4*3000));
//		path.add(path(5, 4, 4*2230));
//		path.add(path(4, 10, 4*22300));
//		path.add(path(9, 11, 4*2000));
//		path.add(path(12, 11, 4*2820));
//		path.add(path(10, 12, 4*3000));
//		path.add(path(5, 7, 4*2820));
//
//		RouteFinder f = new RouteFinder(path, nodes);
//
//		System.out.println(f.getRoute(0, 12));
//	}

	public static FNode node(long id, double x, double y) {
		FNode f = new FNode();
		f.x = x;
		f.y = y;
		f.nodeId = id;
		return f;
	}

	public static PathTimes path(int a, int b, int cost) {
		PathTimes p = new PathTimes(a, b);
		PathRun pR = new PathRun(new Date(System.currentTimeMillis()));
		pR.setTimeTaken(new Date(System.currentTimeMillis() + cost));
		p.add(pR);
		return p;
	}

	

	public Vector<Integer> getRoute(int startPoint, int endPoint) {
		
		Vector<Path> paths = util.misc.convertToSingleTime(pathTimes, new NaivePathSelector(),0);
		
		return AStarSearch.search(nodes, paths, startPoint, endPoint);
	}

}
