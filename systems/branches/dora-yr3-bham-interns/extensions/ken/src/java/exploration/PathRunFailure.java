package exploration;

import java.util.Vector;

/**
 * class used to pass visited and blocked paths in the event of a run failure
 * @author kenslaptop
 *
 */
public class PathRunFailure extends Exception {
	private Vector<PathTimes> visited;
	private Vector<PathTimes> blocked;

	
	public PathRunFailure(Vector<PathTimes> visited, Vector<PathTimes> blocked,
			String msg) {
		super(msg);
		this.visited = visited;
		this.blocked = blocked;

	}

	public Vector<PathTimes> getVisited() {
		return visited;
	}

	public Vector<PathTimes> getBlocked() {
		return blocked;
	}

}
