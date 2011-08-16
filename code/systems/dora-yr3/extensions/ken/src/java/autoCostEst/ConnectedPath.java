package autoCostEst;

import SpatialProperties.PathProperty;

/**
 * represents a path, as well as the previous one travelled to reach it's start
 * node
 * 
 * @author ken
 * 
 */
public class ConnectedPath {
	private PathProperty path;
	private PathProperty prevPath;
	private int time;

	/**
	 * creates a new ExtenedPath with the two given parameters
	 * 
	 * @param path
	 * @param prevPath
	 */
	public ConnectedPath(PathProperty path, PathProperty prevPath) {
		this.path = path;
		this.prevPath = prevPath;

	}

	/**
	 * returns the main path of this object
	 * 
	 * @return
	 */
	public PathProperty getPath() {
		return path;

	}

	/**
	 * returns the previous path, i.e one connecting to the main path's start
	 * point
	 * 
	 * @return
	 */
	public PathProperty getPrevPath() {
		return prevPath;
	}

	public void setTime(int time) {
		this.time = time;
	}

	public int getTime() {
		return time;

	}
}
