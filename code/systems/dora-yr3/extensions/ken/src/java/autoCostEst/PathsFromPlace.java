package autoCostEst;

import java.util.Vector;

import SpatialProperties.PathProperty;


/**
 * method of storing all paths starting at a given placeID
 * @author ken
 *
 */
public class PathsFromPlace {
	private int placeID;
	private Vector<PathProperty> paths;

	/**
	 * create a new PathsFromPlace with all nodes starting at a given place
	 * 
	 * @param placeid
	 */
	public PathsFromPlace(int placeID) {
		this.placeID = placeID;
		paths = new Vector<PathProperty>();
	}

	public void add(PathProperty path) {
		assert(path.place1Id==placeID);
		paths.add(path);
	}

	public Vector<PathProperty> getPaths() {
		return paths;
	}

	public int getPlaceID() {
		return placeID;
	}
}
