package autoCostEst;

import java.util.Vector;

import SpatialProperties.PathProperty;

/**
 * method of storing all the paths ending at a given placeID
 * 
 * @author ken
 * 
 */
public class PathsToPlace {

	private int placeID;
	private Vector<PathProperty> paths;

	/**
	 * create a new PathsToPlace with all nodes starting at a given place
	 * 
	 * @param placeid
	 */
	public PathsToPlace(int placeID) {
		this.placeID = placeID;
		paths = new Vector<PathProperty>();
	}

	public void add(PathProperty path) {
		assert(path.place2Id==placeID);
		paths.add(path);
	}

	public Vector<PathProperty> getPaths() {
		return paths;
	}

	public int getPlaceID() {
		return placeID;
	}
}
