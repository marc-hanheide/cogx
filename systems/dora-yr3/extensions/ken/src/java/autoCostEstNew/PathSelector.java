package autoCostEstNew;

/**
 * class to be given PathTimes and objects, and return a single time
 * class is abstract so that many different methods can be selected from
 * @author ken
 *
 */
public abstract class PathSelector {

	/**
	 * given a pathTime, return the time this class believes should be computed
	 * for the current time
	 * @param pathTime
	 * @return
	 */
	public abstract long computeTime(PathTimes pathTime);
	
}
