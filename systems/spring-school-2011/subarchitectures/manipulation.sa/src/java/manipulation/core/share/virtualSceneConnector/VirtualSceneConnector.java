package manipulation.core.share.virtualSceneConnector;


/**
 * represents a connector to a virtual scene
 * 
 * @author Torben Toeniges
 * 
 */
public interface VirtualSceneConnector {
	/**
	 * gets the time in the virtual scene (used to define goals of the arm)
	 * 
	 * @return time in the virtual scene
	 */
	public double getTime();

	/**
	 * deletes all elements in the scene
	 */
	public void clearScene();

	/**
	 * update the arm in the virtual scene
	 */
	public void updateArm();

}
