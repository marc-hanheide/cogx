package manipulation.core.share.baseConnector;

import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;

/**
 * represents a connection to a robot base
 * 
 * @author Torben Toeniges
 * 
 */
public interface BaseConnector {
	/**
	 * sets the current position of the robot
	 * 
	 * @param position
	 *            place where the robot is at the moment
	 */
	public void setCurrentPosition(BasePositionData position);

	/**
	 * gets the current position of the robot
	 * 
	 * @return current position of the robot
	 * @throws ExternalMemoryException
	 */
	public BasePositionData getCurrentPosition() throws ExternalMemoryException;

	/**
	 * convert a relative point in relation to the robot in world coordinates
	 * 
	 * @param relativePoint
	 *            relative point in relation to the robot
	 * @return world coordinates value of the relative point
	 */
	//public Vector3D getRobotToWorldTranslation(Vector3D relativePoint);

	/**
	 * convert the relative rotation in relation to the robot in world
	 * coordinates
	 * 
	 * @param rotation
	 *            relative rotation to the robot
	 * @return world coordinates value of the relative rotation
	 */
	//public Matrix getRobotToWorldRotation(Matrix rotation);
}
