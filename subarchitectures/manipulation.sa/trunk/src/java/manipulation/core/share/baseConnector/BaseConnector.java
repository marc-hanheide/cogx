package manipulation.core.share.baseConnector;

import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;

/**
 * represents a connection to a robot base
 * 
 * @author ttoenige
 * 
 */
public interface BaseConnector {
	/**
	 * represent different goal reaching success possibilities
	 * 
	 * @author ttoenige
	 * 
	 */
	public enum GoalReachingSuccess {
		/**
		 * name of the status if the robot successful reached the goal
		 */
		GOAL_REACHED,
		/**
		 * name of the status if the goal is unreachable
		 */
		GOAL_UNREACHABLE,
		// TODO was ist das. not waiting?!?!?!
		/**
		 * name of the status if is not waiting?!
		 */
		NOT_WAITING,
		/**
		 * all other possibilities
		 */
		OTHER
	}

	/**
	 * the robot goes to a specifies position
	 * 
	 * @param target
	 *            position to go to
	 * @throws ExternalMemoryException
	 */
	public void goTo(BasePositionData target) throws ExternalMemoryException;

	/**
	 * the robot starts exploring the area
	 */
	public void startExplore();

	/**
	 * the robot stop his current movement
	 */
	public void stop();

	/**
	 * gets the weather the robot is exploring the area
	 * 
	 * @return <code>true</code> if the robot is exploring, <code>false</code>
	 *         if the robot is not exploring the area
	 */
	public boolean isExploring();

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
	 * gets weather the robot is moving
	 * 
	 * @return <code>true</code> if the robot is moving, <code>false</code> if
	 *         the robot is not moving
	 */
	public boolean isMoving();

	/**
	 * sets the moving value
	 * 
	 * @param moving
	 *            <code>true</code> if the robot is moving, <code>false</code>
	 *            if the robot is not movin
	 */
	public void setMoving(boolean moving);

	/**
	 * convert a relative point in relation to the robot in world coordinates
	 * 
	 * @param relativePoint
	 *            relative point in relation to the robot
	 * @return world coordinates value of the relative point
	 */
	public Vector3D getRobotToWorldTranslation(Vector3D relativePoint);

	/**
	 * convert the relative rotation in relation to the robot in world
	 * coordinates
	 * 
	 * @param rotation
	 *            relative rotation to the robot
	 * @return world coordinates value of the relative rotation
	 */
	public Matrix getRobotToWorldRotation(Matrix rotation);
}
