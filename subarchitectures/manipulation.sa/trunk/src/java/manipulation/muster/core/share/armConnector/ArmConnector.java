package manipulation.muster.core.share.armConnector;

import java.util.HashMap;

import manipulation.muster.core.share.exceptions.ManipulatorException;
import manipulation.muster.core.share.types.ArmError;
import manipulation.muster.core.share.types.Matrix;
import manipulation.muster.core.share.types.Vector3D;
import manipulation.muster.core.share.types.SensorData.SensorPosition;

/**
 * represents a connection to an arm hardware
 * 
 * @author ttoenige
 * 
 */
public interface ArmConnector {
	public enum ArmName {
		/**
		 * running in simulation
		 */
		SIMULATION,
		/**
		 * using the katana 300
		 */
		KATANA300,
		/**
		 * using the katana 450
		 */
		KATANA450
	}	
	
	/**
	 * reaches a position with the arm
	 * 
	 * @param position
	 *            position to reach
	 * @param rotation
	 *            rotation to reach
	 * @throws ManipulatorException
	 */
	public void reach(Vector3D position, Matrix rotation)
			throws ManipulatorException;

	/**
	 * sends the arm to its home position
	 * 
	 * @throws ManipulatorException
	 */
	public void goHome() throws ManipulatorException;

	/**
	 * stops the arm
	 * 
	 * @throws ManipulatorException
	 */
	public void stopArm() throws ManipulatorException;

	/**
	 * returns if the goal position is reached or not
	 * 
	 * @return <code>true</code> if the goal position is reached,
	 *         <code>false</code> if the goal position is not reached
	 */
	public boolean isReached();

	/**
	 * sets the reaching value of the arm
	 * 
	 * @param reached
	 *            <code>true</code> if the goal is reached, <code>false</code>
	 *            if the goal is not reached
	 */
	public void setReached(boolean reached);

	/**
	 * opens the gripper
	 * 
	 * @throws ManipulatorException
	 */
	public void openGripper();

	/**
	 * closes the gripper with a given force (small values light, high value
	 * strong)
	 * 
	 * @param force
	 *            force value (small values light, high value strong)
	 * @throws ManipulatorException
	 */
	public void closeGripper(int force);

	/**
	 * checks weather the gripper is closed
	 * 
	 * @return <code>true</code> if the gripper is closed, <code>false</code> if
	 *         the gripper is not closed
	 */
	@Deprecated
	public boolean isClosed();

	/**
	 * freezes the gripper
	 * 
	 * @throws ManipulatorException
	 */
	public void freezeGripper();

	/**
	 * receives the sensor data of the gripper
	 * 
	 * @return received data
	 */
	public HashMap<SensorPosition, Integer> receiveGripperSensorData();

	/**
	 * gets the current position of the end-effector
	 * 
	 * @return position value
	 * @throws ManipulatorException
	 */
	public Vector3D getCurrentPosition() throws ManipulatorException;

	/**
	 * gets the current rotation of the end-effector
	 * 
	 * @return rotation value
	 * @throws ManipulatorException
	 */
	public Matrix getCurrentRotation() throws ManipulatorException;

	public ArmError getPosError(Vector3D targetPosition, Matrix targetRotation)
			throws ManipulatorException;

	public boolean isGraspingObject();
	
	public boolean isHome();

}