package manipulation.muster.core.share.armConnector;

import java.util.HashMap;

import VisionData.VisualObject;

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
	 */
	public void reach(VisualObject obj);

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
	 * opens the gripper
	 * 
	 */
	public void openGripper();

	/**
	 * closes the gripper with a given force (small values light, high value
	 * strong)
	 * 
	 * @param force
	 *            force value (small values light, high value strong)
	 */
	public void closeGripper(int force);

	/**
	 * freezes the gripper
	 * 
	 */
	public void freezeGripper();


	public void simulate(VisualObject obj);

}