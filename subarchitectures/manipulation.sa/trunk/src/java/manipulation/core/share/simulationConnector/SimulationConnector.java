package manipulation.core.share.simulationConnector;

import golem.tinyice.ArmPrx;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.core.share.exceptions.ViewPointException;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.ViewPoint;
import manipulation.itemMemory.Item;

/**
 * represents a connector to a virtual scene
 * 
 * @author ttoenige
 * 
 */
public interface SimulationConnector {

	/**
	 * moves the robot in the virtual scene but not in the real world
	 * 
	 * @param basePosInVirtualScene
	 *            base position to move the robot to in the virtual scene
	 */
	public void moveRobotInVirtualScene(BasePositionData basePosInVirtualScene);

	/**
	 * updates the best base positions to grasp the given item
	 * 
	 * @param currentPosition
	 *            current position of the robot in the real world
	 * @param item
	 *            corresponding item to grasp
	 * @throws ItemException
	 * @throws InternalMemoryException
	 * @throws ManipulatorException
	 */
	public void updateBestGraspingBasePoints(BasePositionData currentPosition,
			Item item) throws ItemException, InternalMemoryException;

	/**
	 * gets the best grasping point of the base to grasp the object
	 * 
	 * @return best grasping point of the base to grasp the object
	 * @throws GraspingBasePointException
	 */
	public ViewPoint getBestGraspingBasePoint() throws ViewPointException;

	public boolean removeGraspingBasePoint(ViewPoint point)
		;

	/**
	 * gets the initial arm rotation
	 * 
	 * @return initial arm rotation
	 */
	public Matrix getInitArmRotation();

	/**
	 * gets the time in the virtual scene (used to define goals of the arm)
	 * 
	 * @return time in the virtual scene
	 */
	public double getTime();

	/**
	 * gets the arm in the virtual scene
	 * 
	 * @return arm in the virtual scene
	 */
	public ArmPrx getArm();
	
	public void clearScene();

}
