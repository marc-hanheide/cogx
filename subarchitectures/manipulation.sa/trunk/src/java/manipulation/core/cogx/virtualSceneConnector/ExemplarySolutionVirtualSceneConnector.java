package manipulation.core.cogx.virtualSceneConnector;

import golem.tinyice.ArmPrx;
import golem.tinyice.RigidBodyPrx;
import golem.tinyice.TinyPrx;

import java.util.Vector;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.exceptions.ViewPointException;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.ViewPoint;
import manipulation.core.share.virtualSceneConnector.VirtualSceneConnector;
import manipulation.itemMemory.Item;

import org.apache.log4j.Logger;

/**
 * represents a connector to the GOLEM virtual scene (Birmingham / CogX
 * environment)
 * 
 * @author ttoenige
 * 
 */
public class ExemplarySolutionVirtualSceneConnector implements
		VirtualSceneConnector {

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor of the connector to the GOLEM virtual scene
	 * 
	 * @param manipulator
	 *            current manipulator
	 */
	public ExemplarySolutionVirtualSceneConnector(Manipulator manipulator) {

	}

	/**
	 * update the arm in the virtual scene
	 */
	public void updateArm() {
		logger.error("DO NOT USE updateArm!");
	}

	/**
	 * update the robot position in the virtual scene
	 * 
	 * @param position
	 *            new position value
	 * @return new robot representation
	 * @throws Exception
	 */
	public RigidBodyPrx updateRobot(BasePositionData position) throws Exception {
		logger.error("DO NOT USE updateRobot!");
		return null;
	}

	/**
	 * updates an item position in the virtual scene
	 * 
	 * @param item
	 *            item to change its the position
	 * @return new item representation
	 * @throws Exception
	 */
	public RigidBodyPrx updateObstacle(Item item) throws Exception {
		logger.error("DO NOT USE updateObstacle!");
		return null;
	}

	/**
	 * gets the communication interface of GOLEM
	 * 
	 * @return communication interface of GOLEM
	 */
	public TinyPrx getTinyInterface() {
		logger.error("DO NOT USE getTinyInterface!");
		return null;
	}

	/**
	 * sets the communication interface of GOLEM
	 * 
	 * @param tinyInterface
	 *            new communication interface of GOLEM
	 */
	public void setTinyInterface(TinyPrx tinyInterface) {
		logger.error("DO NOT USE setTinyInterface!");
	}

	/**
	 * @return the points
	 */
	public Vector<ViewPoint> getPoints() {
		logger.error("DO NOT USE getPoints!");
		return null;
	}

	/**
	 * gets the robot representation of the virtual scene
	 * 
	 * @return the robot representation of the virtual scene
	 */
	public RigidBodyPrx getRobot() {
		logger.error("DO NOT USE getRobot!");
		return null;
	}

	/**
	 * sets the robot representation
	 * 
	 * @param robot
	 *            new robot representation
	 */
	public void setRobot(RigidBodyPrx robot) {
		logger.error("DO NOT USE setRobot!");
	}

	/**
	 * gets all obstacle / item representations
	 * 
	 * @return all obstacle / item representations
	 */
	public Vector<RigidBodyPrx> getObstacles() {
		logger.error("DO NOT USE getObstacles!");
		return null;
	}

	/**
	 * sets all obstacle / item representations
	 * 
	 * @param obstacles
	 *            new obstacle / item representations
	 */
	public void setObstacles(Vector<RigidBodyPrx> obstacles) {
		logger.error("DO NOT USE setObstacles!");
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Matrix getInitArmRotation() {
		logger.error("DO NOT USE getInitArmRotation!");
		return null;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public double getTime() {
		logger.error("DO NOT USE getTime!");
		return 0;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public ArmPrx getArm() {
		logger.error("DO NOT USE getArm!");
		return null;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void moveRobotInVirtualScene(BasePositionData basePosInVirtualScene) {
		logger.error("DO NOT USE moveRobotInVirtualScene!");
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public boolean removeGraspingBasePoint(ViewPoint point) {
		logger.error("DO NOT USE removeGraspingBasePoint!");
		return false;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void updateBestGraspingBasePoints(BasePositionData currentPosition,
			Item item) throws ItemException, InternalMemoryException {
		logger.error("DO NOT USE updateBestGraspingBasePoints!");
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public ViewPoint getBestGraspingBasePoint() throws ViewPointException {
		logger.error("DO NOT USE removeGraspingBasePoint!");
		return null;
	}

	@Override
	public void clearScene() {
		logger.error("DO NOT USE clearScene!");
	}

	public void addBox(Matrix rotation) {
		logger.error("DO NOT USE addBox!");
	}
}
