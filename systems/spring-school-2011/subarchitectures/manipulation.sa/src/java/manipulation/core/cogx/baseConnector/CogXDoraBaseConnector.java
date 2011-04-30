package manipulation.core.cogx.baseConnector;

import manipulation.core.share.Manipulator;
import manipulation.core.share.baseConnector.BaseConnector;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;
import manipulation.math.MathOperation;
import manipulation.runner.cogx.CogXRunner;

import org.apache.log4j.Logger;

import NavData.RobotPose2d;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryChange;

/**
 * represents a connector to the base robot of the CogX system
 * 
 * @author Torben Toeniges
 * 
 */
public class CogXDoraBaseConnector implements BaseConnector {

	private Logger logger = Logger.getLogger(this.getClass());

	private BasePositionData currentPosition;
	private Manipulator manipulator;

	/**
	 * constructor of the CogX base connector
	 * 
	 * @param manipulator
	 *            current manipulator
	 */
	public CogXDoraBaseConnector(Manipulator manipulator) {
		this.manipulator = manipulator;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public BasePositionData getCurrentPosition() throws ExternalMemoryException {
		if (currentPosition == null)
			throw new ExternalMemoryException("No current position available");
		return currentPosition;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void setCurrentPosition(BasePositionData position) {
		this.currentPosition = position;
	}

	/**
	 * listener function which will be called, if the position of the robot
	 * changes
	 * 
	 * @param _wmc
	 *            current working memory change
	 */
	public void robotPositionChanged(WorkingMemoryChange _wmc) {
		try {
			RobotPose2d robotPose = ((CogXRunner) manipulator.getRunner())
					.getMemoryEntry(_wmc.address, RobotPose2d.class);
			BasePositionData newPosition = new BasePositionData(robotPose.x,
					robotPose.y, robotPose.theta);
			setCurrentPosition(newPosition);
		} catch (DoesNotExistOnWMException e) {
			logger.error(e);
		} catch (UnknownSubarchitectureException e) {
			logger.error(e);
		}
	}

//	/**
//	 * {@inheritDoc}
//	 */
//	@Override
//	public Vector3D getRobotToWorldTranslation(Vector3D relativePoint) {
//		Matrix rotMatrix;
//
//		Vector3D globalPose = null;
//		try {
//			rotMatrix = MathOperation.getRotationAroundZ(getCurrentPosition()
//					.getAngle());
//
//			Vector3D newRelCoordinate = MathOperation
//					.getMatrixVectorMultiplication(rotMatrix, relativePoint);
//
//			globalPose = new Vector3D(getCurrentPosition().getPoint().getX()
//					+ newRelCoordinate.getX(), getCurrentPosition().getPoint()
//					.getY() + newRelCoordinate.getY(), newRelCoordinate.getZ());
//
//		} catch (ExternalMemoryException e) {
//			logger.error(e);
//		}
//
//		return globalPose;
//	}
//
//	/**
//	 * {@inheritDoc}
//	 */
//	@Override
//	public Matrix getRobotToWorldRotation(Matrix rotation) {
//		Matrix returnMatrix = null;
//		try {
//			returnMatrix = MathOperation.getMatrixMatrixMultiplication(
//					MathOperation.getRotationAroundZ(getCurrentPosition()
//							.getAngle()), rotation);
//		} catch (ExternalMemoryException e) {
//			logger.error(e);
//		}
//
//		return returnMatrix;
//
//	}

}
