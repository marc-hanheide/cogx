package manipulation.core.bham.baseConnector;

import manipulation.core.bham.simulationConnector.BhamSimulationConnector;
import manipulation.core.share.Manipulator;
import manipulation.core.share.baseConnector.BaseConnector;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;
import manipulation.itemMemory.ItemMemory.ReachingStatus;
import manipulation.math.MathOperation;
import manipulation.runner.bham.BhamRunner;

import org.apache.log4j.Logger;

import NavData.RobotPose2d;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialData.Priority;
import SpatialData.StatusError;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;

/**
 * represents a connector to the base robot of the Birmingham / CogX system
 * 
 * @author ttoenige
 * 
 */
public class BhamDoraBaseConnector implements BaseConnector {

	private Logger logger = Logger.getLogger(this.getClass());
	
	private BasePositionData currentPosition;
	private Manipulator manipulator;

	private boolean moving = false;

	private WorkingMemoryChangeReceiver wmcr;

	private boolean exploring = false;

	/**
	 * constructor of the Birmingham / CogX base connector
	 * 
	 * @param manipulator
	 *            current manipulator
	 */
	public BhamDoraBaseConnector(Manipulator manipulator) {
		this.manipulator = manipulator;
	}

	private static NavCommand newNavCommand() {
		return new NavCommand(CommandType.GOTOPOSITION, Priority.URGENT, null,
				null, null, null, null, StatusError.NONE,
				Completion.COMMANDPENDING);
	}

	private void navCommandChanged(WorkingMemoryChange _wmc,
			WorkingMemoryChangeReceiver wmcr) {
		logger.debug("navCommand Changed");
		try {
			NavCommand command = ((BhamRunner) manipulator.getRunner())
					.getMemoryEntry(_wmc.address, NavCommand.class);
			logger.error("COMPLETION::" + command.comp);
			logger.error(command.cmd);
			logger.error("STATUS::" + command.status);

			// TODO andere faelle abfangen?
			switch (command.comp) {
			// case COMMANDINPROGRESS:
			// logger.debug("Command in Progess");
			// manipulator.getItemMemory().updateViewPointReachingStatus(
			// ReachingStatus.ON_THE_WAY);
			case COMMANDFAILED:
				logger.debug("Command Failed!");
				stop();
				setMoving(false);
				manipulator.getItemMemory().updateViewPointReachingStatus(
						ReachingStatus.VP_NOT_REACHABLE);

				((BhamRunner) manipulator.getRunner()).removeChangeFilter(wmcr);

				break;
			case COMMANDABORTED:
				logger.debug("Command Aborted");
				stop();
				setMoving(false);

				manipulator.getItemMemory().updateViewPointReachingStatus(
						ReachingStatus.OTHER);

				((BhamRunner) manipulator.getRunner()).removeChangeFilter(wmcr);
				break;
			case COMMANDSUCCEEDED:
				if ((command.cmd == CommandType.GOTOPOSITION)) {
					logger.debug("Command successful");
					setMoving(false);
					manipulator.getItemMemory().updateViewPointReachingStatus(
							ReachingStatus.VP_REACHED);
				}
				break;
			default:
				break;
			}

		} catch (DoesNotExistOnWMException e) {
			logger.error(e);
		} catch (CASTException e) {
			logger.error(e);
		} catch (InternalMemoryException e) {
			logger.error(e);
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void goTo(BasePositionData target) throws ExternalMemoryException {
		stop();
		logger.debug("BhamDoraBaseConnector goto");
		try {
			manipulator.getItemMemory().updateViewPointReachingStatus(
					ReachingStatus.ON_THE_WAY);
		} catch (InternalMemoryException e1) {
			logger.error(e1);
		}

		NavCommand nav = newNavCommand();
		nav.cmd = CommandType.GOTOPOSITION;
		double[] position = new double[3];
		position[0] = target.getPoint().getX();
		position[1] = target.getPoint().getY();
		position[2] = target.getAngle();
		nav.pose = position;
		double[] tolerance = new double[3];
		tolerance[0] = 0.05;
		tolerance[1] = 0.05;
		tolerance[2] = Math.PI / 36; // 5 grad
		nav.tolerance = tolerance;

		String id = ((BhamRunner) manipulator.getRunner()).newDataID();

		try {
			((BhamRunner) manipulator.getRunner()).addToWorkingMemory(id, nav);
		} catch (CASTException e) {
			throw new ExternalMemoryException(e.message);

		}

		wmcr = new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				navCommandChanged(_wmc, wmcr);
			}
		};

		setMoving(true);

		((BhamRunner) manipulator.getRunner()).addChangeFilter(
				ChangeFilterFactory.createIDFilter(id), wmcr);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public boolean isExploring() {
		return exploring;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void startExplore() {
		logger.debug("explore not implemented yet");

//		stop();
//
//		NavCommand nav = newNavCommand();
//		nav.cmd = CommandType.EXPLORE;
//
//		String id = ((BhamRunner) manipulator.getRunner()).newDataID();
//
//		try {
//			((BhamRunner) manipulator.getRunner()).addToWorkingMemory(id, nav);
//		} catch (AlreadyExistsOnWMException e) {
//			logger.error(e);
//		}
//
//		setMoving(true);
//		exploring = true;

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void stop() {
		NavCommand nav = newNavCommand();
		nav.cmd = CommandType.STOP;

		String id = ((BhamRunner) manipulator.getRunner()).newDataID();

		try {
			((BhamRunner) manipulator.getRunner()).addToWorkingMemory(id, nav);
		} catch (AlreadyExistsOnWMException e) {
			logger.error(e);
		}

		setMoving(false);
		exploring = false;

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

	public void robotPositionChanged(WorkingMemoryChange _wmc) {
		try {
			RobotPose2d robotPose = ((BhamRunner) manipulator.getRunner())
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

	/**
	 * {@inheritDoc}
	 */
	@Override
	public boolean isMoving() {
		return moving;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void setMoving(boolean movingIn) {
		if (movingIn) {
			((BhamRunner) manipulator.getRunner()).addBaseMovementListener();
//			((BhamSimulationConnector) manipulator.getSimulationConnector())
//					.startPosThread();
			manipulator.getMapConnector().updateMapConstant();
		} else {

			((BhamRunner) manipulator.getRunner()).removeMovementListener();
//			((BhamSimulationConnector) manipulator.getSimulationConnector())
//					.stopPosThread();
			manipulator.getMapConnector().stopupdateMapConstant();
		}

		this.moving = movingIn;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Vector3D getRobotToWorldTranslation(Vector3D relativePoint) {
		Matrix rotMatrix;

		Vector3D globalPose = null;
		try {
			rotMatrix = MathOperation.getRotationAroundZ(getCurrentPosition()
					.getAngle());

			Vector3D newRelCoordinate = MathOperation
					.getMatrixVectorMultiplication(rotMatrix, relativePoint);

			globalPose = new Vector3D(getCurrentPosition().getPoint().getX()
					+ newRelCoordinate.getX(), getCurrentPosition().getPoint()
					.getY()
					+ newRelCoordinate.getY(), newRelCoordinate.getZ());

		} catch (ExternalMemoryException e) {
			logger.error(e);
		}

		return globalPose;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Matrix getRobotToWorldRotation(Matrix rotation) {
		Matrix returnMatrix = null;
		try {
			returnMatrix = MathOperation.getMatrixMatrixMultiplication(
					MathOperation.getRotationAroundZ(getCurrentPosition()
							.getAngle()), rotation);
		} catch (ExternalMemoryException e) {
			logger.error(e);
		}

		return returnMatrix;

	}

}
