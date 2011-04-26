package manipulation.muster.core.cogx.armConnector;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.armConnector.ArmConnector;
import manipulation.muster.core.share.exceptions.ManipulatorException;
import manipulation.muster.runner.cogx.ExemplarySolutionRunner;
import manipulation.slice.CloseGripperCommand;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.MoveArmToHomePositionCommand;
import manipulation.slice.OpenGripperCommand;
import manipulation.slice.SimulateGraspCommand;
import manipulation.slice.StopCommand;

import org.apache.log4j.Logger;

import VisionData.VisualObject;
import cast.AlreadyExistsOnWMException;

public class ExemplarySolutionKatanaArmConnector implements ArmConnector {

	private Logger logger = Logger.getLogger(this.getClass());

	private Manipulator manipulator;

	/**
	 * constructor of the Birmingham / CogX arm controller
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 */
	public ExemplarySolutionKatanaArmConnector(Manipulator manipulator) {
		this.manipulator = manipulator;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void goHome() throws ManipulatorException {
		String id = ((ExemplarySolutionRunner) manipulator.getRunner())
				.newDataID();
		MoveArmToHomePositionCommand moveHomeCmd = new MoveArmToHomePositionCommand();
		try {
			((ExemplarySolutionRunner) manipulator.getRunner())
					.addToWorkingMemory(id, moveHomeCmd);
		} catch (AlreadyExistsOnWMException e1) {
			logger.error(e1);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void reach(VisualObject obj) {
		FarArmMovementCommand farArmMovementCom = new FarArmMovementCommand();
	}

	@Override
	public void simulate(VisualObject obj) {
		SimulateGraspCommand simGCmd = new SimulateGraspCommand();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void stopArm() throws ManipulatorException {
		String id = ((ExemplarySolutionRunner) manipulator.getRunner())
				.newDataID();
		StopCommand stopCommand = new StopCommand();
		try {
			((ExemplarySolutionRunner) manipulator.getRunner())
					.addToWorkingMemory(id, stopCommand);
		} catch (AlreadyExistsOnWMException e1) {
			logger.error(e1);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void closeGripper(int force) {
		String id = ((ExemplarySolutionRunner) manipulator.getRunner())
				.newDataID();
		CloseGripperCommand closeGripperCmd = new CloseGripperCommand();
		try {
			((ExemplarySolutionRunner) manipulator.getRunner())
					.addToWorkingMemory(id, closeGripperCmd);
		} catch (AlreadyExistsOnWMException e1) {
			logger.error(e1);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void openGripper() {
		String id = ((ExemplarySolutionRunner) manipulator.getRunner())
				.newDataID();
		OpenGripperCommand openGripperCmd = new OpenGripperCommand();
		try {
			((ExemplarySolutionRunner) manipulator.getRunner())
					.addToWorkingMemory(id, openGripperCmd);
		} catch (AlreadyExistsOnWMException e1) {
			logger.error(e1);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void freezeGripper() {

	}
}
