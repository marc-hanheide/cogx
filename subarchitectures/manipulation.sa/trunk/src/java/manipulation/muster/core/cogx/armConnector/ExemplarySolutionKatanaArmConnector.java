package manipulation.muster.core.cogx.armConnector;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.armConnector.ArmConnector;
import manipulation.muster.core.share.exceptions.InternalMemoryException;
import manipulation.muster.core.share.exceptions.ItemException;
import manipulation.muster.core.share.exceptions.ManipulatorException;
import manipulation.muster.itemMemory.Item.PropertyName;
import manipulation.muster.runner.cogx.ExemplarySolutionRunner;
import manipulation.slice.CloseGripperCommand;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.ManipulationCommandStatus;
import manipulation.slice.ManipulationCompletion;
import manipulation.slice.MoveArmToHomePositionCommand;
import manipulation.slice.OpenGripperCommand;
import manipulation.slice.SimulateFarArmMovementCommand;
import manipulation.slice.StopCommand;

import org.apache.log4j.Logger;

import VisionData.VisualObject;
import cast.AlreadyExistsOnWMException;
import cast.cdl.WorkingMemoryAddress;

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

		moveHomeCmd.comp = ManipulationCompletion.COMPINIT;
		moveHomeCmd.status = ManipulationCommandStatus.NEW;

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

		farArmMovementCom.comp = ManipulationCompletion.COMPINIT;
		farArmMovementCom.status = ManipulationCommandStatus.NEW;
		try {
			farArmMovementCom.targetObjectAddr = (WorkingMemoryAddress) manipulator
					.getItemMemory().getFirstGraspItem()
					.getAttribute(PropertyName.WMA_ADDRESS);

			String id = ((ExemplarySolutionRunner) manipulator.getRunner())
					.newDataID();

			((ExemplarySolutionRunner) manipulator.getRunner())
					.addToWorkingMemory(id, farArmMovementCom);
		} catch (ItemException e) {
			logger.error(e);
		} catch (InternalMemoryException e) {
			logger.error(e);
		} catch (AlreadyExistsOnWMException e) {
			logger.error(e);
		}
	}

	@Override
	public void simulate(VisualObject obj) {
		SimulateFarArmMovementCommand simGCmd = new SimulateFarArmMovementCommand();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void stopArm() throws ManipulatorException {
		String id = ((ExemplarySolutionRunner) manipulator.getRunner())
				.newDataID();
		StopCommand stopCommand = new StopCommand();
		stopCommand.comp = ManipulationCompletion.COMPINIT;
		stopCommand.status = ManipulationCommandStatus.NEW;
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
		closeGripperCmd.comp = ManipulationCompletion.COMPINIT;
		closeGripperCmd.status = ManipulationCommandStatus.NEW;
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
		openGripperCmd.comp = ManipulationCompletion.COMPINIT;
		openGripperCmd.status = ManipulationCommandStatus.NEW;
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
