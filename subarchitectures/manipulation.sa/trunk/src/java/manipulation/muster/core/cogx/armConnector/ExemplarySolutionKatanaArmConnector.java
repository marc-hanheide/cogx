package manipulation.muster.core.cogx.armConnector;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.armConnector.ArmConnector;
import manipulation.muster.core.share.exceptions.InternalMemoryException;
import manipulation.muster.core.share.exceptions.ItemException;
import manipulation.muster.itemMemory.Item.PropertyName;
import manipulation.muster.runner.cogx.ExemplarySolutionRunner;
import manipulation.slice.CloseGripperCommand;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.FineArmMovementCommand;
import manipulation.slice.ManipulationCommand;
import manipulation.slice.ManipulationCommandStatus;
import manipulation.slice.ManipulationCompletion;
import manipulation.slice.MoveArmToHomePositionCommand;
import manipulation.slice.OpenGripperCommand;
import manipulation.slice.SimulateFarArmMovementCommand;
import manipulation.slice.StopCommand;

import org.apache.log4j.Logger;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;

public class ExemplarySolutionKatanaArmConnector implements ArmConnector {

	private Logger logger = Logger.getLogger(this.getClass());

	private Manipulator manipulator;

	private WorkingMemoryChangeReceiver wmcr;

	/**
	 * constructor of the Birmingham / CogX arm controller
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 */
	public ExemplarySolutionKatanaArmConnector(Manipulator manipulator) {
		this.manipulator = manipulator;
	}

	private void manCommandChanged(WorkingMemoryChange _wmc,
			WorkingMemoryChangeReceiver wmcr) {
		logger.debug("manCommandChanged Changed");

		try {
			ManipulationCommand command = ((ExemplarySolutionRunner) manipulator
					.getRunner()).getMemoryEntry(_wmc.address,
					ManipulationCommand.class);

			if (command.comp == ManipulationCompletion.SUCCEEDED) {

				manipulator.getItemMemory().updateManipulationStatus(command);

				((ExemplarySolutionRunner) manipulator.getRunner())
						.removeChangeFilter(wmcr);
			} else {
				logger.error("COMP: " + command.comp + "-> not SUCCEEDED");
			}

		} catch (DoesNotExistOnWMException e) {
			logger.error(e);
		} catch (UnknownSubarchitectureException e) {
			logger.error(e);
		} catch (SubarchitectureComponentException e) {
			logger.error(e);
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void goHome() {
		String id = ((ExemplarySolutionRunner) manipulator.getRunner())
				.newDataID();
		MoveArmToHomePositionCommand moveHomeCmd = new MoveArmToHomePositionCommand();

		moveHomeCmd.comp = ManipulationCompletion.COMPINIT;
		moveHomeCmd.status = ManipulationCommandStatus.NEW;

		try {
			((ExemplarySolutionRunner) manipulator.getRunner())
					.addToWorkingMemory(id, moveHomeCmd);

			wmcr = new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					manCommandChanged(_wmc, wmcr);
				}
			};

			((ExemplarySolutionRunner) manipulator.getRunner())
					.addChangeFilter(ChangeFilterFactory.createIDFilter(id),
							wmcr);

		} catch (AlreadyExistsOnWMException e1) {
			logger.error(e1);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void reach() {
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

			wmcr = new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					manCommandChanged(_wmc, wmcr);
				}
			};

			((ExemplarySolutionRunner) manipulator.getRunner())
					.addChangeFilter(ChangeFilterFactory.createIDFilter(id),
							wmcr);

		} catch (ItemException e) {
			logger.error(e);
		} catch (InternalMemoryException e) {
			logger.error(e);
		} catch (AlreadyExistsOnWMException e) {
			logger.error(e);
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void reachFine() {
		FineArmMovementCommand fineArmMovCmd = new FineArmMovementCommand();

		fineArmMovCmd.comp = ManipulationCompletion.COMPINIT;
		fineArmMovCmd.status = ManipulationCommandStatus.NEW;
		try {
			fineArmMovCmd.targetObjectAddr = (WorkingMemoryAddress) manipulator
					.getItemMemory().getFirstGraspItem()
					.getAttribute(PropertyName.WMA_ADDRESS);

			String id = ((ExemplarySolutionRunner) manipulator.getRunner())
					.newDataID();

			((ExemplarySolutionRunner) manipulator.getRunner())
					.addToWorkingMemory(id, fineArmMovCmd);

			wmcr = new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					manCommandChanged(_wmc, wmcr);
				}
			};

			((ExemplarySolutionRunner) manipulator.getRunner())
					.addChangeFilter(ChangeFilterFactory.createIDFilter(id),
							wmcr);

		} catch (ItemException e) {
			logger.error(e);
		} catch (InternalMemoryException e) {
			logger.error(e);
		} catch (AlreadyExistsOnWMException e) {
			logger.error(e);
		}

	}

	@Override
	public void simulate() {
		SimulateFarArmMovementCommand simCmd = new SimulateFarArmMovementCommand();
		simCmd.comp = ManipulationCompletion.COMPINIT;
		simCmd.status = ManipulationCommandStatus.NEW;

		try {
			simCmd.targetObjectAddr = (WorkingMemoryAddress) manipulator
					.getItemMemory().getFirstGraspItem()
					.getAttribute(PropertyName.WMA_ADDRESS);

			String id = ((ExemplarySolutionRunner) manipulator.getRunner())
					.newDataID();

			((ExemplarySolutionRunner) manipulator.getRunner())
					.addToWorkingMemory(id, simCmd);

			wmcr = new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					manCommandChanged(_wmc, wmcr);
				}
			};

			((ExemplarySolutionRunner) manipulator.getRunner())
					.addChangeFilter(ChangeFilterFactory.createIDFilter(id),
							wmcr);

		} catch (ItemException e) {
			logger.error(e);
		} catch (InternalMemoryException e) {
			logger.error(e);
		} catch (AlreadyExistsOnWMException e) {
			logger.error(e);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void stopArm() {
		String id = ((ExemplarySolutionRunner) manipulator.getRunner())
				.newDataID();
		StopCommand stopCommand = new StopCommand();
		stopCommand.comp = ManipulationCompletion.COMPINIT;
		stopCommand.status = ManipulationCommandStatus.NEW;
		try {
			((ExemplarySolutionRunner) manipulator.getRunner())
					.addToWorkingMemory(id, stopCommand);

			wmcr = new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					manCommandChanged(_wmc, wmcr);
				}
			};

			((ExemplarySolutionRunner) manipulator.getRunner())
					.addChangeFilter(ChangeFilterFactory.createIDFilter(id),
							wmcr);

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

			wmcr = new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					manCommandChanged(_wmc, wmcr);
				}
			};

			((ExemplarySolutionRunner) manipulator.getRunner())
					.addChangeFilter(ChangeFilterFactory.createIDFilter(id),
							wmcr);

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

			wmcr = new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					manCommandChanged(_wmc, wmcr);
				}
			};

			((ExemplarySolutionRunner) manipulator.getRunner())
					.addChangeFilter(ChangeFilterFactory.createIDFilter(id),
							wmcr);

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
