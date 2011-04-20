package manipulation.muster.strategies;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.strategies.parts.StrategyPart.PartName;
import manipulation.muster.strategies.parts.commands.CloseGripperPart;
import manipulation.muster.strategies.parts.commands.FarArmMovementCommandPart;
import manipulation.muster.strategies.parts.commands.LinearGraspApproachCommandPart;
import manipulation.muster.strategies.parts.commands.MoveArmToHomePositionCommandPart;
import manipulation.muster.strategies.parts.commands.OpenGripperPart;
import manipulation.muster.strategies.parts.commands.PutDownCommandPart;
import manipulation.muster.strategies.parts.commands.SimulateGraspCommandPart;
import manipulation.muster.strategies.parts.commands.StopCommandPart;
import manipulation.muster.strategies.parts.commands.WaitPart;
import manipulation.slice.ManipulationCommand;

public class CommandExecution extends Strategy {

	private ManipulationCommand currentCommand = null;

	/**
	 * constructor of the strategy
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 */
	public CommandExecution(Manipulator manipulator) {
		setManipulator(manipulator);
		setName(Name.COMMAND_EXECUTION);
		initParts();
		setFirstPart(getPart(PartName.WAIT_PART));
	}

	/**
	 * @return the currentCommand
	 */
	public ManipulationCommand getCurrentCommand() {
		return currentCommand;
	}

	/**
	 * @param currentCommand
	 *            the currentCommand to set
	 */
	public void setCurrentCommand(ManipulationCommand currentCommand) {
		this.currentCommand = currentCommand;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void initParts() {
		addToPartList(new WaitPart(getManipulator(), this));
		addToPartList(new FarArmMovementCommandPart(getManipulator(), this));
		addToPartList(new PutDownCommandPart(getManipulator(), this));
		addToPartList(new LinearGraspApproachCommandPart(getManipulator(), this));
		addToPartList(new SimulateGraspCommandPart(getManipulator(), this));
		addToPartList(new StopCommandPart(getManipulator(), this));
		addToPartList(new MoveArmToHomePositionCommandPart(getManipulator(),
				this));
		addToPartList(new OpenGripperPart(getManipulator(), this));
		addToPartList(new CloseGripperPart(getManipulator(), this));
	}
}
