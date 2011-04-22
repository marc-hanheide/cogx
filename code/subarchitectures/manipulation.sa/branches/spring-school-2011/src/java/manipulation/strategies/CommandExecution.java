package manipulation.strategies;

import manipulation.core.share.Manipulator;
import manipulation.slice.ManipulationExternalCommand;
import manipulation.strategies.parts.StrategyPart.PartName;
import manipulation.strategies.parts.commands.CloseGripperPart;
import manipulation.strategies.parts.commands.FarArmMovementCommandPart;
import manipulation.strategies.parts.commands.LinearGraspApproachCommandPart;
import manipulation.strategies.parts.commands.MoveArmToHomePositionCommandPart;
import manipulation.strategies.parts.commands.OpenGripperPart;
import manipulation.strategies.parts.commands.PutDownCommandPart;
import manipulation.strategies.parts.commands.SimulateGraspCommandPart;
import manipulation.strategies.parts.commands.StopCommandPart;
import manipulation.strategies.parts.commands.WaitPart;

public class CommandExecution extends Strategy {

	private ManipulationExternalCommand currentCommand = null;

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
	public ManipulationExternalCommand getCurrentCommand() {
		return currentCommand;
	}

	/**
	 * @param currentCommand
	 *            the currentCommand to set
	 */
	public void setCurrentCommand(ManipulationExternalCommand currentCommand) {
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
