package manipulation.strategies;

import org.apache.log4j.Logger;

import manipulation.core.share.Manipulator;
import manipulation.slice.ManipulationExternalCommand;
import manipulation.strategies.parts.StrategyPart.PartName;
import manipulation.strategies.parts.commands.CloseGripperPart;
import manipulation.strategies.parts.commands.FarArmMovementCommandPart;
import manipulation.strategies.parts.commands.FineArmMovementCommandPart;
import manipulation.strategies.parts.commands.GetCurrentArmPosePart;
import manipulation.strategies.parts.commands.MoveArmToHomePositionCommandPart;
import manipulation.strategies.parts.commands.MoveArmToPosePart;
import manipulation.strategies.parts.commands.OpenGripperPart;
import manipulation.strategies.parts.commands.PutDownCommandPart;
import manipulation.strategies.parts.commands.SimulateFarArmMovGraspCommandPart;
import manipulation.strategies.parts.commands.SimulateMoveToPosePart;
import manipulation.strategies.parts.commands.StopCommandPart;
import manipulation.strategies.parts.commands.WaitPart;

public class CommandExecution extends Strategy {

	private Logger logger = Logger.getLogger(this.getClass());

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
		logger.error("CURRENT COMMAND STRATEǴY" + currentCommand.getClass());
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
		addToPartList(new FineArmMovementCommandPart(getManipulator(), this));
		addToPartList(new SimulateFarArmMovGraspCommandPart(getManipulator(),
				this));
		addToPartList(new StopCommandPart(getManipulator(), this));
		addToPartList(new MoveArmToHomePositionCommandPart(getManipulator(),
				this));
		addToPartList(new OpenGripperPart(getManipulator(), this));
		addToPartList(new CloseGripperPart(getManipulator(), this));
		addToPartList(new MoveArmToPosePart(getManipulator(), this));
		addToPartList(new GetCurrentArmPosePart(getManipulator(), this));
		addToPartList(new SimulateMoveToPosePart(getManipulator(), this));

	}
}
