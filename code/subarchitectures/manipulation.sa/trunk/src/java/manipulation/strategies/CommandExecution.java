package manipulation.strategies;

import manipulation.core.share.Manipulator;
import manipulation.strategies.parts.StrategyPart.PartName;
import manipulation.strategies.parts.commands.FarArmMovementCommandPart;
import manipulation.strategies.parts.commands.LinearBaseMovementApproachCommandPart;
import manipulation.strategies.parts.commands.LinearGraspApproachCommandPart;
import manipulation.strategies.parts.commands.PutDownCommandPart;
import manipulation.strategies.parts.commands.SimulateGraspCommandPart;
import manipulation.strategies.parts.commands.WaitPart;

public class CommandExecution extends Strategy {

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
	 * {@inheritDoc}
	 */
	@Override
	public void initParts() {
		addToPartList(new WaitPart(getManipulator(), this));
		addToPartList(new FarArmMovementCommandPart(getManipulator(), this));
		addToPartList(new PutDownCommandPart(getManipulator(), this));
		addToPartList(new LinearGraspApproachCommandPart(getManipulator(), this));
		addToPartList(new SimulateGraspCommandPart(getManipulator(), this));
		addToPartList(new LinearBaseMovementApproachCommandPart(
				getManipulator(), this));
	}
}
