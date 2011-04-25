package manipulation.strategies.parts.commands;

import java.util.Observable;
import java.util.Observer;

import org.apache.log4j.Logger;

import manipulation.core.share.Manipulator;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

public class MoveArmToPosePart extends StrategyPart implements Observer {

	private Logger logger = Logger.getLogger(this.getClass());

	public MoveArmToPosePart(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.MOVE_ARM_TO_POSE_PART);
	}

	@Override
	public void changeToNextPart() {
		getManipulator().getWatcher().deleteObserver(this);

		getGlobalStrategy().setNextPart(
				getGlobalStrategy().getPart(getNextPartName()));

	}

	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());
		
		getManipulator().getWatcher().addObserver(this);
	}

	@Override
	public void update(Observable o, Object arg) {

	}
}
